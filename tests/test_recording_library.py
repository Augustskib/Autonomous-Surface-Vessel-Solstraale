import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'boat_data'))
from boat_data.library import name_recording, recording_library
spec=importlib.util.spec_from_file_location('fetch_recording',Path(__file__).resolve().parents[1]/'scripts/comms/fetch-recording.py')
fetch=importlib.util.module_from_spec(spec);spec.loader.exec_module(fetch)


def run(root):
    folder=root/'20261111_site_run001';folder.mkdir()
    (folder/'manifest.json').write_text(json.dumps({'ended_utc':'today','quality':{'write_confirmed':True,'bytes':7}}))
    (folder/'capture.mcap').write_bytes(b'example')
    (folder/'checksums.sha256').write_text('\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name for p in [folder/'manifest.json',folder/'capture.mcap'])+'\n')
    return folder


def test_name_survives_restart_and_preserves_capture_checksums(tmp_path):
    folder=run(tmp_path);original=(folder/'capture.mcap').read_bytes()
    name_recording(tmp_path,folder.name,'Morning lake test')
    assert recording_library(tmp_path)[0]['name']=='Morning lake test'
    assert fetch.verify_run(folder)['display_name']=='Morning lake test'
    assert (folder/'capture.mcap').read_bytes()==original


@pytest.mark.parametrize('bad',['../escape','/absolute','a/b',''])
def test_name_cannot_address_outside_library(tmp_path,bad):
    with pytest.raises(ValueError):name_recording(tmp_path,bad,'test')


def test_cannot_name_active_or_unfinished_run(tmp_path):
    folder=run(tmp_path)
    with pytest.raises(ValueError):name_recording(tmp_path,folder.name,'test',folder.name)
    (folder/'manifest.json').write_text('{}')
    with pytest.raises(ValueError):name_recording(tmp_path,folder.name,'test')
    assert recording_library(tmp_path)[0]['state']=='incomplete'


def test_transfer_rejects_corruption_and_unhashed_files(tmp_path):
    folder=run(tmp_path)
    (folder/'unhashed.txt').write_text('oops')
    with pytest.raises(ValueError):fetch.verify_run(folder)
    (folder/'unhashed.txt').unlink()
    (folder/'capture.mcap').write_bytes(b'bad')
    with pytest.raises(ValueError,match='mismatch'):fetch.verify_run(folder)


def test_catalog_exposes_failed_recordings(tmp_path):
    folder=run(tmp_path)
    (folder/'manifest.json').write_text(json.dumps({'ended_utc':'today','quality':{'unexpected_exit':True}}))
    assert recording_library(tmp_path)[0]['state']=='needs review'
    (folder/'manifest.json').write_text('{broken')
    assert recording_library(tmp_path)[0]['state']=='unreadable manifest'
