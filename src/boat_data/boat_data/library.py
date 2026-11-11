"""On-disk recording catalogue. Names never become filesystem paths."""
import hashlib
import json
import re
from pathlib import Path


def recording_library(root, active_id=None, limit=100):
    rows = []
    for path in sorted(Path(root).glob('*/manifest.json'), reverse=True):
        if path.is_symlink() or path.parent.is_symlink():
            continue
        try:
            m = json.loads(path.read_text(encoding='utf-8'))
            quality = m.get('quality', {})
            ended = bool(m.get('ended_utc'))
            state = 'recording' if path.parent.name == active_id else 'stopped' if ended else 'incomplete'
            if ended and (quality.get('forced_shutdown') or quality.get('unexpected_exit')
                          or not quality.get('write_confirmed')):
                state = 'needs review'
            rows.append({'run_id': path.parent.name, 'name': m.get('display_name', ''),
                         'started_utc': m.get('started_utc'), 'duration_s': m.get('duration_s'),
                         'bytes': quality.get('bytes'), 'state': state,
                         'can_name': ended and path.parent.name != active_id,
                         'folder': str(path.parent), 'site': m.get('site', '')})
        except (OSError, ValueError, TypeError, AttributeError):
            rows.append({'run_id': path.parent.name, 'name': '', 'state': 'unreadable manifest', 'can_name': False})
        if len(rows) >= limit:
            break
    return rows


def name_recording(root, run_id, name, active_id=None):
    if not isinstance(run_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,160}', run_id):
        raise ValueError('Invalid recording ID')
    if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80 or any(ord(c) < 32 for c in name):
        raise ValueError('Use a name of 1–80 characters without control characters')
    root = Path(root).resolve()
    folder = root / run_id
    if folder.is_symlink() or folder.resolve().parent != root:
        raise ValueError('Invalid recording folder')
    path = folder / 'manifest.json'
    if path.is_symlink():
        raise ValueError('Invalid manifest path')
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if run_id == active_id or not manifest.get('ended_utc'):
        raise ValueError('Stop the recording before saving its name')
    manifest['display_name'] = name.strip()
    raw = json.dumps(manifest, indent=2).encode('utf-8')
    tmp = folder / 'manifest.json.tmp'
    tmp.write_bytes(raw)
    tmp.replace(path)
    # Update only the changed manifest hash, without re-reading gigabytes of MCAP.
    checksums = folder / 'checksums.sha256'
    if checksums.is_symlink():
        raise ValueError('Invalid checksum path')
    if checksums.exists():
        lines = [line for line in checksums.read_text(encoding='utf-8').splitlines()
                 if len(line.split('  ', 1)) == 2 and line.split('  ', 1)[1] != 'manifest.json']
        lines.append(hashlib.sha256(raw).hexdigest() + '  manifest.json')
        checksum_tmp = folder / 'checksums.sha256.tmp'
        checksum_tmp.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        checksum_tmp.replace(checksums)
    return manifest['display_name']
