"""Copy one stopped run from the Pi; verify capture hashes before publishing locally.
Requires OpenSSH scp. No deletion, cloud upload, or propulsion commands.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import uuid


def verify_run(folder):
    folder = Path(folder)
    if any(p.is_symlink() for p in folder.rglob('*')):
        raise ValueError('Symlinks are not allowed in a recording archive')
    manifest = json.loads((folder / 'manifest.json').read_text(encoding='utf-8'))
    if not manifest.get('ended_utc'):
        raise ValueError('Recording is not finalized')
    checked = set()
    for line in (folder / 'checksums.sha256').read_text(encoding='utf-8').splitlines():
        digest, name = line.split('  ', 1)
        name = name.replace('\\', '/')
        rel = PurePosixPath(name)
        if rel.is_absolute() or '..' in rel.parts or ':' in name or not re.fullmatch('[0-9a-f]{64}', digest):
            raise ValueError('Invalid checksum entry')
        path = folder / name
        h = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                h.update(block)
        if h.hexdigest() != digest:
            raise ValueError('Checksum mismatch: ' + name)
        checked.add(name)
    actual = {p.relative_to(folder).as_posix() for p in folder.rglob('*') if p.is_file() and p.name != 'checksums.sha256'}
    if checked != actual or 'manifest.json' not in checked or not any(n.endswith('.mcap') for n in checked):
        raise ValueError('Missing checksums or no MCAP data')
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', required=True)
    ap.add_argument('--user', required=True)
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--destination', required=True, help='Local recordings library folder')
    ap.add_argument('--remote-root', default='asv-boat/data/runs', help='Relative to Pi user home, or absolute path')
    args = ap.parse_args()
    for value, pattern in [(args.host, r'[A-Za-z0-9][A-Za-z0-9.-]*'),
                           (args.user, r'[A-Za-z_][A-Za-z0-9_-]*'),
                           (args.run_id, r'[A-Za-z0-9_-]{1,160}'),
                           (args.remote_root, r'/?[A-Za-z0-9_/-]+')]:
        if not re.fullmatch(pattern, value) or '..' in value:
            ap.error('Invalid host, user, run ID or remote path')
    root = Path(args.destination).resolve()
    root.mkdir(parents=True, exist_ok=True)
    final = root / args.run_id
    if final.exists():
        ap.error('Destination recording already exists; it will not be overwritten')
    staging = root / ('.partial-' + uuid.uuid4().hex)
    staging.mkdir()
    remote = f'{args.user}@{args.host}:{args.remote_root.rstrip("/")}/{args.run_id}'
    subprocess.run(['scp', '-r', remote, str(staging)], check=True)
    copied = staging / args.run_id
    manifest = verify_run(copied)
    copied.rename(final)
    staging.rmdir()
    print('Verified recording: ' + str(final))
    print('Name: ' + str(manifest.get('display_name', '(unnamed)')))
    print('Import the MCAP files under bag/rosbag2 into Foxglove. Source remains on the Pi.')


if __name__ == '__main__':
    main()
