"""Offline five-stream MCAP indexing, review export and verified archive copy.

Raw capture and its checksums are immutable. Derived outputs are versioned.
Memory contains one decoded message at a time; metadata lives in SQLite.
"""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import shutil
import sqlite3
import uuid

TOPICS = {'camera': '/boat/camera/image_raw', 'gps': '/boat/gps/fix',
          'imu': '/boat/imu', 'thrust': '/boat/thrust_command',
          'lidar': '/boat/lidar/points_raw'}
TOLERANCE_NS = {'gps': 200_000_000, 'imu': 50_000_000,
                'thrust': 500_000_000, 'lidar': 150_000_000}


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def discover(path):
    path = Path(path).resolve()
    if path.is_file():
        if path.suffix != '.mcap':
            raise ValueError('Expected an MCAP file')
        root = next((p for p in path.parents if (p / 'manifest.json').is_file()), path.parent)
        return root, [path]
    root = path
    base = root / 'bag' if (root / 'bag').is_dir() else root
    # Natural ordering keeps segment 10 after segment 9, not after segment 1.
    key = lambda p: [int(x) if x.isdigit() else x for x in re.split(r'(\d+)', str(p))]
    bags = sorted(base.rglob('*.mcap'), key=key)
    if not bags:
        raise FileNotFoundError(f'No MCAP segments under {base}')
    return root, bags


def decoded(path):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    with Path(path).open('rb') as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        # Physical message order is required for detecting timestamp reversal.
        yield from reader.iter_decoded_messages(log_time_order=False)


def timestamp(msg, log_ns):
    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
    if stamp is not None:
        ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if ns > 0:
            return ns, 'header'
    return log_ns, 'receive'


def valid_sample(kind, msg):
    if kind == 'gps':
        status = getattr(msg, 'status', None)
        status = getattr(status, 'status', status)
        return (status is not None and 0 <= status < 255
                and math.isfinite(msg.latitude) and -90 <= msg.latitude <= 90
                and math.isfinite(msg.longitude) and -180 <= msg.longitude <= 180)
    if kind == 'thrust':
        return all(math.isfinite(v) and abs(v) <= 100 for v in (msg.left, msg.right))
    if kind == 'lidar':
        return msg.width * msg.height > 0 and len(msg.data) == msg.row_step * msg.height
    if kind == 'camera':
        return msg.width > 0 and msg.height > 0 and len(msg.data) >= msg.step * msg.height
    if kind == 'imu':
        return (msg.angular_velocity_covariance[0] != -1 and
                all(math.isfinite(getattr(msg.angular_velocity, a)) for a in ('x', 'y', 'z')))
    return True


def match(db, kind, ns):
    before = db.execute('SELECT id,t,valid,basis FROM sample WHERE kind=? AND t<=? ORDER BY t DESC,id DESC LIMIT 1',
                        (kind, ns)).fetchone()
    # Commands use causal sample-and-hold, never the future command.
    if kind == 'thrust':
        return before
    after = db.execute('SELECT id,t,valid,basis FROM sample WHERE kind=? AND t>=? ORDER BY t,id LIMIT 1',
                       (kind, ns)).fetchone()
    options = [r for r in (before, after) if r is not None]
    return min(options, key=lambda r: abs(r[1]-ns)) if options else None


def index_run(path, required=('camera', 'gps', 'imu', 'thrust', 'lidar')):
    root, bags = discover(path)
    output = root / 'derived' / ('dataset-v1-' + uuid.uuid4().hex[:12])
    output.mkdir(parents=True)
    db = sqlite3.connect(output / 'index.sqlite')
    db.execute('CREATE TABLE sample(id INTEGER PRIMARY KEY,kind TEXT,t INTEGER,log_ns INTEGER,basis TEXT,valid INTEGER,bag TEXT,ordinal INTEGER,frame TEXT)')
    counts, reversals, missing_stamps = {}, {}, {}
    previous = {}
    lidar_clock = (False, 0)
    source_files = []
    try:
        for bag in bags:
            relative = bag.relative_to(root).as_posix()
            source_files.append({'path': relative, 'sha256': sha256(bag), 'bytes': bag.stat().st_size})
            for ordinal, (_, channel, message, msg) in enumerate(decoded(bag)):
                if channel.topic == '/boat/lidar/driver_status':
                    try:
                        status = json.loads(msg.data)
                        lidar_clock = (status.get('timestamp_mode') == 'sensor' and status.get('clock_verified') is True,
                                       message.log_time)
                    except (ValueError, AttributeError):
                        lidar_clock = (False, 0)
                    continue
                kind = next((k for k, t in TOPICS.items() if t == channel.topic), None)
                if kind is None:
                    continue
                ns, basis = timestamp(msg, message.log_time)
                if kind == 'lidar' and not (lidar_clock[0] and 0 <= message.log_time-lidar_clock[1] < 2500000000):
                    basis = 'unverified_lidar_clock'
                counts[kind] = counts.get(kind, 0) + 1
                if kind in previous and ns < previous[kind]:
                    reversals[kind] = reversals.get(kind, 0) + 1
                previous[kind] = ns
                if basis != 'header' and kind != 'thrust':
                    missing_stamps[kind] = missing_stamps.get(kind, 0) + 1
                frame = getattr(getattr(msg, 'header', None), 'frame_id', '')
                db.execute('INSERT INTO sample VALUES(NULL,?,?,?,?,?,?,?,?)',
                           (kind, ns, message.log_time, basis, int(valid_sample(kind, msg)), relative, ordinal, frame))
            db.commit()
        db.execute('CREATE INDEX sample_time ON sample(kind,t)')
        db.execute('CREATE INDEX sample_source ON sample(bag,ordinal)')
        db.execute('CREATE TABLE sync(camera_id INTEGER PRIMARY KEY,accepted INTEGER,matches TEXT,reasons TEXT)')
        accepted, frames = 0, 0
        for camera_id, ns, valid, basis in db.execute("SELECT id,t,valid,basis FROM sample WHERE kind='camera' ORDER BY t,id"):
            frames += 1
            reasons = [] if valid and basis == 'header' else ['invalid camera or missing acquisition stamp']
            associations = {}
            for kind in ('gps', 'imu', 'thrust', 'lidar'):
                row = match(db, kind, ns)
                associations[kind] = None if row is None else {'sample_id': row[0], 'offset_ns': row[1]-ns, 'time_basis': row[3]}
                if kind in required and (row is None or not row[2] or abs(row[1]-ns) > TOLERANCE_NS[kind]
                                         or (kind != 'thrust' and row[3] != 'header')):
                    reasons.append(f'{kind}: missing, invalid, unstamped or outside tolerance')
            ok = not reasons
            accepted += int(ok)
            db.execute('INSERT INTO sync VALUES(?,?,?,?)', (camera_id, int(ok), json.dumps(associations), json.dumps(reasons)))
        db.commit()
        missing = [k for k in required if not counts.get(k)]
        report = {'schema_version': 1, 'required': list(required), 'topics': TOPICS,
                  'counts': counts, 'missing': missing, 'timestamp_reversals': reversals,
                  'missing_acquisition_stamps': missing_stamps, 'camera_frames': frames,
                  'accepted_frames': accepted, 'tolerances_ns': TOLERANCE_NS,
                  'pass': bool(frames and accepted == frames and not missing
                               and not any(reversals.get(k) or missing_stamps.get(k) for k in required)),
                  'sources': source_files,
                  'limitations': ['Nearest scan is not per-point deskew or hardware synchronization.',
                                  'Thrust is requested duty, not measured force or applied PWM.',
                                  'Quality checks are necessary, not proof of calibration or sensor accuracy.']}
        (output / 'quality_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        return output, report
    finally:
        db.close()


def resolve_index(index_dir):
    path = Path(index_dir).resolve()
    if path.is_file() and path.name == 'index.sqlite':
        path = path.parent
    if not (path / 'index.sqlite').is_file() or not (path / 'quality_report.json').is_file():
        raise ValueError('Expected an index directory or its index.sqlite file')
    return path


def export_images(root, index_dir, every=10):
    from PIL import Image
    root, index_dir = Path(root).resolve(), resolve_index(index_dir)
    if every < 1:
        raise ValueError('every must be positive')
    report = json.loads((index_dir / 'quality_report.json').read_text())
    if not report['accepted_frames']:
        raise ValueError('No accepted synchronized frames to export')
    if any(report['timestamp_reversals'].get(k) for k in report['required']):
        raise ValueError('Resolve clock reversals before export')
    for source in report['sources']:
        path = (root / source['path']).resolve()
        if not path.is_relative_to(root) or sha256(path) != source['sha256']:
            raise ValueError('Source path or checksum differs from indexed capture')
    out = index_dir / ('roboflow-review-' + uuid.uuid4().hex[:8])
    (out / 'images').mkdir(parents=True)
    # A run is indivisible across train/validation/test. Split assignment is
    # left to a reviewed dataset manifest, never random adjacent-frame splitting.
    db = sqlite3.connect(index_dir / 'index.sqlite')
    try:
        with (out / 'provenance.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['image', 'run', 'camera_sample_id', 'acquisition_ns', 'associations', 'split_group'])
            selected = 0
            for source in report['sources']:
                bag = source['path']
                for ordinal, (_, channel, _, msg) in enumerate(decoded(root / bag)):
                    if channel.topic != TOPICS['camera']:
                        continue
                    row = db.execute('SELECT s.id,s.t,y.matches FROM sample s JOIN sync y ON y.camera_id=s.id WHERE s.bag=? AND s.ordinal=? AND y.accepted=1', (bag, ordinal)).fetchone()
                    if row is None:
                        continue
                    selected += 1
                    if (selected - 1) % every:
                        continue
                    enc = msg.encoding.lower()
                    modes = {'rgb8': ('RGB', 'RGB', 3), 'bgr8': ('RGB', 'BGR', 3), 'mono8': ('L', 'L', 1)}
                    if enc not in modes:
                        raise ValueError(f'Unsupported image encoding {enc}; no silent reinterpretation')
                    mode, rawmode, channels = modes[enc]
                    if msg.step < msg.width * channels or len(msg.data) < msg.step * msg.height:
                        raise ValueError('Invalid image stride/size')
                    image = Image.frombytes(mode, (msg.width, msg.height), bytes(msg.data), 'raw', rawmode, msg.step, 1)
                    name = f'{row[0]:09d}.png'
                    image.save(out / 'images' / name)
                    writer.writerow([name, root.name, row[0], row[1], row[2], root.name])
        (out / 'README.txt').write_text('Unlabelled images for review/annotation. No fabricated labels. Keep entire runs/site sessions in one split. Original MCAP remains the Foxglove and sensor-data source.\n')
    finally:
        db.close()
    return out


def export_3d(root, index_dir, every=10):
    from boat_data.pcd import write_pcd
    root, index_dir = Path(root).resolve(), resolve_index(index_dir)
    report = json.loads((index_dir / 'quality_report.json').read_text())
    if 'lidar' not in report.get('required', []):
        raise ValueError('Re-index with LiDAR required before 3D export')
    images = export_images(root, index_dir, every)
    out = images / 'cvat_3d'
    out.mkdir()
    db = sqlite3.connect(index_dir / 'index.sqlite')
    db.execute('CREATE TEMP TABLE pairs(lidar_id INTEGER,image TEXT,camera_id INTEGER,matches TEXT)')
    with (images / 'provenance.csv').open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            matches = json.loads(row['associations'])
            lidar = matches.get('lidar')
            if lidar is None:
                raise ValueError('3D export requires accepted LiDAR associations')
            db.execute('INSERT INTO pairs VALUES(?,?,?,?)',
                       (lidar['sample_id'],row['image'],row['camera_sample_id'],row['associations']))
    report=json.loads((index_dir / 'quality_report.json').read_text())
    try:
        with (out / 'provenance.jsonl').open('w',encoding='utf-8') as provenance:
            for source in report['sources']:
                for ordinal, (_,channel,_,msg) in enumerate(decoded(root/source['path'])):
                    if channel.topic != TOPICS['lidar']:
                        continue
                    pairs=db.execute('SELECT p.image,p.camera_id,p.matches,s.id FROM sample s JOIN pairs p ON p.lidar_id=s.id WHERE s.bag=? AND s.ordinal=?', (source['path'],ordinal))
                    for image,camera_id,matches,lidar_id in pairs:
                        name=Path(image).stem
                        write_pcd(msg,out/(name+'.pcd'))
                        shutil.copyfile(images/'images'/image,out/(name+'.png'))
                        provenance.write(json.dumps({'name':name,'camera_id':camera_id,'lidar_id':lidar_id,
                            'frame_id':msg.header.frame_id,'associations':json.loads(matches),
                            'calibration_applied':False,'deskew_applied':False})+'\n')
        calibration=root/'calibration.json'
        if calibration.exists():
            shutil.copyfile(calibration, images/'calibration.json')
        return out
    finally:
        db.close()


def archive_run(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists() or destination.is_relative_to(source) or source.is_relative_to(destination):
        raise ValueError('Destination must be a new, separate directory')
    if any(p.is_symlink() for p in source.rglob('*')):
        raise ValueError('Archive refuses symlinks')
    manifest = json.loads((source / 'manifest.json').read_text())
    if not manifest.get('ended_utc'):
        raise ValueError('Stop and finalize recording before archive')
    files = [p for p in source.rglob('*') if p.is_file()]
    checks = {p.relative_to(source).as_posix(): sha256(p) for p in files}
    expected = source / 'checksums.sha256'
    if expected.exists():
        for line in expected.read_text().splitlines():
            digest, name = line.split(None, 1)
            name = name.lstrip('*').replace('\\', '/')
            if checks.get(name) != digest:
                raise ValueError(f'Capture checksum failed: {name}')
    staging = destination.with_name(destination.name + '.partial-' + uuid.uuid4().hex[:8])
    shutil.copytree(source, staging)
    for name, digest in checks.items():
        if sha256(staging / name) != digest or sha256(source / name) != digest:
            raise ValueError('Copy verification failed or source changed; partial copy retained')
    (staging / 'transfer_receipt.json').write_text(json.dumps({'verified_sha256': checks}, indent=2))
    staging.rename(destination)
    return destination


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    idx = sub.add_parser('index'); idx.add_argument('run'); idx.add_argument('--without-lidar', action='store_true')
    exp = sub.add_parser('export'); exp.add_argument('run'); exp.add_argument('index'); exp.add_argument('--every', type=int, default=10)
    cloud = sub.add_parser('export-3d'); cloud.add_argument('run'); cloud.add_argument('index'); cloud.add_argument('--every', type=int, default=10)
    arc = sub.add_parser('archive'); arc.add_argument('run'); arc.add_argument('destination')
    args = p.parse_args(argv)
    if args.command == 'index':
        required = tuple(k for k in TOPICS if not (args.without_lidar and k == 'lidar'))
        out, report = index_run(args.run, required)
        print(out); print(json.dumps(report, indent=2))
        return 0 if report['pass'] else 2
    if args.command == 'export-3d':
        print(export_3d(args.run, args.index, args.every))
    elif args.command == 'export':
        print(export_images(args.run, args.index, args.every))
    else:
        print(archive_run(args.run, args.destination))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
