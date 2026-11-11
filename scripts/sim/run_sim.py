#!/usr/bin/env python3
"""Linux-only simulation process ownership; never kill by global process name."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def stop_group(pid, timeout=10., reap=lambda: None):
    # Called only for the process group created by this invocation.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            reap()
            try:
                os.killpg(pid, 0)
            except ProcessLookupError:
                return
            time.sleep(.1)


def main():
    if os.name != 'posix':
        raise SystemExit('Run simulation in Linux/WSL, not native Windows')
    import fcntl
    root = Path(__file__).resolve().parents[2]
    with (root / '.asv-simulation.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('This workspace already has a managed simulation running')
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = env.get('ASV_SIM_DOMAIN_ID', '42')
        # Prevent Gazebo transport from joining another workspace's simulation.
        import hashlib
        env['GZ_PARTITION'] = 'asv-' + hashlib.sha256(str(root).encode()).hexdigest()[:16]
        proc = subprocess.Popen(['ros2','launch','boat_simulation','boat_simulation.launch.py'],
                                cwd=root,env=env,start_new_session=True)
        def interrupted(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupted)
        try:
            return proc.wait()
        except KeyboardInterrupt:
            return 130
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            stop_group(proc.pid, reap=proc.poll)
            proc.wait()


if __name__ == '__main__':
    sys.exit(main())
