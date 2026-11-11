#!/usr/bin/env python3
"""Compatibility entry point for the streaming five-sensor dataset indexer."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'boat_data'))
from boat_data.dataset import main
if __name__ == '__main__':
    raise SystemExit(main(['index', *sys.argv[1:]]))
