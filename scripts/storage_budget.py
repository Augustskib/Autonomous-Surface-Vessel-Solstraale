#!/usr/bin/env python3
"""Estimate capacity from measured aggregate rates; decimal GB/MB, no hardware access."""
import argparse
import json
import math


def budget(free_gb, reserve_gb, megabytes_per_second, minutes):
    values=(free_gb,reserve_gb,megabytes_per_second,minutes)
    if not all(math.isfinite(v) and v >= 0 for v in values) or megabytes_per_second == 0:
        raise ValueError('Finite nonnegative values and positive measured rate required')
    available=max(0,free_gb-reserve_gb)
    # 25% margin for variation and overhead; this is an estimate, not a guarantee.
    rate=megabytes_per_second*1.25
    needed=rate*60*minutes/1000
    return {'usable_gb':available,'required_gb_with_margin':needed,
            'estimated_minutes_with_margin':available*1000/rate/60,
            'fits_estimate':needed<=available}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--free-gb',type=float,required=True)
    p.add_argument('--reserve-gb',type=float,default=2)
    p.add_argument('--mbps',type=float,required=True,help='Measured total MB/s, not megabits/s')
    p.add_argument('--minutes',type=float,required=True)
    a=p.parse_args(); print(json.dumps(budget(a.free_gb,a.reserve_gb,a.mbps,a.minutes),indent=2))
