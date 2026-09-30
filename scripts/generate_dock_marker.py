#!/usr/bin/env python3
"""Generate a printable ArUco marker for the dock.

Print at 100% scale (not "fit to page"), measure the black square with a
ruler and set that as marker_length_m on dock_marker_node. Distance error
scales with that number. Needs only OpenCV.

    python3 generate_dock_marker.py --marker-id 0 --size-cm 15 --output dock_marker_id0.png
    python3 generate_dock_marker.py --dictionary 4X4_50 --marker-id 0 --size-cm 15 \
        --dpi 300 --output dock_marker_id0.png
"""

import argparse

import cv2
import numpy as np

ARUCO_DICTIONARIES = {
    '4X4_50': cv2.aruco.DICT_4X4_50,
    '4X4_100': cv2.aruco.DICT_4X4_100,
    '5X5_50': cv2.aruco.DICT_5X5_50,
    '5X5_100': cv2.aruco.DICT_5X5_100,
    '6X6_50': cv2.aruco.DICT_6X6_50,
    '6X6_100': cv2.aruco.DICT_6X6_100,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dictionary', default='4X4_50', choices=sorted(ARUCO_DICTIONARIES),
                         help='ArUco dictionary. Must match dock_marker_node\'s "marker_dictionary" '
                              'parameter. Default 4X4_50 -- easiest to detect at a distance/in poor '
                              'light since each cell is bigger relative to the marker.')
    parser.add_argument('--marker-id', type=int, default=0,
                         help='Marker ID within the dictionary. Must match dock_marker_node\'s '
                              '"target_marker_id" parameter. Default 0.')
    parser.add_argument('--size-cm', type=float, default=15.0,
                         help='Physical printed size of the marker (the black square, not '
                              'including white border) in centimeters. Default 15cm -- a good '
                              'balance of "detectable from a few meters" vs "fits on a normal '
                              'printout". Bigger marker = detectable from farther away.')
    parser.add_argument('--dpi', type=int, default=300,
                         help='Print resolution. 300 DPI is standard for a sharp printout; '
                              'lower will look blocky and may hurt detection at a distance.')
    parser.add_argument('--border-bits', type=int, default=1,
                         help='White/black border width in marker cells around the pattern '
                              '(the aruco library\'s own quiet-zone border, not the printed '
                              'white margin below). Leave at default unless you know why not.')
    parser.add_argument('--quiet-zone-cm', type=float, default=3.0,
                         help='Extra flat white margin printed around the marker, in cm. '
                              'ArUco detection needs white space around the pattern to find its '
                              'edges reliably -- do not mount the marker flush to the edge of '
                              'the dock or right next to other high-contrast clutter.')
    parser.add_argument('--output', default='dock_marker.png', help='Output PNG path.')
    args = parser.parse_args()

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[args.dictionary])

    marker_px = int(round(args.size_cm / 2.54 * args.dpi))
    quiet_px = int(round(args.quiet_zone_cm / 2.54 * args.dpi))

    marker_img = cv2.aruco.generateImageMarker(
        aruco_dict, args.marker_id, marker_px, borderBits=args.border_bits)

    canvas_size = marker_px + 2 * quiet_px
    canvas = np.full((canvas_size, canvas_size), 255, dtype=np.uint8)
    canvas[quiet_px:quiet_px + marker_px, quiet_px:quiet_px + marker_px] = marker_img

    cv2.imwrite(args.output, canvas)

    print(f'Wrote {args.output}  ({canvas_size}x{canvas_size}px at {args.dpi} DPI)')
    print(f'Dictionary: {args.dictionary}   Marker ID: {args.marker_id}')
    print(f'Printed marker (black square) should measure {args.size_cm:.1f}cm x {args.size_cm:.1f}cm '
          f'-- MEASURE IT after printing and set that exact value (in meters) as dock_marker_node\'s '
          f'"marker_length_m" parameter, e.g. {args.size_cm / 100.0:.3f}.')
    print('Print at 100% / "actual size" -- NOT "fit to page" or "shrink to fit".')
    print('Mount flat, avoid glare/lamination glare, keep the white margin around it clear.')


if __name__ == '__main__':
    main()
