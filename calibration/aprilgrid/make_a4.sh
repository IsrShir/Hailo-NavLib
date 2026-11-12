#!/bin/sh
# Laptop: regenerate aprilgrid_6x4_30mm_A4.pdf (6x4 tag36h11, 30 mm tags, spacing 0.3) with Kalibr's own generator.
# The PDF is committed; only run this to change the target. Needs python3 with numpy + pyx (pip install pyx numpy).
# Kalibr (BSD-3, https://github.com/ethz-asl/kalibr) is downloaded, not stored here; we patch two things:
# drop the text label (needs TeX) and centre the grid on an A4 page.
#   make_a4.sh [python (default python3)]
# Print at 100% (no "fit to page") and measure a tag: if it is not 30.0 mm, put the measured size in the yaml.
set -e
PY=${1:-python3}
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
curl -fsSL -o "$WORK/kalibr_create_target_pdf" \
  https://raw.githubusercontent.com/ethz-asl/kalibr/master/aslam_offline_calibration/kalibr/python/kalibr_create_target_pdf
"$PY" - "$WORK/kalibr_create_target_pdf" "$WORK/make_aprilgrid.py" <<'PATCH'
import re, sys
s = open(sys.argv[1]).read()
s = re.sub(r'^(\s*)c\.text\(.*$', r'\1pass  # label removed (needs TeX)', s, flags=re.M)
s = s.replace('c.writePDFfile(parsed.output)',
              'document.document([document.page(c, paperformat=document.paperformat.A4, '
              'centered=1, fittosize=0, rotated=1)]).writePDFfile(parsed.output)')
open(sys.argv[2], 'w').write(s)
PATCH
cd "$HERE"
"$PY" -I "$WORK/make_aprilgrid.py" --type apriltag --nx 6 --ny 4 --tsize 0.030 --tspace 0.3 aprilgrid_6x4_30mm_A4
rm -r "$WORK"
ls -l "$HERE/aprilgrid_6x4_30mm_A4.pdf"
