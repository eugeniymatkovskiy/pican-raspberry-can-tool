#!/usr/bin/env python3
"""Summarise gcov output for files under uds/ only.

Invoked from the coverage Dockerfile after `gcov` has emitted *.gcov next
to the build artifacts. Prints a per-file and total line-coverage summary.
"""
import glob
import os
import re
import sys

total_lines = total_hit = 0
per_file = []

WANT = ("iso_tp.cpp", "uds_client.cpp", "vw_mqb.cpp", "backup.cpp")

candidates = (glob.glob("/src/build/*.gcov") + glob.glob("/src/uds/*.gcov")
              + glob.glob("/src/*.gcov"))
seen = set()
for gcov_file in sorted(candidates):
    src = None
    lines = hit = 0
    with open(gcov_file) as f:
        for line in f:
            m = re.match(r"\s*-:\s*0:Source:(.*)", line)
            if m:
                src = m.group(1).strip()
                continue
            if src and os.path.basename(src) not in WANT:
                src = None
                break
            m = re.match(r"\s*(\d+|#####|-):\s*\d+:", line)
            if m and src:
                tag = m.group(1)
                if tag == "-":
                    continue
                lines += 1
                if tag != "#####":
                    hit += 1
    if src and os.path.basename(src) not in seen:
        seen.add(os.path.basename(src))
        per_file.append((src, lines, hit))
        total_lines += lines
        total_hit += hit

for src, lines, hit in per_file:
    pct = (hit / lines * 100) if lines else 0.0
    print(f"  {os.path.basename(src):18s}  lines {hit:4d}/{lines:<4d} ({pct:5.1f}%)")
print()
if total_lines:
    pct = total_hit / total_lines * 100
    print(f"  TOTAL (uds/*.cpp) : lines {total_hit}/{total_lines} ({pct:.1f}%)")
else:
    print("  (no uds/ source files found in gcov output)")
