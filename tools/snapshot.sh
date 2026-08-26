#!/bin/bash
# Timestamped backup of the working tree.
#
#   tools/snapshot.sh "what changed"
#
# Writes _backup/<YYYYMMDD_HHMMSS>/ containing the source directories, plus a
# NOTE file recording what the snapshot was taken for and the QC result at that
# moment. Run it every time a piece of work is finished, so there is always a
# point to go back to that is independent of the promotion patches.
#
# Build output, caches and the archives themselves are excluded: those are
# regenerable and would make every snapshot hundreds of megabytes.
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$(pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEST="_backup/$STAMP"
NOTE="${1:-manual snapshot}"

DIRS=(gps_localize_ws webapp tools setup docs firmware)

mkdir -p "$DEST"
for d in "${DIRS[@]}"; do
    [ -e "$d" ] || continue
    rsync -a \
        --exclude='build/' --exclude='install/' --exclude='log/' \
        --exclude='__pycache__/' --exclude='.pio/' --exclude='*.pyc' \
        "$d/" "$DEST/$d/"
done

# Root-level files worth keeping with the source.
for f in README.md plan.html; do
    [ -f "$f" ] && cp -a "$f" "$DEST/" || true
done

{
    echo "snapshot : $STAMP"
    echo "taken    : $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo "reason   : $NOTE"
    echo "host     : $(hostname)"
    echo
    echo "--- QC at snapshot time ---"
    if python3 tools/qc.py >/dev/null 2>&1; then
        echo "project QC : PASS"
    else
        echo "project QC : FAIL"
    fi
    if python3 tools/port_qc.py --static-only . >/dev/null 2>&1; then
        echo "port QC    : PASS"
    else
        echo "port QC    : FAIL"
    fi
    echo
    echo "--- contents ---"
    echo "files: $(find "$DEST" -type f | wc -l)"
    echo "size : $(du -sh "$DEST" | cut -f1)"
} > "$DEST/NOTE.txt"

echo "snapshot -> $ROOT/$DEST"
sed -n '1,3p;9,12p' "$DEST/NOTE.txt" | sed 's/^/  /'

# Keep the most recent 20 snapshots; older ones are covered by the _patches
# rollbacks, which are diff-sized rather than full copies.
mapfile -t OLD < <(ls -1d _backup/*/ 2>/dev/null | grep -E '_backup/[0-9]{8}_[0-9]{6}/' | sort | head -n -20)
for o in "${OLD[@]:-}"; do
    [ -n "$o" ] && rm -rf "$o" && echo "  pruned old snapshot: $o"
done
