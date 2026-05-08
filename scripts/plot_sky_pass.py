#!/usr/bin/env python3
"""
plot_sky_pass.py — polar sky plot of where each decoded packet was
received from, using the az/el columns the receivers (and rx_replay
--update for older captures) wrote into the packet DB.

Reads packet_query --format=csv and renders a matplotlib polar plot:
radius = 90 - elevation_deg (so the horizon sits on the outer ring),
theta = azimuth_deg (north up, east clockwise — the conventional
ground-station view). Markers are coloured by packet type. Optional
line connecting consecutive packets within one pass — handy when the
sat traced a long arc and you want to see continuity.

Usage:
    plot_sky_pass.py [--db PATH] [--since 24h] [--until ...]
                     [--type beacon|tcmd_response|log|bulk_file]
                     [--satellite CTS1] [--source-tool ...]
                     [--out sky.png]
                     [--show]                 # interactive window
                     [--connect]              # draw line between points
                     [--packet-query PATH]    # default: ./packet_query

The CSV is produced by `packet_query --format=csv`. Time/type filters
are forwarded straight through. Rows whose az_deg or el_deg is NULL
are dropped silently — those are typically packets decoded by audio-
path receivers that don't run SGP4. Run rx_replay --update on those
captures (with their TLE) to fill the columns in retroactively.
"""

import argparse
import csv
import io
import os
import shutil
import subprocess
import sys

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as e:
    sys.stderr.write(
        "plot_sky_pass.py: missing matplotlib/numpy. Install with\n"
        "  pip install matplotlib numpy\n"
        f"({e})\n")
    sys.exit(1)


TYPE_COLOURS = {
    "beacon":         "#1f77b4",
    "tcmd_response":  "#ff7f0e",
    "log":            "#2ca02c",
    "bulk_file":      "#9467bd",
}


def main():
    ap = argparse.ArgumentParser(
        description="Polar sky plot of decoded-packet az/el from the "
                    "AX100 packet DB.")
    ap.add_argument("--db", help="DB path (default: packet_query's default)")
    ap.add_argument("--since")
    ap.add_argument("--until")
    ap.add_argument("--type", help="beacon | tcmd_response | log | bulk_file")
    ap.add_argument("--satellite")
    ap.add_argument("--source-tool")
    ap.add_argument("--like")
    ap.add_argument("--limit", type=int, default=10000)
    ap.add_argument("--out", default="sky_pass.png",
                    help="PNG output path (default sky_pass.png)")
    ap.add_argument("--show", action="store_true",
                    help="open an interactive matplotlib window")
    ap.add_argument("--connect", action="store_true",
                    help="draw a line through consecutive packets")
    ap.add_argument("--packet-query", default=None,
                    help="path to packet_query (default: search PATH and "
                         "the script's sibling build/ directory)")
    ap.add_argument("--title", default=None)
    args = ap.parse_args()

    pq = locate_packet_query(args.packet_query)
    if pq is None:
        sys.stderr.write("plot_sky_pass.py: cannot find packet_query "
                         "binary. Pass --packet-query=<path>.\n")
        return 1

    cmd = [pq, "--format=csv", f"--limit={args.limit}"]
    for opt, val in (
        ("--db", args.db), ("--since", args.since), ("--until", args.until),
        ("--type", args.type), ("--satellite", args.satellite),
        ("--source-tool", args.source_tool), ("--like", args.like),
    ):
        if val is not None:
            cmd.append(f"{opt}={val}")

    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr or "packet_query failed\n")
        return proc.returncode

    rows = list(csv.DictReader(io.StringIO(proc.stdout)))
    rows = [r for r in rows if r.get("az_deg") and r.get("el_deg")]
    if not rows:
        sys.stderr.write(
            "plot_sky_pass.py: no rows with az/el in this query. "
            "Run rx_replay --update with the pass's TLE to backfill, "
            "or widen the filter.\n")
        return 1

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)  # clockwise — east on the right
    ax.set_rlim(0, 90)
    ax.set_rticks([15, 30, 45, 60, 75])
    ax.set_yticklabels(["75°", "60°", "45°", "30°", "15°"])
    ax.grid(True, alpha=0.3)

    # Group by packet type so the legend stays clean.
    by_type = {}
    for r in rows:
        by_type.setdefault(r["packet_type_name"] or "unknown", []).append(r)

    for tname, group in by_type.items():
        az = np.deg2rad([float(r["az_deg"]) for r in group])
        el = np.array([float(r["el_deg"]) for r in group])
        rad = 90.0 - el
        ax.scatter(az, rad,
                   s=24,
                   color=TYPE_COLOURS.get(tname, "#7f7f7f"),
                   label=f"{tname} ({len(group)})",
                   alpha=0.8,
                   edgecolors="black", linewidths=0.4)

    if args.connect:
        # Connect ALL points in time order regardless of type so the
        # arc is visible as one continuous line.
        rows_sorted = sorted(rows, key=lambda r: r["ts_received"])
        az = np.deg2rad([float(r["az_deg"]) for r in rows_sorted])
        rad = 90.0 - np.array([float(r["el_deg"]) for r in rows_sorted])
        ax.plot(az, rad, color="#444444", alpha=0.4, linewidth=0.8, zorder=0)

    title = args.title or "Decoded-packet sky positions (RAO frame)"
    ax.set_title(title, pad=20)
    ax.legend(loc="lower left", bbox_to_anchor=(1.05, 0), fontsize=9)
    fig.tight_layout()

    fig.savefig(args.out, dpi=150)
    sys.stderr.write(f"plot_sky_pass.py: wrote {args.out} ({len(rows)} points)\n")
    if args.show:
        plt.show()
    return 0


def locate_packet_query(arg):
    if arg is not None:
        return arg if os.access(arg, os.X_OK) else None
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "..", "build", "packet_query"),
        shutil.which("packet_query"),
    ]
    for c in candidates:
        if c and os.access(c, os.X_OK):
            return c
    return None


if __name__ == "__main__":
    sys.exit(main())
