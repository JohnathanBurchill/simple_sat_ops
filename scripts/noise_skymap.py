#!/usr/bin/env python3
# Build an all-sky noise map from a --scan-sky run.
#
# Inputs are auto-discovered in the chosen pass directory (default
# cwd; override with the positional argument):
#   scan_sky_UT=*.csv          rotator dwell log
#   simple_sat_ops_UT=*.iq     raw interleaved int16 IQ at 96 kHz
#
# For every "arrived" event we grab a window of IQ samples and average
# |I|^2 + |Q|^2 over it, then plot the result on a polar sky chart
# (zenith at centre, horizon at the edge, N up, E to the right) using a
# Lambert azimuthal equal-area radial map so plotted area is
# proportional to solid angle on the sky.
#
# Usage:
#   noise_skymap.py                          # use cwd, write noise_skymap.png there
#   noise_skymap.py /path/to/pass/dir        # use that dir, write png there
#   noise_skymap.py PASS --out foo.png       # explicit output
#   noise_skymap.py PASS --rate 96000        # override sample rate
#   noise_skymap.py PASS --dwell 5.0 --settle 0.3
#       # override dwell + settle (defaults track SCAN_DWELL_S in main.c)

import argparse
import glob
import os
import re
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


def find_one(directory, pattern):
    hits = sorted(glob.glob(os.path.join(directory, pattern)))
    if not hits:
        sys.exit(f"no match for {pattern} in {directory}")
    return hits[0]


def iq_start_unix_ms(iq_path):
    # Filename pattern: simple_sat_ops_UT=YYYYMMDDTHHMMSS.mmm.iq
    m = re.search(r"UT=(\d{8})T(\d{6})\.(\d{3})\.iq$", iq_path)
    if m is None:
        sys.exit(f"can't parse start time from {iq_path}")
    import datetime as dt
    d = dt.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
    d = d.replace(tzinfo=dt.timezone.utc)
    return int(d.timestamp() * 1000) + int(m.group(3))


def read_arrived(csv_path):
    rows = []
    with open(csv_path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 6 or parts[5] != "arrived":
                continue
            rows.append({
                "t_ms":  int(parts[0]),
                "az":    float(parts[1]),
                "el":    float(parts[2]),
            })
    return rows


def power_for_window(iq_path, sample_offset, n_samples):
    # int16 I,Q pairs, native endianness.  np.fromfile is fine for a
    # short read because we seek by offset rather than reading the
    # whole file.
    byte_off = sample_offset * 2 * 2  # 2 chans * int16
    with open(iq_path, "rb") as f:
        f.seek(byte_off)
        buf = np.fromfile(f, dtype=np.int16, count=n_samples * 2)
    if buf.size < 2:
        return np.nan
    iq = buf.astype(np.float32).reshape(-1, 2)
    p = iq[:, 0] ** 2 + iq[:, 1] ** 2
    return float(p.mean())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("pass_dir", nargs="?", default=".",
                    help="pass directory holding scan_sky_UT=*.csv "
                         "and simple_sat_ops_UT=*.iq (default: cwd)")
    ap.add_argument("--out", default=None,
                    help="output PNG (default: <pass_dir>/noise_skymap.png)")
    ap.add_argument("--rate", type=int, default=96000,
                    help="IQ complex sample rate in Hz (default 96000 — "
                         "matches simple_sat_ops' rx_session)")
    ap.add_argument("--dwell", type=float, default=5.0,
                    help="dwell window per pointing in seconds, "
                         "should match SCAN_DWELL_S (default 5.0)")
    ap.add_argument("--settle", type=float, default=0.3,
                    help="seconds to skip at the start of each dwell "
                         "to let the rotator settle (default 0.3)")
    args = ap.parse_args()

    pass_dir = os.path.abspath(args.pass_dir)
    csv_path = find_one(pass_dir, "scan_sky_UT=*.csv")
    iq_path  = find_one(pass_dir, "simple_sat_ops_UT=*.iq")
    out_path = args.out or os.path.join(pass_dir, "noise_skymap.png")
    rate     = args.rate
    dwell_s  = args.dwell
    settle_s = args.settle

    print(f"pass = {pass_dir}")
    print(f"csv  = {os.path.basename(csv_path)}")
    print(f"iq   = {os.path.basename(iq_path)}")

    t0_ms = iq_start_unix_ms(iq_path)
    iq_bytes = os.path.getsize(iq_path)
    iq_total_samples = iq_bytes // 4
    print(f"iq start unix_ms = {t0_ms}")
    print(f"iq samples       = {iq_total_samples}  ({iq_total_samples/rate:.1f} s @ {rate} Hz)")

    rows = read_arrived(csv_path)
    print(f"arrived events   = {len(rows)}")

    settle_n = int(settle_s * rate)
    dwell_n  = int((dwell_s - settle_s) * rate)

    az_list, el_list, p_list = [], [], []
    for r in rows:
        off_ms = r["t_ms"] - t0_ms
        if off_ms < 0:
            continue
        start = int(off_ms * rate / 1000) + settle_n
        if start + dwell_n > iq_total_samples:
            continue
        p = power_for_window(iq_path, start, dwell_n)
        if not np.isfinite(p) or p <= 0:
            continue
        az_list.append(r["az"])
        el_list.append(r["el"])
        p_list.append(p)

    p_db = 10.0 * np.log10(np.array(p_list))
    print(f"useable dwells   = {len(p_db)}")
    print(f"power range (dB) = [{p_db.min():.1f}, {p_db.max():.1f}]")

    # Lambert azimuthal equal-area projection, zenith-centred:
    #     r = 2·sin((90-el)/2 · π/180)
    # so dA_plot ∝ dΩ_sphere exactly.  Range: r=0 at zenith, r=√2 at
    # horizon.  Plot uses one pcolormesh strip per elevation ring; each
    # ring carries its own azimuth count (the scan grid is dense near
    # the horizon and sparse near zenith), so per-ring meshing avoids
    # any interpolation.
    fig = plt.figure(figsize=(9.0, 9.0))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)

    def el_to_r(el_deg):
        return 2.0 * np.sin(np.deg2rad((90.0 - el_deg) / 2.0))

    r_horizon = el_to_r(0.0)
    ax.set_rlim(0, r_horizon)
    el_ticks = [0, 30, 60, 90]
    ax.set_rticks([el_to_r(e) for e in el_ticks])
    ax.set_yticklabels([f"{e}" for e in el_ticks])
    ax.set_thetagrids([0, 90, 180, 270], labels=["N", "E", "S", "W"])

    # Group by elevation ring.
    rings = {}
    for a, e, p in zip(az_list, el_list, p_db.tolist()):
        rings.setdefault(round(e, 3), []).append((a, p))
    unique_els = sorted(rings.keys())

    def el_bounds(i):
        e = unique_els[i]
        if i == 0:
            e_lo = max(0.0, e - (unique_els[1] - e) / 2.0)
        else:
            e_lo = (unique_els[i - 1] + e) / 2.0
        if i == len(unique_els) - 1:
            e_hi = min(90.0, e + (e - unique_els[i - 1]) / 2.0)
        else:
            e_hi = (e + unique_els[i + 1]) / 2.0
        return e_lo, e_hi

    norm = mcolors.Normalize(vmin=float(p_db.min()), vmax=float(p_db.max()))
    cmap = plt.get_cmap("viridis")
    mesh = None
    for i, el_ring in enumerate(unique_els):
        pts = sorted(rings[el_ring], key=lambda x: x[0])
        azs  = np.array([p[0] for p in pts])
        vals = np.array([p[1] for p in pts])
        n = len(azs)
        e_lo, e_hi = el_bounds(i)
        r_outer = el_to_r(e_lo)   # closer to horizon → bigger r
        r_inner = el_to_r(e_hi)   # closer to zenith → smaller r

        # pcolormesh in polar coords draws straight chords between
        # adjacent (theta, r) vertices, so wide cells leave triangular
        # gaps between the chord and the real arc.  Subdivide anything
        # wider than ~5° so each rendered chord is short enough that the
        # gap is sub-pixel.
        if n == 1:
            raw_edges_deg = np.array([-180.0, 180.0])
            raw_vals      = vals
        else:
            raw_edges_deg = np.empty(n + 1)
            for k in range(n):
                prev_az = azs[k - 1] if k > 0 else (azs[-1] - 360.0)
                raw_edges_deg[k] = 0.5 * (prev_az + azs[k])
            raw_edges_deg[n] = raw_edges_deg[0] + 360.0
            raw_vals = vals
        max_step_deg = 5.0
        sub_edges = []
        sub_vals  = []
        for k in range(len(raw_vals)):
            w = raw_edges_deg[k + 1] - raw_edges_deg[k]
            n_sub = max(1, int(np.ceil(abs(w) / max_step_deg)))
            for j in range(n_sub):
                sub_edges.append(raw_edges_deg[k] + j * w / n_sub)
                sub_vals.append(raw_vals[k])
        sub_edges.append(raw_edges_deg[-1])
        theta_edges = np.deg2rad(np.array(sub_edges))
        C = np.array(sub_vals).reshape(1, -1)
        T, R = np.meshgrid(theta_edges, [r_inner, r_outer])
        mesh = ax.pcolormesh(T, R, C, cmap=cmap, norm=norm,
                             shading="flat", edgecolors="face",
                             linewidth=0, antialiased=False)

    cb = fig.colorbar(mesh, ax=ax, pad=0.10, shrink=0.85)
    cb.set_label("Mean power  10·log10(<|I|² + |Q|²>)  [dB rel. int16²]")

    base = os.path.basename(csv_path).replace(".csv", "")
    ax.set_title(f"All-sky noise map (Lambert equal-area) — {base}\n"
                 f"{len(p_db)} dwells × {dwell_s - settle_s:.1f} s "
                 f"@ {rate/1000:.0f} kHz complex",
                 pad=18)

    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
