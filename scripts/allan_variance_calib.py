#!/usr/bin/env python3
"""allan_variance_calib.py

Static bench capture -> Allan deviation -> gyro noise parameters.

Two numbers we actually care about for the EKF:
  * angle random walk (ARW)  -> GYRO_NOISE in ekf_6dof.cpp
  * bias instability (BI)    -> GYRO_BIAS_WALK in ekf_6dof.cpp

Usage:
    python allan_variance_calib.py data/mpu6050_bench_raw.csv
    python allan_variance_calib.py capture.csv --axis gy --rate 200
"""

import argparse
import csv
import math


def load_gyro(path, axis="gx"):
    """Read timestamp_ms plus the requested gyro column.

    Accepts either the bare axis name (gx) or the full header (gx_rads).
    """
    ts, vals = [], []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        fields = reader.fieldnames or []
        if axis in fields:
            col = axis
        elif f"{axis}_rads" in fields:
            col = f"{axis}_rads"
        elif f"{axis}_rad_s" in fields:
            col = f"{axis}_rad_s"
        else:
            raise SystemExit(f"no gyro column for axis '{axis}' in {fields}")

        for row in reader:
            ts.append(float(row["timestamp_ms"]))
            vals.append(float(row[col]))
    return ts, vals


def sample_rate(ts):
    if len(ts) < 2:
        return 0.0
    span_s = (ts[-1] - ts[0]) / 1000.0
    return (len(ts) - 1) / span_s if span_s > 0 else 0.0


def overlapping_allan(vals, rate, max_clusters=20):
    """Overlapping Allan deviation. Returns [(tau_s, adev_rads), ...]."""
    n = len(vals)
    out = []
    m = 1
    while m < n // 2 and len(out) < max_clusters:
        # cluster means, overlapping
        sums = [0.0] * (n - m + 1)
        acc = sum(vals[:m])
        sums[0] = acc / m
        for i in range(1, n - m + 1):
            acc += vals[i + m - 1] - vals[i - 1]
            sums[i] = acc / m

        diffs = [sums[i + 1] - sums[i] for i in range(len(sums) - 1)]
        if not diffs:
            break
        mean_sq = sum(d * d for d in diffs) / (2.0 * len(diffs))
        adev = math.sqrt(mean_sq)
        tau = m / rate
        out.append((tau, adev))
        m *= 2
    return out


def estimate_params(adev):
    """Fit the -1/2 slope region for ARW and the flat region for BI."""
    if len(adev) < 3:
        return None, None

    # ARW: at tau = 1 s, adev = ARW (rad/s/sqrt(Hz)) if the curve is still
    # on the white-noise slope. Use the smallest tau point as the anchor.
    tau0, adev0 = adev[0]
    arw = adev0 * math.sqrt(tau0)

    # Bias instability: minimum of the Allan deviation curve.
    bi = min(a for _, a in adev)

    return arw, bi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="bench capture with timestamp_ms + gyro columns")
    ap.add_argument("--axis", default="gx", choices=["gx", "gy", "gz"])
    ap.add_argument("--rate", type=float, default=0.0,
                    help="override sample rate in Hz")
    args = ap.parse_args()

    ts, vals = load_gyro(args.csv, args.axis)
    rate = args.rate or sample_rate(ts)
    if rate <= 0:
        raise SystemExit("could not infer sample rate, pass --rate")

    mean = sum(vals) / len(vals)
    var = sum((v - mean) ** 2 for v in vals) / len(vals)

    print(f"samples      : {len(vals)}")
    print(f"rate         : {rate:.1f} Hz")
    print(f"axis         : {args.axis}")
    print(f"mean bias    : {mean:.6f} rad/s")
    print(f"std dev      : {math.sqrt(var):.6f} rad/s")
    print()

    adev = overlapping_allan(vals, rate)
    print(f"{'tau_s':>10} {'adev_rads':>14}")
    for tau, a in adev:
        print(f"{tau:10.4f} {a:14.8f}")

    arw, bi = estimate_params(adev)
    print()
    if arw is None:
        print("not enough clusters for a fit")
        return

    print(f"ARW (angle random walk) : {arw:.6f} rad/s/sqrt(Hz)")
    print(f"BI  (bias instability)  : {bi:.8f} rad/s")
    print()
    print("Suggested EKF constants:")
    print(f"  GYRO_NOISE     = {arw:.5f}f")
    print(f"  GYRO_BIAS_WALK = {bi * 0.1:.2e}f   # ~10% of BI as a starting point")


if __name__ == "__main__":
    main()
