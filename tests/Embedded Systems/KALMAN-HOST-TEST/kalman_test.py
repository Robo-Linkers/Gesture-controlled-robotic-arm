"""Host-side check of kalmanFilter() in src/Glove.ino. No hardware needed.

Run:  python kalman_test.py [path/to/Glove.ino]

What it does
- Reads kalmanQ, kalmanR, interval and the dt definition from the real Glove.ino,
  and checks the covariance-update lines in the source text.
- Runs a Python port of kalmanFilter() on simulated IMU data (float64 here,
  float32 on the ESP32). The port mirrors the C++ by hand, so keep them in sync.

Simulated IMU: 40 deg / 8 s sine gesture, gyro bias 2 deg/s, gyro noise 0.5 deg/s,
accelerometer noise 3 deg (these noise levels are assumptions, not measurements).
"""
import math
import random
import re
import sys
from pathlib import Path

SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[3] / "src" / "Glove.ino"
text = SRC.read_text()


def grab(pattern):
    m = re.search(pattern, text)
    if not m:
        sys.exit(f"FAIL: could not find /{pattern}/ in {SRC}")
    return m.group(1)


INTERVAL_MS = int(grab(r"const long interval = (\d+);"))
Q = float(grab(r"kalmanQ = ([\d.]+)"))
R = float(grab(r"kalmanR = ([\d.]+)"))
DT_EXPR = grab(r"\bdt = ([^;]+);")
LOOP_DT = INTERVAL_MS / 1000.0


class Kalman:
    """Port of kalmanFilter(). fix_cov=False reproduces the old covariance update."""

    def __init__(self, dt, fix_cov):
        self.dt, self.fix_cov = dt, fix_cov
        self.angle = self.bias = 0.0
        self.P = [[1.0, 0.0], [0.0, 1.0]]
        self.max_asym = 0.0

    def update(self, new_angle, new_rate):
        P, dt = self.P, self.dt
        self.angle += dt * (new_rate - self.bias)
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q)
        P[0][1] -= dt * P[1][1]
        P[1][0] -= dt * P[1][1]
        P[1][1] += Q * dt
        S = P[0][0] + R
        K0, K1 = P[0][0] / S, P[1][0] / S
        y = new_angle - self.angle
        self.angle += K0 * y
        self.bias += K1 * y
        p00, p01 = P[0][0], P[0][1]
        P[0][0] -= K0 * P[0][0]
        P[0][1] -= K0 * P[0][1]
        P[1][0] -= K1 * (p00 if self.fix_cov else P[0][0])
        P[1][1] -= K1 * (p01 if self.fix_cov else P[0][1])
        self.max_asym = max(self.max_asym, abs(P[0][1] - P[1][0]))
        return self.angle


def simulate(dt_used, fix_cov, seed, seconds=120, gyro_bias=2.0, gyro_sd=0.5, acc_sd=3.0):
    rng = random.Random(seed)
    k = Kalman(dt_used, fix_cov)
    n = int(seconds / LOOP_DT)
    sq_err = []
    for i in range(n):
        t = i * LOOP_DT
        truth = 40 * math.sin(2 * math.pi * t / 8)
        rate = 40 * (2 * math.pi / 8) * math.cos(2 * math.pi * t / 8)
        est = k.update(truth + rng.gauss(0, acc_sd), rate + gyro_bias + rng.gauss(0, gyro_sd))
        if t >= 10:  # skip start-up
            sq_err.append((est - truth) ** 2)
    return math.sqrt(sum(sq_err) / len(sq_err)), k.max_asym


def mean_over_seeds(dt_used, fix_cov, runs=20):
    res = [simulate(dt_used, fix_cov, s) for s in range(runs)]
    return sum(r[0] for r in res) / runs, max(r[1] for r in res)


failures = []


def check(ok, msg):
    print(("PASS  " if ok else "FAIL  ") + msg)
    if not ok:
        failures.append(msg)


# 1) Source checks: dt must follow the loop interval; covariance update must use pre-update values.
check("interval / 1000" in DT_EXPR, f"dt is derived from interval (found: dt = {DT_EXPR})")
check(grab(r"P\[1\]\[0\] -= K\[1\] \* (\S+);") != "P[0][0]",
      "P[1][0] update does not read the already-updated P[0][0]")
check(grab(r"P\[1\]\[1\] -= K\[1\] \* (\S+);") != "P[0][1]",
      "P[1][1] update does not read the already-updated P[0][1]")

# 2) Port checks on simulated data (new = this PR, old = dt 0.05 and old covariance update).
new_rmse, new_asym = mean_over_seeds(LOOP_DT, True)
old_rmse, old_asym = mean_over_seeds(0.05, False)
print(f"\n  old (dt=0.05, old P update): RMSE {old_rmse:6.2f} deg, max |P01-P10| {old_asym:.1e}")
print(f"  new (dt={LOOP_DT}, fixed P):   RMSE {new_rmse:6.2f} deg, max |P01-P10| {new_asym:.1e}\n")
check(new_asym < 1e-9, "covariance matrix stays symmetric with the fixed update")
check(old_asym > 1e-3, "old update does become asymmetric (sanity check that the test can detect the bug)")
check(new_rmse < 3.0, "filtered error on a moving gesture is below the 3 deg raw accelerometer noise")

sys.exit(1 if failures else 0)
