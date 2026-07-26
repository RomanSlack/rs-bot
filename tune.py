"""Coordinate-descent gain search for the balancer. Writes the winner to stdout."""

import math
import sys
from dataclasses import asdict, replace

from src.rsbot.balance import Gains
from src.rsbot.sim import cost

FIELDS = ["kp", "kd", "kv", "kx", "tau_odom"]
STEPS = [2.0, 1.4, 1.15]


def search(g, rounds=3, backlash=0.0):
    best, bc = g, cost(g, backlash=backlash)
    print(f"start {bc:.3f}  {asdict(best)}")
    for step in STEPS:
        for _ in range(rounds):
            improved = False
            for f in FIELDS:
                for mul in (step, 1 / step):
                    cand = replace(best, **{f: getattr(best, f) * mul})
                    c = cost(cand, backlash=backlash)
                    if c < bc - 1e-3:
                        best, bc, improved = cand, c, True
                        print(f"  {f} -> {getattr(best, f):.3f}   cost {bc:.3f}")
            if not improved:
                break
    return best, bc


if __name__ == "__main__":
    deg = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    print(f"tuning against {deg:.2f} deg of backlash")
    g, c = search(Gains(), backlash=math.radians(deg))
    print(f"\nbest cost {c:.3f}")
    print("Gains(" + ", ".join(f"{f}={getattr(g, f):.3f}" for f in FIELDS) + ")")
