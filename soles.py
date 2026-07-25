"""Search sole geometry x deploy rate for a transition that survives.

The baseline sole (half-length 40 mm, face 55 mm below the pivot) fails: its
corners sit 68 mm from the pivot, so deploying jacks the robot 28 mm up onto a
single edge with the wheels airborne and the balancer powerless. It topples
every time.

Two knobs:
  half-length L   sets the support polygon, and therefore standing tilt margin
  face radius Rf  sets how far the wheel is lifted at rest

and the coupling that hurts: corner radius = hypot(Rf, L), so the airborne
jacking is hypot(Rf, L) - 40 mm no matter how the controller is tuned.
"""

import itertools

import numpy as np

from src.rsbot.model import WHEEL_R
from src.rsbot.sim import rollout_deploy
from src.rsbot.transition import DeployCfg

COM_Z = 0.281


def summarise(L, Rf):
    rc = np.hypot(Rf, L)
    return dict(
        lift_mm=(Rf - WHEEL_R) * 1000,
        jack_mm=(rc - WHEEL_R) * 1000,
        tilt_deg=np.degrees(np.arctan2(L, COM_Z)),
    )


def main():
    # Rf floor is set by the wheel: the plate's inner face must clear it.
    lengths = (0.060, 0.050, 0.040, 0.030, 0.020)
    radii = (0.047, 0.052, 0.058, 0.065)
    rates = (1.5, 4.0, 6.0)

    print(f"{'L':>5} {'Rf':>5} {'rate':>5} {'lift':>6} {'jack':>6} {'tilt':>6}  "
          f"{'result':>8} {'peak_pitch':>10}")
    ok = []
    for L, Rf, rate in itertools.product(lengths, radii, rates):
        s = summarise(L, Rf)
        cfg = DeployCfg(deploy_rate=rate, contact_err=99.0)
        r = rollout_deploy(cfg=cfg, duration=14.0,
                           sole=dict(sole_half_len=L, sole_face_r=Rf))
        verdict = "FELL" if r["fell"] else r["state"]
        print(f"{L*1000:5.0f} {Rf*1000:5.1f} {rate:5.1f} "
              f"{s['lift_mm']:5.1f}  {s['jack_mm']:5.1f}  {s['tilt_deg']:5.1f}  "
              f"{verdict:>8} {r['peak_pitch']:10.3f}")
        if not r["fell"] and r["state"] == "STAND":
            ok.append((L, Rf, rate, s, r))

    print(f"\n{len(ok)} configurations reached STAND without falling")
    for L, Rf, rate, s, r in ok:
        print(f"  L={L*1000:.0f} mm Rf={Rf*1000:.1f} mm rate={rate} "
              f"-> tilt margin {s['tilt_deg']:.1f} deg, "
              f"wheel clear {r['wheel_clear']*1000:+.1f} mm, "
              f"stood {r['stand_duration']:.1f} s")
    return ok


if __name__ == "__main__":
    main()
