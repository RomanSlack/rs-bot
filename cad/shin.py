"""Proof of concept: one real printable part, and its MuJoCo mesh.

    uv run python cad/shin.py

Demonstrates the whole loop in ~40 lines:

    build123d  ->  STEP (send to a printer or a machine shop)
               ->  STL  (MuJoCo asset)
    MuJoCo     ->  mass and inertia computed from the actual geometry

The two traps, both found by measuring rather than by reading docs:

  * build123d exports millimetres, MuJoCo assumes metres. Without
    scale="0.001 0.001 0.001" the part comes out 1e9 times too heavy.
  * MuJoCo defaults to the CONVEX HULL for mesh inertia, which fills every
    bolt hole and pocket. inertia="exact" takes the error from 2.1% to 0.001%.
"""

import os
from pathlib import Path

import build123d as bd
import mujoco

PETG = 1270.0          # kg/m3
OUT = Path(__file__).parent / "out"

# Shin spine, from the dimensions the sim already uses.
LEN, THICK, WIDTH = 92.0, 20.0, 12.0
BOLT_R, BOLT_Z = 1.1, (-30, -10, 10, 30)


def build():
    with bd.BuildPart() as part:
        bd.Box(THICK, WIDTH, LEN)
        with bd.Locations(*[(0, 0, z) for z in BOLT_Z]):
            bd.Hole(radius=BOLT_R)
        bd.fillet(part.edges().filter_by(bd.Axis.X), radius=1.5)
    return part.part


def main():
    OUT.mkdir(exist_ok=True)
    p = build()
    step, stl = OUT / "shin_spine.step", OUT / "shin_spine.stl"
    bd.export_step(p, str(step))
    bd.export_stl(p, str(stl), tolerance=0.01, angular_tolerance=0.1)

    cad_mass = p.volume / 1e9 * PETG
    xml = f'''<mujoco>
  <asset><mesh name="spine" file="{stl}"
               scale="0.001 0.001 0.001" inertia="exact"/></asset>
  <worldbody><body><freejoint/>
    <geom type="mesh" mesh="spine" density="{PETG}"/>
  </body></worldbody>
</mujoco>'''
    m = mujoco.MjModel.from_xml_string(xml)

    print(f"STEP  {step}  ({step.stat().st_size/1024:.0f} kB)")
    print(f"STL   {stl}  ({stl.stat().st_size/1024:.0f} kB)")
    print(f"CAD    mass {cad_mass*1000:8.3f} g")
    print(f"MuJoCo mass {m.body_mass[1]*1000:8.3f} g"
          f"   ({100*(m.body_mass[1]/cad_mass - 1):+.3f}%)")
    print(f"MuJoCo diaginertia {m.body_inertia[1]} kg m2")


if __name__ == "__main__":
    main()
