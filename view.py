"""Watch the balancer.  uv run python view.py

  W / S   drive forward / back      A / D   spin left / right (stage 1)
  SPACE   shove it forward          B       shove it backward
  R       reset
"""

import time

import mujoco
import mujoco.viewer
import numpy as np

from src.rsbot.model import load
from src.rsbot.sim import CTRL_HZ, obs
from src.rsbot.balance import Balancer

keys = {"v_des": 0.0, "shove": 0.0, "reset": False}


def on_key(code):
    c = chr(code) if 32 <= code < 127 else ""
    if c == "W":
        keys["v_des"] = 0.35
    elif c == "S":
        keys["v_des"] = -0.35
    elif c == "B":
        keys["shove"] = -1.0
    elif c == " ":
        keys["shove"] = 1.0
    elif c == "R":
        keys["reset"] = True
    else:
        keys["v_des"] = 0.0


def main():
    m, d = load()
    bal = Balancer()
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep
    shove_until = -1.0

    with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as v:
        k = 0
        while v.is_running():
            step_start = time.time()

            if keys["reset"]:
                mujoco.mj_resetDataKeyframe(m, d, 0)
                bal.reset()
                keys["reset"] = False

            if k % decim == 0:
                d.ctrl[:] = bal(obs(m, d), dt, keys["v_des"])

            d.xfrc_applied[torso] = 0.0
            if keys["shove"]:
                shove_until = d.time + 0.010
                keys["shove_mag"] = keys["shove"]
                keys["shove"] = 0.0
            if d.time < shove_until:
                d.xfrc_applied[torso, 0] = keys.get("shove_mag", 1.0) / 0.010

            mujoco.mj_step(m, d)
            k += 1
            v.sync()

            lag = m.opt.timestep - (time.time() - step_start)
            if lag > 0:
                time.sleep(lag)


if __name__ == "__main__":
    main()
