"""Persistent live sim in the browser.  uv run python serve.py  ->  localhost:8781

The sim runs continuously in a background thread and never resets itself, so
you can leave the tab open and poke at the robot. Rendering goes through EGL
offscreen because GLX windows don't work in this environment; frames are
pushed to the browser as MJPEG.

Controls are the same ones the headless tests drive, so anything you can do
here is something the test suite can reproduce.
"""

import io
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from src.rsbot.model import WHEEL_HALF_W, load  # noqa: E402
from src.rsbot.sim import CTRL_HZ, obs  # noqa: E402
from src.rsbot.balance import pitch_from_quat  # noqa: E402
from src.rsbot.transition import DeployMachine, NAMES, STAND  # noqa: E402

W, H, FPS = 900, 520, 30
PORT = 8781


class Sim:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = b""
        self.cam = mujoco.MjvCamera()
        self.cam.distance, self.cam.elevation, self.cam.azimuth = 0.95, -8, 132
        self.speed = 1.0
        self.v_des = 0.0
        self.shove = 0.0
        self.follow = True
        self._reset()

    def _reset(self):
        self.m, self.d = load()
        self.mach = DeployMachine()
        self.torso = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "torso")
        self.wheel = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")
        self.decim = int(round(1.0 / (CTRL_HZ * self.m.opt.timestep)))
        self.k = 0
        self.shove_until = -1.0
        self.pitch = 0.0

    def command(self, q):
        a = q.get("a", [""])[0]
        v = float(q.get("v", ["0"])[0]) if "v" in q else 0.0
        with self.lock:
            if a == "reset":
                self._reset()
            elif a == "drive":
                self.v_des = v
            elif a == "shove":
                self.shove = v
                self.shove_until = self.d.time + 0.010
            elif a == "deploy":
                self.mach.start_deploy()
            elif a == "retract":
                self.mach.start_retract()
            elif a == "toggle":
                self.mach.toggle()
            elif a == "speed":
                self.speed = max(0.05, min(2.0, v))
            elif a == "cam":
                self.cam.azimuth += float(q.get("az", ["0"])[0])
                self.cam.elevation = max(-85, min(5, self.cam.elevation
                                                 + float(q.get("el", ["0"])[0])))
                self.cam.distance = max(0.3, min(3.0, self.cam.distance
                                                * float(q.get("dz", ["1"])[0])))
            elif a == "follow":
                self.follow = not self.follow

    def telemetry(self):
        with self.lock:
            return {
                "state": NAMES[self.mach.state],
                "t": round(self.d.time, 2),
                "pitch": round(float(np.degrees(self.pitch)), 2),
                "x": round(float(self.d.xpos[self.torso][0]), 3),
                "axle_mm": round(float(self.d.xpos[self.wheel][2]) * 1000, 1),
                "roll_deg": round(float(np.degrees(self.mach.roll)), 1),
                "v_des": self.v_des,
                "speed": self.speed,
                "standing": self.mach.state == STAND,
            }

    def run(self):
        renderer = mujoco.Renderer(self.m, H, W)
        next_frame = time.perf_counter()
        next_step = time.perf_counter()
        while True:
            with self.lock:
                dt = self.m.opt.timestep
                if self.k % self.decim == 0:
                    o = obs(self.m, self.d)
                    self.pitch = pitch_from_quat(o["quat"])
                    # Driving only has an effect while still on the wheels;
                    # the machine ignores it once the transition starts.
                    self.d.ctrl[:] = self.mach(o, self.decim * dt,
                                               v_des=self.v_des)

                self.d.xfrc_applied[self.torso] = 0.0
                if self.d.time < self.shove_until:
                    self.d.xfrc_applied[self.torso, 0] = self.shove / 0.010

                mujoco.mj_step(self.m, self.d)
                self.k += 1

            next_step += dt / self.speed
            lag = next_step - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            elif lag < -0.25:
                next_step = time.perf_counter()

            now = time.perf_counter()
            if now >= next_frame:
                next_frame = now + 1.0 / FPS
                with self.lock:
                    if self.follow:
                        self.cam.lookat[0] = self.d.xpos[self.torso][0]
                        self.cam.lookat[2] = 0.20
                    renderer.update_scene(self.d, camera=self.cam)
                    rgb = renderer.render()
                buf = io.BytesIO()
                Image.fromarray(rgb).save(buf, "JPEG", quality=80)
                self.jpeg = buf.getvalue()


SIM = Sim()

PAGE = """<!doctype html><html><head><meta charset=utf-8>
<title>rs-bot</title><style>
*{box-sizing:border-box}
body{margin:0;background:#14161a;color:#d8dce3;font:14px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
.wrap{max-width:940px;margin:0 auto;padding:18px}
h1{font-size:15px;font-weight:600;letter-spacing:.14em;text-transform:uppercase;color:#8a929e;margin:0 0 14px}
img{width:100%;border-radius:6px;display:block;background:#000}
.row{display:flex;gap:6px;flex-wrap:wrap;margin-top:10px}
button{background:#232830;color:#d8dce3;border:1px solid #333a45;border-radius:5px;
padding:8px 13px;font:inherit;cursor:pointer}
button:hover{background:#2c323c;border-color:#4a5361}
button:active{background:#3a424f}
button.go{background:#2d4a63;border-color:#3d6383}
button.hot{background:#5e3527;border-color:#7d4633}
#tel{margin-top:12px;display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.c{background:#1b1f26;border:1px solid #262c35;border-radius:5px;padding:8px 10px}
.c b{display:block;color:#6f7885;font-size:10px;letter-spacing:.1em;text-transform:uppercase}
.c span{font-size:16px}
.ok{color:#6cc08b}.warn{color:#d99a5b}
</style></head><body><div class=wrap>
<h1>rs-bot &mdash; live</h1>
<img src="/stream">
<div class=row>
  <button onmousedown="c('drive',0.35)" onmouseup="c('drive',0)">&#9654; forward</button>
  <button onmousedown="c('drive',-0.35)" onmouseup="c('drive',0)">&#9664; back</button>
  <button onclick="c('drive',0)">stop</button>
  <button class=hot onclick="c('shove',1.0)">shove &rarr;</button>
  <button class=hot onclick="c('shove',-1.0)">&larr; shove</button>
  <button class=go onclick="c('deploy')">flip to feet</button>
  <button class=go onclick="c('retract')">back to wheels</button>
  <button onclick="c('reset')">reset</button>
</div>
<div class=row>
  <button onclick="cam(-15,0,1)">&#8630; orbit</button>
  <button onclick="cam(15,0,1)">orbit &#8631;</button>
  <button onclick="cam(0,-6,1)">tilt up</button>
  <button onclick="cam(0,6,1)">tilt dn</button>
  <button onclick="cam(0,0,0.85)">zoom in</button>
  <button onclick="cam(0,0,1.18)">zoom out</button>
  <button onclick="c('follow')">follow on/off</button>
  <button onclick="c('speed',0.15)">0.15x</button>
  <button onclick="c('speed',0.5)">0.5x</button>
  <button onclick="c('speed',1)">1x</button>
</div>
<div id=tel></div>
</div><script>
const F=['state','t','pitch','x','axle_mm','roll_deg','speed'];
const L={t:'sim time',pitch:'pitch deg',x:'x m',axle_mm:'axle height mm',
         roll_deg:'wheel roll deg',speed:'rate'};
function c(a,v){fetch(`/cmd?a=${a}&v=${v??0}`)}
function cam(az,el,dz){fetch(`/cmd?a=cam&az=${az}&el=${el}&dz=${dz}`)}
async function tick(){
  try{const d=await (await fetch('/tel')).json();
    document.getElementById('tel').innerHTML=F.map(k=>{
      let cl='';
      if(k=='state')cl=d.standing?'ok':'';
      if(k=='axle_mm')cl=d[k]<20?'ok':'';
      return `<div class=c><b>${L[k]||k}</b><span class="${cl}">${d[k]}</span></div>`}).join('');
  }catch(e){}
}
setInterval(tick,200);tick();
document.addEventListener('keydown',e=>{
  if(e.repeat)return;
  if(e.key=='w')c('drive',0.35); if(e.key=='s')c('drive',-0.35);
  if(e.key==' ')c('shove',1.0); if(e.key=='d')c('toggle'); if(e.key=='r')c('reset');
});
document.addEventListener('keyup',e=>{if(e.key=='w'||e.key=='s')c('drive',0)});
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/tel":
            body = json.dumps(SIM.telemetry()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/cmd":
            SIM.command(parse_qs(u.query))
            self.send_response(204)
            self.end_headers()
        elif u.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                while True:
                    frame = SIM.jpeg
                    if frame:
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: "
                                         + str(len(frame)).encode()
                                         + b"\r\n\r\n" + frame + b"\r\n")
                    time.sleep(1.0 / FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()


if __name__ == "__main__":
    threading.Thread(target=SIM.run, daemon=True).start()
    print(f"rs-bot live at http://localhost:{PORT}")
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
