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
import math
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
from src.rsbot.balance import LASH_GAINS  # noqa: E402

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
        self.yaw_des = 0.0
        self.shove = 0.0
        self.follow = True
        self.lash = 0.0
        # The real printed parts, not the stand-in boxes. Visual only: the
        # collision shapes and the CAD-derived inertials are the same either
        # way, so this changes what you see and nothing about what it does.
        self.meshes = True
        self.pan = [0.0, 0.0]      # x,z offset on the follow point
        self._reset()

    def _reset(self):
        self.m, self.d = load(backlash=self.lash, meshes=self.meshes)
        self.mach = DeployMachine(gains=LASH_GAINS if self.lash else None)
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
            elif a == "turn":
                self.yaw_des = v
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
                self.cam.elevation = max(-85, min(20, self.cam.elevation
                                                 + float(q.get("el", ["0"])[0])))
                self.cam.distance = max(0.18, min(6.0, self.cam.distance
                                                 * float(q.get("dz", ["1"])[0])))
            elif a == "pan":
                # Scale with distance so panning feels the same at any zoom.
                k = self.cam.distance
                self.pan[0] += float(q.get("dx", ["0"])[0]) * k
                self.pan[1] += float(q.get("dy", ["0"])[0]) * k
            elif a == "home":
                self.cam.distance, self.cam.elevation, self.cam.azimuth = 0.95, -8, 132
                self.pan = [0.0, 0.0]
            elif a == "lash":
                self.lash = math.radians(v)
                self._reset()
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
                "lash_deg": round(float(np.degrees(self.lash)), 2),
                "v_des": self.v_des,
                "yaw_des": self.yaw_des,
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
                                               v_des=self.v_des,
                                               yaw_des=self.yaw_des)

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
                        self.cam.lookat[0] = self.d.xpos[self.torso][0] + self.pan[0]
                        self.cam.lookat[1] = self.d.xpos[self.torso][1]
                        self.cam.lookat[2] = 0.20 + self.pan[1]
                    renderer.update_scene(self.d, camera=self.cam)
                    rgb = renderer.render()
                buf = io.BytesIO()
                Image.fromarray(rgb).save(buf, "JPEG", quality=80)
                self.jpeg = buf.getvalue()


SIM = Sim()

PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>rs-bot</title><style>
*{box-sizing:border-box;margin:0;padding:0}
:root{--bg:#0e1013;--panel:#171a1f;--line:#252a32;--txt:#dfe3ea;--dim:#79828f;--acc:#4a9eff}
body{background:var(--bg);color:var(--txt);
 font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;padding:16px}
.wrap{max-width:1180px;margin:0 auto;display:grid;grid-template-columns:1fr 260px;gap:14px}
h1{grid-column:1/-1;font-size:12px;font-weight:600;letter-spacing:.18em;
 text-transform:uppercase;color:var(--dim);display:flex;gap:12px;align-items:center}
h1 b{color:var(--txt);letter-spacing:.06em}
#stage{position:relative;border-radius:10px;overflow:hidden;background:#000;
 border:1px solid var(--line);cursor:grab}
#stage.drag{cursor:grabbing}
#view{width:100%;display:block;user-select:none;-webkit-user-drag:none}
#hud{position:absolute;left:12px;top:12px;font-size:11px;color:#aeb6c2;
 background:rgba(0,0,0,.55);padding:7px 10px;border-radius:6px;line-height:1.7;
 pointer-events:none;letter-spacing:.03em}
#hud b{color:#fff}
#hint{position:absolute;right:12px;bottom:12px;font-size:10px;color:#8e97a4;
 background:rgba(0,0,0,.5);padding:6px 9px;border-radius:6px;pointer-events:none}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{font-size:10px;letter-spacing:.16em;text-transform:uppercase;
 color:var(--dim);margin-bottom:9px;font-weight:600}
.row{display:flex;flex-wrap:wrap;gap:5px}
button{background:#20242b;color:var(--txt);border:1px solid #2d333c;border-radius:6px;
 padding:7px 10px;font:inherit;font-size:12px;cursor:pointer;transition:.12s}

button:hover{background:#2a303a;border-color:#3b434f}
button:active{transform:translateY(1px)}
button.go{background:#1d3a57;border-color:#2b5580;color:#cfe4ff}
button.hot{background:#4a2a1e;border-color:#6d3f2c;color:#ffd8c6}
button.on{outline:1px solid var(--acc)}
.side{display:flex;flex-direction:column;gap:12px}
table{width:100%;border-collapse:collapse;font-size:11.5px}
td{padding:2.5px 0;color:var(--dim)}
td+td{text-align:right;color:var(--txt);font-variant-numeric:tabular-nums}
kbd{background:#20242b;border:1px solid #2d333c;border-radius:4px;padding:1px 5px;font-size:10px}
</style></head><body><div class=wrap>

<h1><b>rs-bot</b> <span id=title>wheel mode</span></h1>

<div>
  <div id=stage>
    <img id=view src="/stream" draggable=false>
    <div id=hud></div>
    <div id=hint>drag orbit &middot; shift+drag pan &middot; scroll zoom &middot; dbl-click reset view</div>
  </div>
  <div class=card style="margin-top:12px">
    <h2>drive</h2>
    <div class=row>
      <button onmousedown="c('drive',0.35)" onmouseup="c('drive',0)" onmouseleave="c('drive',0)">&#9654;&nbsp;forward <kbd>W</kbd></button>
      <button onmousedown="c('drive',-0.35)" onmouseup="c('drive',0)" onmouseleave="c('drive',0)">&#9664;&nbsp;back <kbd>S</kbd></button>
      <button onclick="c('drive',0)">stop</button>
      <button onmousedown="c('turn',1.0)" onmouseup="c('turn',0)" onmouseleave="c('turn',0)">&#8630;&nbsp;left <kbd>A</kbd></button>
      <button onmousedown="c('turn',-1.0)" onmouseup="c('turn',0)" onmouseleave="c('turn',0)">right&nbsp;&#8631; <kbd>D</kbd></button>
      <button class=hot onclick="c('shove',1.0)">shove &rarr; <kbd>space</kbd></button>
      <button class=hot onclick="c('shove',-1.0)">&larr; shove</button>
      <button class=go onclick="c('toggle')">flip mode <kbd>F</kbd></button>
      <button onclick="c('reset')">reset <kbd>R</kbd></button>
    </div>
  </div>
</div>

<div class=side>
  <div class=card>
    <h2>telemetry</h2>
    <table id=tel></table>
  </div>
  <div class=card>
    <h2>dimensions</h2>
    <table>
      <tr><td>height, wheels</td><td>16.8 in</td></tr>
      <tr><td>height, feet</td><td>15.2 in</td></tr>
      <tr><td>width</td><td>8.6 in</td></tr>
      <tr><td>depth</td><td>5.2 in</td></tr>
      <tr><td>wheel dia</td><td>3.15 in</td></tr>
      <tr><td>foot polygon</td><td>3.15 in</td></tr>
      <tr><td>mass</td><td>4.5 lb</td></tr>
      <tr><td>human shown</td><td>5 ft 9 in</td></tr>
    </table>
  </div>
  <div class=card>
    <h2>sim rate</h2>
    <div class=row>
      <button onclick="c('speed',0.15)">0.15x</button>
      <button onclick="c('speed',0.5)">0.5x</button>
      <button onclick="c('speed',1)">1x</button>
    </div>
  </div>
  <div class=card>
    <h2>gear backlash</h2>
    <div class=row>
      <button onclick="c('lash',0)">none</button>
      <button onclick="c('lash',1)">1&deg;</button>
      <button onclick="c('lash',2)">2&deg;</button>
      <button onclick="c('lash',3)">3&deg;</button>
    </div>
  </div>
  <div class=card>
    <h2>view</h2>
    <div class=row>
      <button onclick="c('home')">reset view</button>
      <button onclick="c('follow')">follow on/off</button>
    </div>
  </div>
</div>
</div><script>
const F={state:'state',t:'sim time s',pitch:'pitch deg',x:'travelled m',
         axle_mm:'axle height mm',roll_deg:'wheel roll deg',lash_deg:'backlash deg',
         speed:'rate'};
const CAP={WHEEL:'wheel mode - balancing',SETTLE:'settling',
           FLIP:'flipping to feet',STAND:'foot mode - standing',
           UNFLIP:'flipping back to wheels'};
function c(a,v){fetch(`/cmd?a=${a}&v=${v??0}`)}
function cam(q){fetch('/cmd?a=cam&'+q)}

const stage=document.getElementById('stage');
let drag=null;
stage.addEventListener('mousedown',e=>{drag={x:e.clientX,y:e.clientY,pan:e.shiftKey||e.button==2};
  stage.classList.add('drag');e.preventDefault()});
addEventListener('mouseup',()=>{drag=null;stage.classList.remove('drag')});
addEventListener('mousemove',e=>{
  if(!drag)return;
  const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
  if(Math.abs(dx)<2&&Math.abs(dy)<2)return;
  drag.x=e.clientX; drag.y=e.clientY;
  if(drag.pan) fetch(`/cmd?a=pan&dx=${(-dx*0.0022).toFixed(4)}&dy=${(dy*0.0022).toFixed(4)}`);
  else cam(`az=${(-dx*0.35).toFixed(2)}&el=${(-dy*0.25).toFixed(2)}`);
});
stage.addEventListener('wheel',e=>{e.preventDefault();
  cam(`dz=${e.deltaY>0?1.09:0.917}`)},{passive:false});
stage.addEventListener('contextmenu',e=>e.preventDefault());
stage.addEventListener('dblclick',()=>c('home'));

async function tick(){
  try{const d=await (await fetch('/tel')).json();
    document.getElementById('title').textContent=CAP[d.state]||d.state;
    document.getElementById('hud').innerHTML=
      `<b>${d.state}</b><br>roll ${d.roll_deg}&deg;<br>axle ${d.axle_mm} mm<br>pitch ${d.pitch}&deg;`;
    document.getElementById('tel').innerHTML=Object.keys(F).map(k=>
      `<tr><td>${F[k]}</td><td>${d[k]}</td></tr>`).join('');
  }catch(e){}
}
setInterval(tick,200);tick();
const held={};
addEventListener('keydown',e=>{
  if(held[e.key])return; held[e.key]=1;
  if(e.key=='w')c('drive',0.35); else if(e.key=='s')c('drive',-0.35);
  else if(e.key==' '){c('shove',1.0);e.preventDefault()}
  else if(e.key=='a')c('turn',1.0); else if(e.key=='d')c('turn',-1.0);
  else if(e.key=='f')c('toggle'); else if(e.key=='r')c('reset');
});
addEventListener('keyup',e=>{delete held[e.key];
  if(e.key=='w'||e.key=='s')c('drive',0);
  if(e.key=='a'||e.key=='d')c('turn',0)});
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
