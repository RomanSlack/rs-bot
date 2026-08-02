"""The STS3215, drawn against the bench measurements.  uv run python -m cad.servo_view

    -> http://localhost:8782      drag to orbit, scroll to zoom

Every other viewer in this project renders a part so you can check the part.
This one exists to check the MODEL, because on 2026-08-01 two real servos turned
up and the model stopped being the only source of truth.

What you are looking at:

    grey solid      vendor/refs/STS3215_03a.step, the geometry the whole robot
                    is dimensioned against. Probably the 7.4 V variant.
    brass disc      the horn, built from bench measurements rather than from
                    any STEP: Ø19.93, a 2.51 mm plate on a 1.99 mm nub.
    blue wireframe  what cad/servo.py claims the envelope is
    amber wireframe what the calipers say it is

The two wireframes agree to 0.05 mm in x and 0.03 mm in y and are 2.36 mm apart
along z. That gap is the point of the picture: it is a real disagreement, it is
entirely in the direction that contains the gearbox, and it is the reason
CASE_Z_VERIFIED is False.

Nothing here is a design change. It is an instrument for looking at one.
"""

import json
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import build123d as bd

from cad import servo

PORT = 8782
OUT = Path(__file__).parent / "out" / "servo_view"


# --- geometry ------------------------------------------------------------------

def horn():
    """The horn, from the bench, in the servo's own frame.

    Seated the way it actually sits: the nub points DOWN at the crown and
    bottoms on the boss face, so the plate the wheel lands on floats
    HORN_NUB_T above that face. Getting this the right way up is the whole
    reason the Ø20.00 pocket turned out to be the fault and the 4.0 mm depth
    turned out not to be.
    """
    z0 = servo.HORN_FACE_Z                       # nub bottoms here
    z1 = z0 + servo.HORN_NUB_T                   # plate underside
    x = servo.SHAFT_X

    nub = bd.Pos(x, 0, z0 + servo.HORN_NUB_T / 2) * bd.Cylinder(
        servo.HORN_NUB_D_ASSUMED / 2, servo.HORN_NUB_T)
    plate = bd.Pos(x, 0, z1 + servo.HORN_PLATE_T / 2) * bd.Cylinder(
        servo.HORN_OD / 2, servo.HORN_PLATE_T)

    p = nub + plate
    # The splined bore, and the four tapped holes the wheel screws into.
    p -= bd.Pos(x, 0, z0 + servo.HORN_OVERALL_T / 2) * bd.Cylinder(
        servo.SHAFT_R + 0.05, servo.HORN_OVERALL_T * 2)
    for dx in (-servo.HORN_DX, servo.HORN_DX):
        for dy in (-servo.HORN_DY, servo.HORN_DY):
            p -= bd.Pos(x + dx, dy, z1) * bd.Cylinder(1.5, servo.HORN_OVERALL_T * 4)
    return p.clean()


def export(rebuild=False):
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [("servo.stl", servo.solid), ("horn.stl", horn)]
    for name, build in jobs:
        path = OUT / name
        if path.exists() and not rebuild:
            continue
        print(f"  building {name} ...", flush=True)
        bd.export_stl(build(), str(path), tolerance=0.02, angular_tolerance=0.1)
    return {name: (OUT / name) for name, _ in jobs}


# --- the numbers the page draws -------------------------------------------------

def dims():
    s = servo
    plate_z0 = s.HORN_FACE_Z + s.HORN_NUB_T
    plate_z1 = plate_z0 + s.HORN_PLATE_T
    # Where the wheel's outer face ends up, and how close that puts it to the
    # servo's boss face. This is the stack-up the pocket comment in wheel.py
    # refers to; it is computed here rather than repeated as a constant.
    pocket_depth = 4.0
    wheel_face_z = plate_z1 - pocket_depth
    boss_margin = wheel_face_z - s.HORN_FACE_Z

    rows = [
        ("case length, x", 45.40, s.MEASURED_LENGTH, "ok",
         "two independent readings, 0.05 mm apart"),
        ("case width, y", 24.80, s.MEASURED_WIDTH, "ok",
         "0.03 mm. As good as a caliper gets against CAD."),
        ("shaft tip to tip, z", 39.60, s.MEASURED_Z_SPAN, "bad",
         "2.36 mm short. Likely the 12V gearbox, not a modelling slip."),
        ("horn outer diameter", 20.00, s.HORN_OD, "fixed",
         "was a Ø20.00 pocket on a Ø19.93 horn. Pocket now Ø20.60."),
        ("horn plate thickness", None, s.HORN_PLATE_T, "ok",
         "the 4.5/3.6/2.5 drawing section decoded: 2.5 is the plate."),
        ("horn overall, with nub", None, s.HORN_OVERALL_T, "ok",
         "nub faces the crown, so it never enters the wheel pocket."),
        ("horn screw major dia", 3.00, s.HORN_SCREW_MEASURED, "ok",
         "M3 confirmed a third time, now off the physical screw."),
        ("spline proud of boss", 1.50, 1.0, "shaky",
         "operator flagged this reading. Decides which end lost the 2.36."),
        ("boss diameter", 20.00, None, "todo",
         "NOT MEASURED. The one number that could widen the pocket further."),
        ("horn bolt spacing", 4.95, None, "todo",
         "six mating patterns and a drawing behind it, but not the part."),
        ("case mount holes", None, None, "todo",
         "modelled as 4, all along the shaft axis. 40 M2 screws depend on "
         "them and no bracket in the kit uses them."),
        ("wheel pocket diameter", 20.60, None, "new",
         "clears the horn and the boss. Centring moves to the four screws."),
        ("wheel face to boss face", round(boss_margin, 2), None, "thin",
         "positive but only 0.5 mm, and it rests on an unverified HORN_FACE_Z."),
    ]
    return {
        "length": 45.40, "width": 24.80,
        "z_min": s.Z_MIN, "z_max": s.Z_MAX,
        "shaft_x": s.SHAFT_X, "shaft_r": s.SHAFT_R,
        "horn_face_z": s.HORN_FACE_Z,
        "idler_z": s.IDLER_Z, "idler_r": s.IDLER_R,
        "m_length": s.MEASURED_LENGTH, "m_width": s.MEASURED_WIDTH,
        "m_zspan": s.MEASURED_Z_SPAN,
        "horn_od": s.HORN_OD, "horn_plate_t": s.HORN_PLATE_T,
        "horn_nub_t": s.HORN_NUB_T, "horn_overall_t": s.HORN_OVERALL_T,
        "horn_dx": s.HORN_DX, "horn_dy": s.HORN_DY,
        "plate_z0": plate_z0, "plate_z1": plate_z1,
        "pocket_depth": pocket_depth, "wheel_face_z": wheel_face_z,
        "boss_margin": round(boss_margin, 2),
        "rows": [{"name": n, "cad": c, "bench": b, "state": st, "note": note}
                 for n, c, b, st, note in rows],
    }


PAGE = r"""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>STS3215 - model against bench</title><style>
*{box-sizing:border-box;margin:0;padding:0}
:root{--bg:#0e1013;--panel:#171a1f;--line:#252a32;--txt:#dfe3ea;--dim:#79828f;
 --acc:#4a9eff;--ok:#54c98a;--bad:#ff8b6b;--todo:#c9a227;--new:#9a7fe0}
body{background:var(--bg);color:var(--txt);
 font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;padding:16px;overflow:hidden}
.wrap{max-width:1600px;margin:0 auto;display:grid;
 grid-template-columns:1fr 400px;gap:14px;height:calc(100vh - 32px)}
h1{grid-column:1/-1;font-size:12px;font-weight:600;letter-spacing:.18em;
 text-transform:uppercase;color:var(--dim);display:flex;gap:12px;align-items:baseline}
h1 b{color:var(--txt);letter-spacing:.06em}
h1 i{font-style:normal;color:var(--bad);letter-spacing:.04em}
#stage{position:relative;border-radius:10px;overflow:hidden;background:#07080a;
 border:1px solid var(--line);cursor:grab}
#stage.drag{cursor:grabbing}
#hint{position:absolute;right:12px;bottom:12px;font-size:10px;color:#8e97a4;
 background:rgba(0,0,0,.5);padding:6px 9px;border-radius:6px;pointer-events:none}
#legend{position:absolute;left:12px;top:12px;font-size:11px;
 background:rgba(0,0,0,.55);padding:9px 11px;border-radius:6px;pointer-events:none;
 line-height:1.9;letter-spacing:.02em}
#legend s{display:inline-block;width:22px;height:0;border-top:2px solid;
 margin-right:8px;vertical-align:middle;text-decoration:none}
.side{display:flex;flex-direction:column;gap:12px;overflow-y:auto;padding-right:4px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{font-size:10px;letter-spacing:.16em;text-transform:uppercase;
 color:var(--dim);margin-bottom:9px;font-weight:600}
.row{display:flex;flex-wrap:wrap;gap:5px}
button{background:#20242b;color:var(--txt);border:1px solid #2d333c;border-radius:6px;
 padding:7px 10px;font:inherit;font-size:12px;cursor:pointer;transition:.12s}
button:hover{background:#2a303a;border-color:#3b434f}
button:active{transform:translateY(1px)}
button.on{outline:1px solid var(--acc);background:#1d3a57;border-color:#2b5580}
table{width:100%;border-collapse:collapse;font-size:11px}
th{text-align:left;color:var(--dim);font-weight:600;padding:0 0 6px;
 font-size:9.5px;letter-spacing:.1em;text-transform:uppercase}
th+th,td+td{text-align:right}
td{padding:4px 0;border-top:1px solid #1e222a;font-variant-numeric:tabular-nums}
td.n{text-align:left;color:var(--txt)}
td.d{font-weight:600}
tr.ok td.d{color:var(--ok)} tr.bad td.d{color:var(--bad)}
tr.fixed td.d{color:var(--acc)} tr.shaky td.d{color:var(--todo)}
tr.todo td.d{color:var(--todo)} tr.new td.d{color:var(--new)}
tr.thin td.d{color:var(--todo)}
tr.note td{border:0;padding:0 0 7px;color:var(--dim);font-size:10.5px;
 text-align:left;line-height:1.45}
kbd{background:#12151a;border:1px solid #2d333c;border-radius:3px;padding:0 4px;
 font:inherit;font-size:9.5px;color:var(--dim);margin-left:4px}
#hint b{color:#cfe4ff}
.label{color:#cfe4ff;font:11px ui-monospace,monospace;background:rgba(8,10,14,.82);
 padding:2px 6px;border-radius:4px;white-space:nowrap;pointer-events:none;
 border:1px solid #2a3342}
.label.bad{color:#ffc0ad;border-color:#5a2f22;background:rgba(30,10,6,.85)}
.label.ok{color:#bff0d4;border-color:#22503a;background:rgba(6,24,15,.85)}
.label.new{color:#d9cbff;border-color:#3d2f66;background:rgba(16,10,30,.85)}
</style></head><body><div class=wrap>

<h1><b>STS3215 C018</b> <span>model against bench, 2026-08-01</span>
 <i id=flag></i></h1>

<div id=stage>
  <div id=legend></div>
  <div id=hint>middle or left drag orbit &middot; shift+drag pan &middot;
   right-drag pan &middot; scroll zoom &middot; <b>5</b> ortho</div>
</div>

<div class=side>
  <div class=card>
    <h2>show</h2>
    <div class=row id=toggles></div>
  </div>
  <div class=card>
    <h2>views</h2>
    <div class=row>
      <button data-view=iso>iso <kbd>9</kbd></button>
      <button data-view=front>front <kbd>1</kbd></button>
      <button data-view=side>side <kbd>3</kbd></button>
      <button data-view=top>down the shaft <kbd>7</kbd></button>
      <button id=proj>perspective <kbd>5</kbd></button>
    </div>
    <p style="color:var(--dim);font-size:10.5px;margin-top:8px;line-height:1.45">
      Switch to orthographic before judging any dimension by eye. In
      perspective the far end of the case is drawn smaller than the near end.
    </p>
  </div>
  <div class=card>
    <h2>model against bench &nbsp;(mm)</h2>
    <table id=tbl></table>
  </div>
</div>
</div>

<script type="importmap">
{"imports":{
 "three":"https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js",
 "three/addons/":"https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/"}}
</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {CSS2DRenderer, CSS2DObject} from 'three/addons/renderers/CSS2DRenderer.js';

const D = await (await fetch('/dims')).json();
const stage = document.getElementById('stage');

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x07080a);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(devicePixelRatio);
renderer.domElement.style.touchAction = 'none';
stage.appendChild(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.style.position = 'absolute';
labelRenderer.domElement.style.top = '0';
labelRenderer.domElement.style.pointerEvents = 'none';
stage.appendChild(labelRenderer.domElement);

// TWO CAMERAS. Perspective to look at, orthographic to judge by. This page
// exists so a dimension can be eyeballed against a real part, and perspective
// makes the far end of a 45 mm case smaller than the near end, which is exactly
// the error you do not want while deciding whether a model matches a caliper.
// Blender's numpad-5 toggle, for the same reason Blender has it.
const VIEW_H = 115;                       // mm of model height visible in ortho
const persp = new THREE.PerspectiveCamera(38, 1, 1, 6000);
const ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, -2000, 6000);
let camera = persp, controls = null, isOrtho = false;
const target = new THREE.Vector3(0, 0, 0);

// Rebuilt rather than retargeted, because OrbitControls binds to one camera.
function bindControls(){
  if (controls){ target.copy(controls.target); controls.dispose(); }
  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.copy(target);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.rotateSpeed = 0.85;
  controls.zoomSpeed = 0.9;
  controls.panSpeed = 0.9;
  controls.screenSpacePanning = true;    // Blender pans in screen space
  controls.zoomToCursor = true;
  // Blender's mouse map. Middle drag orbits, shift+middle pans, wheel zooms.
  // Left also orbits and right also pans, because plenty of mice and every
  // trackpad here have no usable middle button.
  controls.mouseButtons = {LEFT: THREE.MOUSE.ROTATE,
                           MIDDLE: THREE.MOUSE.ROTATE,
                           RIGHT: THREE.MOUSE.PAN};
  controls.update();
}

// Capture phase, so the modifier is resolved before OrbitControls reads it.
renderer.domElement.addEventListener('pointerdown', e => {
  const mode = e.shiftKey ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE;
  if (e.button === 0) controls.mouseButtons.LEFT = mode;
  if (e.button === 1){ controls.mouseButtons.MIDDLE = mode; e.preventDefault(); }
}, true);
renderer.domElement.addEventListener('contextmenu', e => e.preventDefault());

function setProjection(useOrtho){
  isOrtho = useOrtho;
  const from = camera;
  camera = useOrtho ? ortho : persp;
  camera.position.copy(from.position);
  camera.lookAt(controls ? controls.target : target);
  bindControls();
  resize();
  document.getElementById('proj').classList.toggle('on', isOrtho);
  document.getElementById('proj').textContent =
    isOrtho ? 'orthographic' : 'perspective';
}

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.5); key.position.set(60,-90,120);
const fill = new THREE.DirectionalLight(0x88aaff, 0.6); fill.position.set(-80,60,-40);
const rim = new THREE.DirectionalLight(0xffd9a0, 0.5); rim.position.set(40,120,-80);
scene.add(key, fill, rim);

// Everything is modelled in the servo's own frame: x along the 45.4 length,
// y across the width, z along the OUTPUT SHAFT. The group is recentred so the
// orbit pivot sits in the middle of the case rather than at the frame origin.
const root = new THREE.Group();
root.position.set(0, 0, -(D.z_min + D.z_max) / 2);
const world = new THREE.Group();
// THE FIX FOR THE ORBIT. Everything below is authored in the servo's own frame,
// where the output shaft is +z. Three.js is Y-up, and the earlier version tried
// to force that by setting camera.up to +z. OrbitControls builds its azimuth and
// polar angles around the world +y axis, so a +z up vector left the model's own
// axis lying in the orbit's equator: the view rolled, and looking down the shaft
// sat exactly on the degenerate pole. Rotating the model into Y-up instead means
// OrbitControls runs on its defaults and behaves like every other 3D viewer.
world.rotation.x = -Math.PI / 2;          // servo +z (shaft) -> world +y (up)
world.add(root);
scene.add(world);

const groups = {};
function grp(name){ const g = new THREE.Group(); groups[name] = g; root.add(g); return g; }
const gServo = grp('servo'), gHorn = grp('horn'), gCad = grp('cad'),
      gBench = grp('bench'), gDims = grp('dims'), gPocket = grp('pocket');

// --- the two solids -----------------------------------------------------------
const loader = new STLLoader();
function stl(url, group, mat){
  loader.load(url, g => {
    g.computeVertexNormals();
    const m = new THREE.Mesh(g, mat);
    group.add(m);
    const e = new THREE.LineSegments(
      new THREE.EdgesGeometry(g, 32),
      new THREE.LineBasicMaterial({color:0x0a0c10, transparent:true, opacity:0.35}));
    group.add(e);
  });
}
stl('/mesh/servo.stl', gServo, new THREE.MeshStandardMaterial({
  color:0x59616e, metalness:0.30, roughness:0.55, flatShading:false}));
stl('/mesh/horn.stl', gHorn, new THREE.MeshStandardMaterial({
  color:0xd9a441, metalness:0.85, roughness:0.32}));

// --- the two envelopes, which are the actual argument -------------------------
function box(w, d, h, cz, colour, dashed){
  const g = new THREE.BoxGeometry(w, d, h);
  g.translate(0, 0, cz);
  const e = new THREE.EdgesGeometry(g);
  const mat = dashed
    ? new THREE.LineDashedMaterial({color:colour, dashSize:2.2, gapSize:1.6})
    : new THREE.LineBasicMaterial({color:colour});
  const l = new THREE.LineSegments(e, mat);
  if (dashed) l.computeLineDistances();
  return l;
}
// What cad/servo.py claims: a solid 45.40 x 24.80 x 39.60 slab.
gCad.add(box(D.length, D.width, D.z_max - D.z_min,
             (D.z_min + D.z_max) / 2, 0x4a9eff, false));
// What the calipers say, hung off the crown tip so the gap shows at the rear,
// which is where the gearbox is and where the 2.36 mm is most likely hiding.
gBench.add(box(D.m_length, D.m_width, D.m_zspan,
               D.z_max - D.m_zspan / 2, 0xffa24a, true));

// --- the wheel pocket, so the fit is visible rather than asserted -------------
{
  const pd = 20.6, depth = D.pocket_depth;
  const g = new THREE.CylinderGeometry(pd/2, pd/2, depth, 64, 1, true);
  g.rotateX(Math.PI/2);
  g.translate(D.shaft_x, 0, D.wheel_face_z + depth/2);
  gPocket.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({
    color:0x9a7fe0, transparent:true, opacity:0.22, side:THREE.DoubleSide,
    metalness:0.1, roughness:0.9})));
  const ring = new THREE.LineBasicMaterial({color:0x9a7fe0, transparent:true,
                                            opacity:0.8});
  for (const z of [D.wheel_face_z, D.wheel_face_z + depth]){
    const pts = [];
    for (let i = 0; i <= 72; i++){
      const a = i / 72 * Math.PI * 2;
      pts.push(new THREE.Vector3(D.shaft_x + Math.cos(a)*pd/2,
                                 Math.sin(a)*pd/2, z));
    }
    gPocket.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), ring));
  }
}

// --- dimensions ---------------------------------------------------------------
function label(text, pos, cls){
  const el = document.createElement('div');
  el.className = 'label' + (cls ? ' ' + cls : '');
  el.textContent = text;
  const o = new CSS2DObject(el);
  o.position.copy(pos);
  return o;
}
function dim(a, b, text, colour, cls, off){
  const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b);
  const O = new THREE.Vector3(...(off || [0,0,0]));
  const A2 = A.clone().add(O), B2 = B.clone().add(O);
  const g = new THREE.Group();
  const mat = new THREE.LineBasicMaterial({color:colour});
  g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([A2,B2]), mat));
  // witness lines back to the feature being measured
  const wit = new THREE.LineDashedMaterial({color:colour, dashSize:1.2, gapSize:1.0,
                                            transparent:true, opacity:0.5});
  for (const [p,q] of [[A,A2],[B,B2]]){
    const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints([p,q]), wit);
    l.computeLineDistances(); g.add(l);
  }
  g.add(label(text, A2.clone().lerp(B2, 0.5), cls));
  gDims.add(g);
}

const hx = D.length/2, hy = D.width/2;
dim([-hx, hy, D.z_min], [hx, hy, D.z_min],
    'length  45.40 cad / 45.35 bench  Δ0.05', 0x54c98a, 'ok', [0, 11, -5]);
dim([hx, -hy, D.z_min], [hx, hy, D.z_min],
    'width  24.80 / 24.77  Δ0.03', 0x54c98a, 'ok', [10, 0, -7]);
dim([-hx, -hy, D.z_min], [-hx, -hy, D.z_max],
    'shaft span  39.60 cad', 0x4a9eff, '', [-13, -8, 0]);
dim([-hx, hy, D.z_max - D.m_zspan], [-hx, hy, D.z_max],
    'bench  37.24   Δ2.36 UNEXPLAINED', 0xff8b6b, 'bad', [-21, 7, 0]);
dim([D.shaft_x - D.horn_od/2, 0, D.plate_z1], [D.shaft_x + D.horn_od/2, 0, D.plate_z1],
    'horn Ø19.93 bench  (pocket was Ø20.00)', 0xd9a441, '', [0, 0, 10]);
dim([D.shaft_x + D.horn_od/2, 0, D.plate_z0], [D.shaft_x + D.horn_od/2, 0, D.plate_z1],
    'plate 2.51', 0xd9a441, '', [13, 0, 0]);
dim([D.shaft_x + D.horn_od/2, 0, D.horn_face_z], [D.shaft_x + D.horn_od/2, 0, D.plate_z1],
    'overall 4.50, nub faces the crown', 0xd9a441, '', [21, -17, 0]);
dim([D.shaft_x - 13, 0, D.horn_face_z], [D.shaft_x - 13, 0, D.wheel_face_z],
    'wheel face clears boss by ' + D.boss_margin, 0xc9a227, 'new', [-7, 0, 0]);
dim([D.shaft_x - 10.3, 0, D.wheel_face_z], [D.shaft_x + 10.3, 0, D.wheel_face_z],
    'new pocket Ø20.60', 0x9a7fe0, 'new', [0, 0, -8]);

// --- toggles ------------------------------------------------------------------
const TOG = [['servo','servo STEP',1],['horn','horn (bench)',1],
             ['cad','cad envelope',1],['bench','bench envelope',1],
             ['pocket','wheel pocket',1],['dims','dimensions',1]];
const bar = document.getElementById('toggles');
for (const [k, text, on] of TOG){
  const b = document.createElement('button');
  b.textContent = text; b.className = on ? 'on' : '';
  groups[k].visible = !!on;
  b.onclick = () => { groups[k].visible = !groups[k].visible;
                      b.classList.toggle('on', groups[k].visible); };
  bar.appendChild(b);
}

document.getElementById('legend').innerHTML =
  '<s style="border-color:#4a9eff"></s>cad/servo.py envelope, 39.60<br>' +
  '<s style="border-color:#ffa24a;border-top-style:dashed"></s>bench envelope, 37.24<br>' +
  '<s style="border-color:#d9a441"></s>horn, measured Ø19.93 x 2.51<br>' +
  '<s style="border-color:#9a7fe0"></s>wheel pocket, new Ø20.60';
document.getElementById('flag').textContent =
  'z span disagrees by ' + (39.60 - D.m_zspan).toFixed(2) + ' mm';

// --- the table ----------------------------------------------------------------
const fmt = v => v === null ? '<span style="color:#4d545e">-</span>'
                            : (typeof v === 'number' ? v.toFixed(2) : v);
let html = '<tr><th>dimension</th><th>cad</th><th>bench</th><th>Δ</th></tr>';
for (const r of D.rows){
  const d = (r.cad !== null && r.bench !== null)
    ? Math.abs(r.cad - r.bench).toFixed(2) : '';
  html += `<tr class="${r.state}"><td class=n>${r.name}</td>` +
          `<td>${fmt(r.cad)}</td><td>${fmt(r.bench)}</td><td class=d>${d || r.state}</td></tr>` +
          `<tr class=note><td colspan=4>${r.note}</td></tr>`;
}
document.getElementById('tbl').innerHTML = html;

// --- views --------------------------------------------------------------------
// World space now, not servo space: +y is up and is the output shaft.
const R = 150;
const VIEWS = {iso:[1, 0.72, 1.15], front:[0, 0, 1], side:[1, 0, 0],
               top:[0.08, 1, 0.08]};
function view(name){
  const v = VIEWS[name] || VIEWS.iso;
  const L = Math.hypot(...v);
  controls.target.set(0, 0, 0);
  target.set(0, 0, 0);
  camera.position.set(v[0]/L*R, v[1]/L*R, v[2]/L*R);
  if (isOrtho){ camera.zoom = 1; camera.updateProjectionMatrix(); }
  controls.update();
}
for (const b of document.querySelectorAll('[data-view]'))
  b.onclick = () => view(b.dataset.view);
document.getElementById('proj').onclick = () => setProjection(!isOrtho);

// Blender's numpad, for anyone who already has the muscle memory.
addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  const k = {'1':'front', '3':'side', '7':'top', '9':'iso', '0':'iso'}[e.key];
  if (k) view(k);
  else if (e.key === '5') setProjection(!isOrtho);
  else if (e.key === 'f' || e.key === '.') view('iso');
});

function resize(){
  const w = stage.clientWidth, h = stage.clientHeight, a = w / h;
  persp.aspect = a; persp.updateProjectionMatrix();
  ortho.left = -VIEW_H * a / 2; ortho.right = VIEW_H * a / 2;
  ortho.top = VIEW_H / 2; ortho.bottom = -VIEW_H / 2;
  ortho.updateProjectionMatrix();
  renderer.setSize(w, h); labelRenderer.setSize(w, h);
}
addEventListener('resize', resize);

bindControls();
resize();
view('iso');

(function loop(){
  requestAnimationFrame(loop);
  controls.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
})();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self._send(PAGE.encode(), "text/html; charset=utf-8")
        elif u.path == "/dims":
            self._send(json.dumps(dims()).encode(), "application/json")
        elif u.path.startswith("/mesh/"):
            p = OUT / Path(u.path).name
            if p.exists():
                self._send(p.read_bytes(), "model/stl")
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()


def main():
    rebuild = "--rebuild" in sys.argv
    print("STS3215 viewer")
    export(rebuild)

    d = dims()
    print()
    print("  the stack-up at the horn joint, computed rather than asserted:")
    print(f"    boss face            z = {d['horn_face_z']:.2f}")
    print(f"    horn plate           z = {d['plate_z0']:.2f} .. {d['plate_z1']:.2f}")
    print(f"    wheel outer face     z = {d['wheel_face_z']:.2f}"
          f"   ({d['pocket_depth']:.1f} mm pocket)")
    print(f"    clearance to boss        {d['boss_margin']:.2f} mm"
          "   positive, but thin and unverified")
    print()
    print(f"  z span   {39.60:.2f} modelled   {d['m_zspan']:.2f} bench"
          f"   {39.60 - d['m_zspan']:.2f} unexplained")
    print()
    url = f"http://localhost:{PORT}"
    print(f"  {url}")
    threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
