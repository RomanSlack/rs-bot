"""The assembled robot, in a browser, with every gap measured.

    uv run python -m cad.fitview [wheel|foot] [--rebuild]

Every other viewer in this project shows one part. `cad/robot.py` shows all of
them but as a still PNG, which is enough to notice a part in the wrong place and
useless for the question you actually have at the bench: does this go together,
and where is it tight.

So this one draws the REAL solids in the pose the SIMULATOR puts them in - the
same `cad.assemble_check.parts()` that the interference checks run on, so what
you are looking at is what was checked - and it measures every pairwise gap
rather than leaving you to judge distance by eye on a screen.

WHY MEASURED AND NOT EYEBALLED. A 0.9 mm running clearance and a 0.0 mm seating
face look identical at any zoom that shows you the whole leg, and this robot has
sixteen faces that are SUPPOSED to touch and one pair that must never get closer
than 0.8 mm. Eyeballing cannot tell those apart. The gap list can, and it is
sorted so the tightest thing in the robot is the first line.

The explode slider is the other half. Parts are pushed out along the line from
the robot's centre, which is roughly the direction each one comes off during
assembly, so you can see the order things go together and whether a fastener
ends up buried.
"""

import itertools
import json
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import build123d as bd
import mujoco
import numpy as np

import cad.assemble_check as ac
import cad.servo as servo
import cad.wiring as wiring

PORT = 8783
OUT = Path(__file__).parent / "out"

# Bought parts are drawn in a different colour from printed ones, because the
# distinction is the whole risk model: a printed part is a fortnight and $30, a
# bought part is a part number you have to get right the first time.
BOUGHT_PREFIX = ("vhipsv", "vkneesv", "vanksv", "vrollsv", "vwhlsv")


def _kind(name):
    """What a part is MADE OF, because that is what you are looking for.

    A viewer that colours by role tells you what you already know from the
    label. Colouring by material tells you something the model knows and you
    cannot see: which parts arrive in a box and which come off a printer, and
    which of the black things is rubber and which is a servo case.
    """
    if name.startswith(BOUGHT_PREFIX):
        return "servo"          # dark grey engineering plastic
    if name.startswith("cable"):
        return "cable"          # PVC sleeve
    if name.startswith("belt"):
        return "belt"           # black rubber, glass-corded
    if name.startswith("pulley"):
        return "alu"            # anodised aluminium
    if name.startswith(("shaft", "bearing")):
        return "steel"          # bright chrome steel
    if name.startswith("horn"):
        return "horn"           # the metal horn, plated
    if name.endswith(("wheel_l", "wheel_r")) or name.startswith("wheel"):
        return "tyre"           # TPU tread over a printed hub
    return "printed"            # PA6-CF, matte black-brown


_SERVO_SOLID = {}


def _servo_solid(order):
    """The REAL servo, from the manufacturer's drawing, on the given axes.

    cad/assemble_check.py draws these as plain boxes and is right to: it
    measures interference from oriented bounding boxes, and a box is the
    conservative shape for that. It is the wrong shape to LOOK at. A box hides
    the two things you actually want to see around a servo - that the horn end
    is not the boss end, and where the connector sticks out - and the whole
    point of this viewer is the fit, not the envelope.

    `order` comes from src/rsbot/model.py's SERVO_MESH, so the mesh here is
    oriented by the same table the sim uses rather than by a second guess.
    """
    if order not in _SERVO_SOLID:
        _SERVO_SOLID[order] = servo.drawing_solid(tuple(order.upper()))
    return _SERVO_SOLID[order]


def solids(mode="wheel"):
    """[(name, solid)] for the whole assembly: parts, real servos, cables."""
    from src.rsbot.model import SERVO_MESH

    out = []
    for name, solid in ac.parts(mode):
        if name.startswith(BOUGHT_PREFIX):
            continue                      # replaced by the real thing below
        out.append((name, solid))

    # The servos, placed by the sim exactly as assemble_check places its boxes,
    # but with the drawing's geometry instead of a block.
    m, d = ac._load_posed(mode) if hasattr(ac, "_load_posed") else (None, None)
    if m is None:
        from fitcheck import pose
        from src.rsbot.model import load
        m, d = load()
        pose(m, d, mode)
        mujoco.mj_forward(m, d)
    for i in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if m.geom_group[i] != 0 or not n.startswith(BOUGHT_PREFIX):
            continue
        order = next(v for k, v in SERVO_MESH.items() if n.startswith(k))
        s = _servo_solid(order)
        if n.endswith("_r") or n.endswith("-1"):
            s = bd.mirror(s, bd.Plane.XZ)
        out.append((n, ac._loc(d.geom_xpos[i], d.geom_xmat[i]) * s))

    # The parallelogram. It was missing from this viewer entirely: 41 solids
    # measured against each other and not one of them the newest mechanism on
    # the robot, so the tool whose whole job is measuring every gap was
    # measuring every gap except this one's. Placed by cad/linkage.py off the
    # same posed model, so it cannot show a linkage in a place the checks did
    # not test.
    import cad.linkage as linkage
    for side in ("l", "r"):
        out.extend(linkage.placed(m, d, side))
        out.extend(linkage.bought(m, d, side))

    # The bought hardware: shafts, bearings, belts. Without these there is
    # literally nothing between the shin and the ankle yoke on screen, because
    # there was nothing in the model - both existed only as holes. See
    # cad/hardware.py.
    import cad.hardware as hw
    out.extend(hw.fitted(mode, parts=dict(out)))

    # And the cables, which are the reason this viewer exists at all: a channel
    # is cut out of a part, so on screen it is an absence. You cannot check an
    # absence. Drawing the cable INTO it turns "is there a groove there" into
    # "does the cable sit in the groove", which is the question.
    for label, a, b, _ in wiring.runs(mode):
        out.append((f"cable {label}", wiring.tube(a, b)))
    return out


def export(mode="wheel", rebuild=False):
    """One STL per part, already in its assembled world position.

    Posed rather than local, deliberately. The alternative is to export each
    part once and send a transform, which means re-deriving the sim's placement
    in JavaScript - a second implementation of the one thing this project keeps
    getting wrong. Exported posed, the browser cannot disagree with the checks.
    """
    OUT.mkdir(parents=True, exist_ok=True)
    meta = []
    for name, solid in solids(mode):
        safe = name.replace(" ", "_").replace(">", "to").replace("-", "_")
        path = OUT / f"fit_{mode}_{safe}.stl"
        if rebuild or not path.exists():
            bd.export_stl(solid, str(path), tolerance=0.05,
                          angular_tolerance=0.2)
        b = solid.bounding_box()
        meta.append(dict(
            name=name, kind=_kind(name), file=path.name,
            centre=[(b.min.X + b.max.X) / 2, (b.min.Y + b.max.Y) / 2,
                    (b.min.Z + b.max.Z) / 2],
            volume=round(solid.volume / 1000.0, 2)))
    return meta


def gaps(mode="wheel"):
    """Every pair's minimum distance, tightest first.

    All pairs, not just the ones a check cares about. `assemble_check` only
    measures running pairs, because those are the ones that can rub; here the
    question is different and softer - "show me what is close" - and a 0.3 mm
    static gap is worth SEEING even where it is legal.

    Touching pairs are reported as touching rather than filtered out. Sixteen
    faces in this robot are meant to be in contact and one of the ways this
    design has gone wrong before is a face that stopped touching and nobody
    noticed, so they are the first thing worth being able to count.
    """
    items = [(n, s) for n, s in solids(mode) if not n.startswith("cable")]
    rows = []
    for (na, a), (nb, b) in itertools.combinations(items, 2):
        try:
            d = ac.gap(a, b)
        except Exception:
            continue
        if d != d or d > 6.0:            # nan, or too far apart to be a fit
            continue
        rows.append(dict(a=na, b=nb, mm=round(float(d), 3),
                         running=bool(ac.running_pair(na, nb))))
    rows.sort(key=lambda r: r["mm"])
    return rows


# --- driving it -----------------------------------------------------------
#
# A still picture cannot answer "what does the belt do". The ankle-pitch servo
# is not on its joint - the wheel owns that axle - so it drives through a 2:1
# GT2 reduction, and the only way to see that is to turn it.
#
# The FLIP is ankle ROLL, so it is the wrong manoeuvre to watch for this: the
# belt barely moves through it. This sweeps ankle PITCH, which is the joint the
# belt actually drives.
#
# Transforms are computed HERE and applied in the browser. The browser never
# works out where anything goes - it multiplies a matrix this file handed it -
# so the animation cannot drift from the checks the way a second placement
# implementation would.

# Which body each solid rides on. Hardware is parented to the link it is bolted
# to, so it moves with it for free.
def _parent(name):
    if name.startswith("cable"):
        return None                       # they stretch; not rigid
    for side in ("l", "r"):
        if name.endswith(f"_{side}") or name.endswith(f" {side}"):
            if name.startswith(("shaft ankle", "bearing ankle")):
                return f"ankle_{side}"
            if name.startswith("pulley 40T"):
                # ON THE ANKLE SHAFT, so it turns with the ankle, not the shin.
                # Parented to the shin it sat perfectly still through the whole
                # sweep, which is the opposite of what this animation is for.
                return f"ankle_{side}"
            if name.startswith(("belt", "pulley")):
                return f"shin_{side}"
    if name.startswith("horn "):
        return name.split()[1]            # resolved to the servo's body below
    return name


_SHIN_AXIS = {}


def frames(n=25):
    """[{part: 4x4 column-major}] sweeping ankle pitch, relative to frame 0."""
    from fitcheck import pose
    from src.rsbot.model import load

    m, d = load()
    pose(m, d, "wheel")
    mujoco.mj_forward(m, d)

    jids, adrs = [], []
    for side in ("l", "r"):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"ankle_pitch_{side}")
        if j >= 0:
            jids.append(j)
            adrs.append(m.jnt_qposadr[j])
    lo, hi = -0.6, 0.6                     # rad, well inside the joint's range

    # Which body every solid belongs to, servos included.
    owner = {}
    for i in range(m.ngeom):
        gn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if gn.startswith(BOUGHT_PREFIX):
            owner[gn] = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY,
                                          m.geom_bodyid[i])

    for side in ("l", "r"):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"shin_{side}")
        if bid >= 0:
            _SHIN_AXIS[side] = (d.xmat[bid].reshape(3, 3) @ np.array([0.0, 1.0, 0.0]))

    def body_T(bid):
        T = np.eye(4)
        T[:3, :3] = d.xmat[bid].reshape(3, 3)
        T[:3, 3] = d.xpos[bid] * 1000.0
        return T

    names = [nm for nm, _ in _cached("wheel")["parts"]] if False else None
    out, base = [], {}
    for k in range(n):
        ang = lo + (hi - lo) * k / (n - 1)
        for a in adrs:
            d.qpos[a] = ang
        mujoco.mj_forward(m, d)
        fr = {}
        for bname in ("torso", "thigh_l", "thigh_r", "shin_l", "shin_r",
                      "ankle_l", "ankle_r", "rollbracket_l", "rollbracket_r",
                      "wheel_l", "wheel_r"):
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bname)
            if bid >= 0:
                fr[bname] = body_T(bid)
        if k == 0:
            base = {b: np.linalg.inv(T) for b, T in fr.items()}
        out.append({b: (T @ base[b]) for b, T in fr.items()})
    return out, owner, (lo, hi)


def _spin(axis, centre, ang):
    """A world 4x4 that turns `ang` about `axis` through `centre`."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(centre, float) - R @ np.asarray(centre, float)
    return T


def frame_payload(n=25):
    mats, owner, rng = frames(n)
    meta = _cached("wheel")["parts"]
    angles = [rng[0] + (rng[1] - rng[0]) * k / (n - 1) for k in range(n)]

    # The 20T sits on the servo horn and turns RATIO times for every turn of
    # the joint, which is the whole point of the drive and the one thing a
    # still picture cannot show. Its axis is the shin's, through its own centre.
    import cad.belt as _belt
    spin_axis, spin_centre = {}, {}
    for pm in meta:
        if pm["name"].startswith("pulley 20T"):
            spin_centre[pm["name"]] = pm["centre"]

    seq = []
    for k, fr in enumerate(mats):
        one = {}
        dth = angles[k] - angles[0]
        for pmeta in meta:
            nm = pmeta["name"]
            par = _parent(nm)
            if par in owner:
                par = owner[par]
            if par not in fr:
                continue
            T = fr[par]
            if nm in spin_centre:
                side = nm.strip()[-1]
                bid = 0
                # the shin's y axis in world at frame 0 is the pulley's axis
                ax = _SHIN_AXIS.get(side)
                if ax is not None:
                    T = T @ _spin(ax, spin_centre[nm], -_belt.RATIO * dth)
            one[nm] = [float(v) for v in T.T.flatten()]
        seq.append(one)
    return dict(frames=seq, lo=rng[0], hi=rng[1])


def scene(mode="wheel", rebuild=False):
    meta = export(mode, rebuild)
    g = gaps(mode)
    # The cable runs, with how much solid material each one passes through.
    # Anything but zero is a channel that does not go where the cable does.
    cab = [dict(label=r["label"], mm=round(r["length"], 1),
                blocked=round(r["blocked"], 1),
                through=sorted(r["by"], key=lambda k: -r["by"][k])[:3])
           for r in wiring.check(mode, verbose=False)]
    # What is held by nothing. The viewer is where this belongs: a list of
    # names in a terminal makes you hunt for the part, and the whole reason
    # these survived is that nobody could see them.
    import cad.floating as floating
    fl = floating.floaters(mode, dict(solids(mode)))
    c = np.mean([m["centre"] for m in meta], axis=0)
    return dict(mode=mode, parts=meta, gaps=g, cables=cab, floating=fl,
                centre=[float(v) for v in c],
                floats=len(fl),
                touching=sum(1 for r in g if r["mm"] <= 1e-6),
                blocked=sum(1 for r in cab if r["blocked"] > 1.0),
                tight=[r for r in g if 0 < r["mm"] < 0.8])


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>rs-bot assembly</title>
<style>
 :root{--bg:#15171b;--panel:#1d2026;--line:#2b2f37;--ink:#d7dae0;--dim:#868d99;
       --hot:#e0603a;--ok:#54c98a;--warn:#e0b23a}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--ink);
      font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;overflow:hidden}
 #c{position:fixed;inset:0}
 #panel{position:fixed;top:0;right:0;width:330px;height:100%;background:var(--panel);
        border-left:1px solid var(--line);overflow-y:auto;padding:14px}
 h1{font-size:13px;margin:0 0 2px;letter-spacing:.08em;text-transform:uppercase}
 .sub{color:var(--dim);margin-bottom:14px}
 h2{font-size:11px;color:var(--dim);letter-spacing:.1em;text-transform:uppercase;
    margin:18px 0 6px;border-bottom:1px solid var(--line);padding-bottom:4px}
 .row{display:flex;justify-content:space-between;gap:8px;padding:3px 4px;
      border-radius:3px;cursor:pointer}
 .row:hover{background:#262a32}
 .row.off{opacity:.35}
 .sw{width:9px;height:9px;border-radius:2px;flex:0 0 auto;margin-top:5px}
 .nm{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
 .v{color:var(--dim);font-variant-numeric:tabular-nums}
 .g{color:var(--ok)} .g.tight{color:var(--warn)} .g.touch{color:var(--hot)}
 .sel{background:#333842!important}
 label{display:block;margin:10px 0 4px;color:var(--dim)}
 input[type=range]{width:100%}
 button{background:#262a32;color:var(--ink);border:1px solid var(--line);
        border-radius:3px;padding:4px 9px;cursor:pointer;font:inherit}
 button.on{background:var(--hot);border-color:var(--hot);color:#fff}
 .note{color:var(--dim);margin-top:6px}
</style></head><body>
<div id="c"></div>
<div id="panel">
  <h1>rs-bot assembly</h1>
  <div class="sub" id="sub">loading…</div>
  <div><button id="mWheel">wheel mode</button> <button id="mFoot">foot mode</button></div>
  <label>explode <span id="expv" class="v">0 mm</span></label>
  <input type="range" id="exp" min="0" max="120" value="0">
  <label>drive the ankle pitch <span id="drvv" class="v">off</span></label>
  <input type="range" id="drv" min="0" max="24" value="12" disabled>
  <div><button id="play">play the belt drive</button></div>
  <div class="note">the 20T on the servo horn turns TWICE for every turn of
    the 40T on the joint. Watch the notch on each pulley rim.</div>
  <div><button id="reset">reset view</button> <button id="showall">show all</button></div>
  <h2>held by nothing</h2>
  <div class="note">shown in RED. A part whose own rigid body it does not
    touch: it moves as one lump with that body and nothing joins it to it.
    Click a row to isolate the pair.</div>
  <div id="floating"></div>
  <h2>parts</h2><div id="parts"></div>
  <h2>cable runs</h2>
  <div class="note">bright green tubes. "through" means the run passes
    through solid material, so the channel is in the wrong place.</div>
  <div id="cables"></div>
  <h2>closest pairs</h2>
  <div class="note">click a row to isolate that pair</div>
  <div id="gaps"></div>
</div>
<script type="importmap">{"imports":{
 "three":"https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js",
 "three/addons/":"https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/"}}
</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

const qs = new URLSearchParams(location.search);
let mode = qs.get('mode') || 'wheel';
const S = await (await fetch('/scene?mode=' + mode)).json();

const el = document.getElementById('c');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x15171b);
const cam = new THREE.PerspectiveCamera(38, 1, 1, 5000);
const ren = new THREE.WebGLRenderer({antialias:true});
ren.setPixelRatio(devicePixelRatio); el.appendChild(ren.domElement);
const ctr = new OrbitControls(cam, ren.domElement);

// Z is up in the CAD frame; three.js assumes Y. Rotate the whole scene once
// rather than every mesh, so the numbers in the panel stay in CAD coordinates.
const root = new THREE.Group();
root.rotation.x = -Math.PI/2;
scene.add(root);

scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x30343c, 1.5));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(300, 500, 400); scene.add(key);
const fill = new THREE.DirectionalLight(0xffffff, 0.5);
fill.position.set(-400, -200, 150); scene.add(fill);

// Colour by MATERIAL, with roughness and metalness to match. PA6-CF is a
// matte dark brown-black in life, not orange, but a robot rendered entirely in
// black reads as a silhouette - so the printed parts keep a warm tint and
// everything bought is rendered as what it actually is.
const MAT = {
  printed: {c:0xc4643a, m:0.02, r:0.85},   // PA6-CF, matte
  servo:   {c:0x23262b, m:0.25, r:0.55},   // servo case, dark engineering plastic
  belt:    {c:0x111316, m:0.05, r:0.95},   // rubber, glass-corded
  tyre:    {c:0x17181b, m:0.02, r:0.98},   // TPU tread
  alu:     {c:0x9fa6ae, m:0.85, r:0.32},   // anodised aluminium pulley
  steel:   {c:0xd7dbe0, m:0.95, r:0.18},   // chrome steel shaft and races
  horn:    {c:0xb9bec6, m:0.90, r:0.28},   // plated steel horn
  cable:   {c:0x2f8f5b, m:0.10, r:0.70},   // PVC sleeve
};
const COL = Object.fromEntries(Object.entries(MAT).map(([k,v])=>[k,v.c]));
const C = new THREE.Vector3(...S.centre);
const meshes = {};
const loader = new STLLoader();

function add(p){
  return new Promise(res => loader.load('/mesh/' + p.file, geo => {
    const spec = MAT[p.kind] || MAT.printed;
    const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
      color: spec.c, metalness: spec.m, roughness: spec.r,
      transparent:true, opacity:1}));
    m.userData = p;
    const c = new THREE.Vector3(...p.centre);
    m.userData.dir = c.clone().sub(C).normalize();
    root.add(m); meshes[p.name] = m; res();
  }));
}
await Promise.all(S.parts.map(add));

// The floating parts go red and stay red. Not a hover state or a filter you
// have to switch on: if you can turn the warning off you will, and then the
// viewer is back to showing you a robot that looks assembled.
const FLOAT = new Set(S.floating.map(f => f.name));
for(const n of FLOAT){
  const m = meshes[n];
  if(!m) continue;
  m.material.color.setHex(0xe0603a);
  m.material.emissive.setHex(0x501a0c);
}

// Frame the robot once everything is in.
const box = new THREE.Box3().setFromObject(root);
const size = box.getSize(new THREE.Vector3()).length();
const mid = box.getCenter(new THREE.Vector3());
function reset(){
  ctr.target.copy(mid);
  cam.position.set(mid.x + size*0.80, mid.y + size*0.55, mid.z + size*0.80);
  cam.near = size/500; cam.far = size*12; cam.updateProjectionMatrix();
  ctr.update();
}
function resize(){
  const w = innerWidth-330, h = innerHeight;
  cam.aspect = w/h; cam.updateProjectionMatrix(); ren.setSize(w,h);
}
addEventListener('resize', resize); resize(); reset();
(function loop(){ requestAnimationFrame(loop); ctr.update(); ren.render(scene,cam); })();

// --- panel -------------------------------------------------------------------
document.getElementById('sub').textContent =
  S.parts.length + ' solids · ' + S.touching + ' faces touching · ' +
  S.tight.length + ' pair(s) under 0.8 mm · ' + S.cables.length + ' cable runs, ' +
  S.blocked + ' blocked';
document.getElementById('m' + (mode==='wheel'?'Wheel':'Foot')).classList.add('on');
document.getElementById('mWheel').onclick = ()=> location.search='?mode=wheel';
document.getElementById('mFoot').onclick  = ()=> location.search='?mode=foot';
document.getElementById('reset').onclick = reset;

const hidden = new Set();
function paint(){
  for(const [n,m] of Object.entries(meshes)) m.visible = !hidden.has(n);
  document.querySelectorAll('#parts .row').forEach(r =>
    r.classList.toggle('off', hidden.has(r.dataset.n)));
}
const pl = document.getElementById('parts');
for(const p of S.parts){
  const d = document.createElement('div');
  d.className='row'; d.dataset.n = p.name;
  d.innerHTML = '<div class="sw" style="background:#' +
    COL[p.kind].toString(16).padStart(6,'0') + '"></div>' +
    '<div class="nm">' + p.name + '</div>' +
    '<div class="v">' + p.volume.toFixed(1) + ' cm³</div>';
  d.onclick = () => { hidden.has(p.name) ? hidden.delete(p.name)
                                         : hidden.add(p.name); paint(); };
  pl.appendChild(d);
}
document.getElementById('showall').onclick = ()=>{ hidden.clear(); paint(); };

// Held by nothing. Clicking isolates the part and the thing it should be
// touching, because "vhipsv1 floats off torso" is only useful once you can see
// which face was supposed to meet which.
const fp = document.getElementById('floating');
if(!S.floating.length){
  fp.innerHTML = '<div class="row"><div class="nm g">nothing floats</div></div>';
}
for(const f of S.floating){
  const d = document.createElement('div');
  d.className = 'row';
  d.innerHTML = '<div class="sw" style="background:#e0603a"></div>' +
    '<div class="nm">' + f.name + '</div>' +
    '<div class="v">' + (f.mm === null ? f.off : f.mm.toFixed(3) + ' mm') + '</div>';
  d.title = f.name + ' -> ' + f.off;
  d.onclick = () => {
    hidden.clear();
    for(const p of S.parts) if(p.name !== f.name && p.name !== f.off) hidden.add(p.name);
    paint();
  };
  fp.appendChild(d);
}

// Cables. Clicking one shows just that run plus the parts it passes through,
// which is the view you want when a channel is in the wrong place.
const cl = document.getElementById('cables');
let csel = null;
for(const c of S.cables){
  const d = document.createElement('div');
  d.className='row';
  const bad = c.blocked > 1.0;
  d.innerHTML = '<div class="nm">' + c.label.replace(/v|sv/g,'') + '</div>' +
    '<div class="g ' + (bad?'touch':'') + '">' +
    (bad ? c.blocked.toFixed(0) + ' mm³' : c.mm.toFixed(0) + ' mm') + '</div>';
  d.title = bad ? 'passes through ' + c.through.join(', ') : 'clear';
  d.onclick = () => {
    if(csel === d){ csel.classList.remove('sel'); csel=null; hidden.clear(); }
    else {
      if(csel) csel.classList.remove('sel');
      csel = d; d.classList.add('sel');
      hidden.clear();
      const keep = new Set(['cable ' + c.label, ...c.through]);
      for(const p of S.parts) if(!keep.has(p.name)) hidden.add(p.name);
    }
    paint();
  };
  cl.appendChild(d);
}

const gl = document.getElementById('gaps');
let sel = null;
for(const g of S.gaps.slice(0, 40)){
  const d = document.createElement('div');
  d.className='row';
  const cls = g.mm <= 1e-6 ? 'touch' : (g.mm < 0.8 ? 'tight' : '');
  d.innerHTML = '<div class="nm">' + g.a + ' × ' + g.b + '</div>' +
    '<div class="g ' + cls + '">' +
    (g.mm <= 1e-6 ? 'touching' : g.mm.toFixed(2) + ' mm') + '</div>';
  d.onclick = () => {
    if(sel === d){ sel.classList.remove('sel'); sel=null; hidden.clear(); }
    else {
      if(sel) sel.classList.remove('sel');
      sel = d; d.classList.add('sel');
      hidden.clear();
      for(const p of S.parts) if(p.name!==g.a && p.name!==g.b) hidden.add(p.name);
    }
    paint();
  };
  gl.appendChild(d);
}
paint();

// --- the drive ------------------------------------------------------------
let FR = null, playing = false, fi = 12;
const drv = document.getElementById('drv'), drvv = document.getElementById('drvv');
const M4 = new THREE.Matrix4();
function applyFrame(i){
  if(!FR) return;
  const f = FR.frames[i];
  for(const [n, mesh] of Object.entries(meshes)){
    const a = f[n];
    if(!a){ continue; }
    mesh.matrixAutoUpdate = false;
    mesh.matrix.fromArray(a);
  }
  drvv.textContent = (FR.lo + (FR.hi-FR.lo)*i/(FR.frames.length-1)).toFixed(2) + ' rad';
}
document.getElementById('play').onclick = async () => {
  if(!FR){
    document.getElementById('play').textContent = 'loading frames…';
    FR = await (await fetch('/frames')).json();
    drv.max = FR.frames.length - 1; drv.disabled = false;
  }
  playing = !playing;
  document.getElementById('play').textContent = playing ? 'pause' : 'play the belt drive';
  document.getElementById('play').classList.toggle('on', playing);
  if(playing) document.getElementById('exp').value = 0;
};
drv.oninput = () => { fi = +drv.value; applyFrame(fi); };
let dir = 1, tick = 0;
setInterval(() => {
  if(!playing || !FR) return;
  if(++tick % 2) return;
  fi += dir;
  if(fi >= FR.frames.length-1){ fi = FR.frames.length-1; dir = -1; }
  if(fi <= 0){ fi = 0; dir = 1; }
  drv.value = fi; applyFrame(fi);
}, 60);

const exp = document.getElementById('exp'), expv = document.getElementById('expv');
exp.oninput = () => {
  const k = +exp.value; expv.textContent = k + ' mm';
  playing = false;
  document.getElementById('play').textContent = 'play the belt drive';
  document.getElementById('play').classList.remove('on');
  for(const m of Object.values(meshes)){
    m.matrixAutoUpdate = true;
    m.matrix.identity();
    m.position.copy(m.userData.dir).multiplyScalar(k);
  }
};
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
        elif u.path == "/frames":
            self._send(json.dumps(frame_payload()).encode(),
                       "application/json")
        elif u.path == "/scene":
            q = dict(p.split("=") for p in u.query.split("&") if "=" in p)
            mode = q.get("mode", "wheel")
            if mode not in ("wheel", "foot"):
                mode = "wheel"
            self._send(json.dumps(_cached(mode)).encode(), "application/json")
        elif u.path.startswith("/mesh/"):
            p = OUT / Path(u.path).name
            if not p.exists():
                # Rebuild rather than 404. The scene is cached in memory, so if
                # the STLs are deleted underneath a running server - which is
                # exactly what happens when you re-export after changing a part
                # - every mesh 404s and the page comes up EMPTY with no error
                # anywhere the user can see. A blank viewer reads as "the robot
                # is broken", and it took a round trip to find out it was not.
                name = p.name
                if name.startswith("fit_"):
                    mode = name.split("_")[1]
                    if mode in ("wheel", "foot"):
                        print(f"  {name} is missing, rebuilding {mode} ...",
                              flush=True)
                        _SCENES.pop(mode, None)
                        _SCENES[mode] = scene(mode, rebuild=True)
            if p.exists():
                self._send(p.read_bytes(), "model/stl")
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()


_SCENES = {}


def _cached(mode):
    """Built once per mode and kept. Posing and measuring every pair takes
    long enough that doing it per request makes the mode buttons feel broken."""
    if mode not in _SCENES:
        print(f"  building the {mode}-mode scene ...", flush=True)
        _SCENES[mode] = scene(mode)
    return _SCENES[mode]


def main():
    mode = "foot" if "foot" in sys.argv else "wheel"
    s = _cached(mode)
    print("rs-bot assembly viewer")
    print(f"  {len(s['parts'])} solids, {s['touching']} faces touching, "
          f"{len(s['tight'])} pair(s) under 0.8 mm")
    print()
    print("  closest pairs:")
    for r in s["gaps"][:6]:
        d = "touching" if r["mm"] <= 1e-6 else f"{r['mm']:.2f} mm"
        print(f"    {r['a']:<14} x {r['b']:<14} {d}")
    print()
    url = f"http://localhost:{PORT}/?mode={mode}"
    print(f"  {url}")
    threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
