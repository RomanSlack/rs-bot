"""What is holding each part up?   uv run python -m cad.floating

Every check in this repo so far asks a GLOBAL question - is the robot one
connected object - and answers it with a union-find over every part at once.
That answer is one number, it is almost always "yes", and it stays "yes" while
a servo hovers half a millimetre off the plate it bolts to, because the servo
is still joined to the robot through its own output horn.

This asks the LOCAL question instead, and there are two of them, because parts
are held two different ways:

  A RIGID BODY has to be one piece.       Everything the sim treats as one body
                                          moves as one lump, so if its solids do
                                          not touch each other, the lump is held
                                          together by nothing. `loose()`.

  A CONNECTOR has to reach both ends.     A shaft, a bearing, a pin, a horn: it
                                          exists to bridge two bodies, so
                                          touching one of them is a failure.
                                          `unreached()`.

The distinction matters because the two have OPPOSITE tells. A bracket that
touches only one body is fine. A shaft that touches only one body is a shaft
that is not driving anything.

WHAT IT FOUND ON THE FIRST RUN. The hip servos sit 0.400 mm off the chassis -
not in a cradle with clearance around it and bolted through its face, just off
it, with nothing between the case and the plate anywhere. The load path from
the torso into the legs goes through two M3 screws in air. cad/assemble_check.py
has caught this exact fault once before, on the wheel-drive servo, and the note
it left says so: "0.5 mm of air reads like 20 mm of air". Nothing generalised
that catch into a check, so the next servo to do it did it silently.

The number to remember is that CRADLE_CLEAR is 0.4. A cradle is cut 0.4 mm
oversize on purpose so the servo drops in; that is correct and it is why the
gap is not a mistake in the cradle. It is a mistake in there being no seating
FACE - a cradle you can drop a servo into is not a cradle that locates it.

VIEWING IT. `cad/fitview.py` colours whatever this reports, so the answer is
not a list of names you then have to find on screen. Run the viewer and the
floating parts are the red ones.
"""

import itertools
from collections import defaultdict

import mujoco

# A seam. Two printed faces that meet are modelled as coincident, so anything
# above float noise is a real gap. NOT the 4 mm the robot-wide connectivity
# check uses: that is a running clearance across a joint, and inside a rigid
# part it is 4 mm of air.
SEAM = 0.05

# Parts that bridge bodies for a living. Everything else is structure.
CONNECTOR = ("shaft ", "bearing ", "horn ", "belt ", "pulley ", "lkpin", "lkbrg")

# Not structure at all. A cable is flexible and is supposed to span a gap.
IGNORE = ("cable ",)


def _body_of(name):
    """Which rigid body a part rides on, or None if it is a connector.

    Written out rather than read off the sim because most of these are not sim
    geoms: the real servo meshes, the hardware and the linkage are all placed
    ONTO bodies by other modules, and the body each one belongs to is a fact
    about the assembly rather than about the model file.
    """
    if name.startswith(IGNORE) or name.startswith(CONNECTOR):
        return None
    side = name[-2:] if name[-2:] in ("_l", "_r") else ""
    if name.startswith("vhipsv"):
        return "torso"
    if name.startswith("vkneesv"):
        return "thigh" + side
    if name.startswith("vrollsv"):
        return "ankle" + side
    if name.startswith("vwhlsv"):
        return "rollbracket" + side
    for stem, body in (("lk_rod1", "lkrod1"), ("lk_rod2", "lkrod2"),
                       ("lk_idler", "lkidler")):
        if name.startswith(stem):
            return body + side
    return name


_BB = {}


def _bbgap(a, b):
    """Axis-aligned box separation: a LOWER bound on the true distance.

    Only ever used to skip work. If two boxes are already further apart than a
    seam, the solids inside them cannot be touching, and the exact query - which
    is the expensive one, and there are about 1600 of them - is not run. Never
    used to decide that something DOES touch, because a box gap of zero says
    nothing at all.
    """
    d = 0.0
    for lo_a, hi_a, lo_b, hi_b in ((a.min.X, a.max.X, b.min.X, b.max.X),
                                   (a.min.Y, a.max.Y, b.min.Y, b.max.Y),
                                   (a.min.Z, a.max.Z, b.min.Z, b.max.Z)):
        s = max(lo_a - hi_b, lo_b - hi_a, 0.0)
        d += s * s
    return d ** 0.5


def _gap(a, b, cheap=False):
    """Exact separation. `cheap` returns early once a bounding-box lower bound
    already exceeds the seam, which is the answer either way at that point."""
    if cheap:
        for s in (a, b):
            if id(s) not in _BB:
                _BB[id(s)] = s.bounding_box()
        if _bbgap(_BB[id(a)], _BB[id(b)]) > SEAM:
            return _bbgap(_BB[id(a)], _BB[id(b)])
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    q = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    q.Perform()
    return q.Value()


def _scene(mode):
    """{name: solid} for the whole assembly, from the viewer's own set.

    fitview.solids() rather than assemble_check.parts(), on purpose: it is the
    superset - real servo meshes instead of boxes, plus the hardware and the
    linkage - and it is what is on screen. A tool that reports floating parts
    the viewer does not draw is a tool nobody can act on.
    """
    import cad.fitview as fitview
    return dict(fitview.solids(mode))


def loose(mode="wheel", scene=None, verbose=False):
    """[(body, [group, ...])] for every body whose solids are not one piece.

    Union-find WITHIN a body. Across bodies is a different question and a
    joint's running clearance is not a fault, which is why the robot-wide
    version of this cannot see any of it.
    """
    S = scene or _scene(mode)
    byb = defaultdict(list)
    for n in S:
        b = _body_of(n)
        if b is not None:
            byb[b].append(n)

    out = []
    for b, names in sorted(byb.items()):
        if len(names) < 2:
            continue
        parent = {n: n for n in names}

        def find(n):
            while parent[n] != n:
                parent[n] = parent[parent[n]]
                n = parent[n]
            return n

        for x, y in itertools.combinations(names, 2):
            if _gap(S[x], S[y], cheap=True) <= SEAM:
                parent[find(x)] = find(y)
        groups = defaultdict(list)
        for n in names:
            groups[find(n)].append(n)
        if len(groups) > 1:
            out.append((b, [sorted(g) for g in groups.values()]))
    return out


def unreached(mode="wheel", scene=None):
    """[(connector, [bodies it touches])] for connectors that reach fewer than
    two bodies.

    A shaft that touches one body is not transmitting anything. This is the
    half of the question that `loose()` cannot ask, because a connector has no
    body of its own to be loose within.

    REACH IS TRANSITIVE THROUGH OTHER CONNECTORS, and the first version of this
    was not. It reported all four ankle shafts as reaching nothing, because a
    shaft does not touch the parts it joins - it touches the BEARING that is
    pressed into them. Four names, every one of them correctly assembled, which
    is the fastest way to teach somebody to ignore a check.
    """
    S = scene or _scene(mode)
    conn = [n for n in S if n.startswith(CONNECTOR)]
    body = {n: _body_of(n) for n in S}

    # Connectors that touch each other are one linkage in the load-path sense:
    # a shaft in a bearing in a yoke is holding the yoke.
    parent = {c: c for c in conn}

    def find(c):
        while parent[c] != c:
            parent[c] = parent[parent[c]]
            c = parent[c]
        return c

    for x, y in itertools.combinations(conn, 2):
        if _gap(S[x], S[y], cheap=True) <= SEAM:
            parent[find(x)] = find(y)

    reach = defaultdict(set)
    for c in conn:
        for n in S:
            if body[n] is not None and _gap(S[c], S[n], cheap=True) <= SEAM:
                reach[find(c)].add(body[n])

    out = []
    for c in conn:
        touched = reach[find(c)]
        if len(touched) < 2:
            out.append((c, sorted(touched)))
    return out


def nearest(name, mode="wheel", scene=None, n=3):
    """The n closest other parts and their gaps. What you want the moment
    something is reported loose: not that it floats, but off WHAT."""
    S = scene or _scene(mode)
    d = sorted(((_gap(S[name], S[o]), o) for o in S if o != name))[:n]
    return [(o, round(g, 3)) for g, o in d]


def floaters(mode="wheel", scene=None):
    """[{name, body, off, mm}] flattened for a viewer.

    The BIGGEST group in a body is taken to be the structure and everything
    else is floating off it. That is a heuristic and it is the right one here:
    a body is a printed part plus the things bolted to it, so the part is
    always the bulk. It would be the wrong call for a body that is genuinely
    two halves, and there is no such body - `loose()` still reports the raw
    groups if you want to check that yourself.
    """
    S = scene or _scene(mode)
    out = []
    for body, groups in loose(mode, S):
        host = max(groups, key=len)
        for g in groups:
            if g is host:
                continue
            for n in g:
                near = min(((_gap(S[n], S[h]), h) for h in host))
                out.append(dict(name=n, body=body, off=near[1],
                                mm=round(near[0], 3)))
    for c, touched in unreached(mode, S):
        out.append(dict(name=c, body="(connector)",
                        off=" ".join(touched) or "nothing",
                        mm=None))
    return out


def report(verbose=True):
    bad = 0
    for mode in ("wheel", "foot"):
        S = _scene(mode)
        lo, un = loose(mode, S), unreached(mode, S)
        bad += len(lo) + len(un)
        if not verbose:
            continue
        print(f"--- {mode} mode: {len(S)} solids")
        if not lo:
            print("   every body is one piece")
        for b, groups in lo:
            print(f"   {b} is {len(groups)} loose pieces:")
            for g in groups:
                print(f"      {' '.join(g)}")
                for o, mm in nearest(g[0], mode, S):
                    print(f"         {g[0]} -> {o:<26} {mm:7.3f} mm")
        if not un:
            print("   every connector reaches two bodies")
        for c, touched in un:
            print(f"   {c} reaches {len(touched)}: {' '.join(touched) or 'nothing'}")
            for o, mm in nearest(c, mode, S):
                print(f"         -> {o:<26} {mm:7.3f} mm")
        print()
    print("nothing floats" if bad == 0 else f"{bad} thing(s) held by nothing")
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if report() else 0)
