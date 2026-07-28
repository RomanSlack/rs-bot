"""Softening edges, without the failure mode that makes it pointless.

    from cad.shape import soften

OCC refuses a fillet fairly often on parts this busy, and it refuses by
throwing. The tempting thing is to wrap it in try/except and carry on, and
docs/stress.md records what that costs:

    OCC refuses the fillet if the edge selection is widened, and swallowing
    that exception returns the SHARP part while looking like a small
    improvement.

A sharp re-entrant corner is a stress singularity - refine the mesh and the
peak grows without limit, because the exact solution there is infinite. So a
fillet that silently did not happen is not a cosmetic miss, it is the one place
the part will crack, still present and now believed to be fixed.

`soften` therefore reports rather than hides: it returns the radius it actually
achieved, and raises if it achieved nothing at all.
"""

import build123d as bd


def long_edges(part, axis="z", length=50.0, at=None, tol=0.6):
    """Edges running mostly along one axis, optionally near given corners.

    `at` is a list of (u, v) positions in the other two axes, which is how you
    pick "the four outside corners" without naming faces.
    """
    i = "xyz".index(axis)
    others = [j for j in range(3) if j != i]
    out = []
    for e in part.edges():
        bb = e.bounding_box()
        size = [bb.size.X, bb.size.Y, bb.size.Z]
        lo = [bb.min.X, bb.min.Y, bb.min.Z]
        hi = [bb.max.X, bb.max.Y, bb.max.Z]
        if size[i] < length:
            continue
        if any(size[j] > 0.2 for j in others):
            continue
        if at is None:
            out.append(e)
            continue
        c = [(lo[j] + hi[j]) / 2 for j in others]
        if any(abs(c[0] - u) < tol and abs(c[1] - v) < tol for u, v in at):
            out.append(e)
    return out


def soften(part, edges, radii=(2.5, 2.0, 1.5, 1.0), what="edges"):
    """Fillet `edges` at the largest radius OCC will accept.

    Returns (part, radius). Raises if every radius fails, because a silently
    sharp corner is worse than no fillet: it looks done.
    """
    if not edges:
        raise ValueError(f"no {what} matched - the selection is wrong, and an "
                         f"empty selection fillets nothing while reporting "
                         f"success")
    last = None
    for r in radii:
        try:
            out = bd.fillet(edges, r)
        except Exception as e:          # noqa: BLE001 - OCC raises broadly
            last = e
            continue
        if len(out.solids()) != 1:
            last = RuntimeError(f"fillet at {r} split the part into "
                                f"{len(out.solids())} solids")
            continue
        return out, r
    raise RuntimeError(f"could not fillet {len(edges)} {what} at any of "
                       f"{radii}: {last}")
