"""Linear-elastic FEA: mesh a STEP file, load it, solve it.

    uv run python -m cad.fea          # runs the verification case

Ten-node tetrahedra, not four. That is not a detail. A four-node tet has a
constant strain field, so it cannot represent the linear strain gradient in a
bending beam at all; it locks, and under-reports peak stress by a factor that
grows with slenderness. Every part in this robot is a slender member in
bending, so TET4 would have produced a confident and badly non-conservative
answer. TET10 integrates bending exactly for a straight-sided element.

Units are mm, N, MPa throughout, which is self-consistent: stresses come out in
MPa and displacements in mm with no conversion anywhere.

Two things to distrust in any result this produces:

  - Stress at a fixed boundary is singular. The mesh does not converge there;
    refining it just makes the number bigger. `report()` gives both the raw
    peak and a percentile away from the constrained nodes, and the second is
    the one to use.
  - This is isotropic. A printed part is not: it is much weaker across the
    layers than along them. `interlayer()` pulls out the normal stress on the
    layer plane so that can be checked against the right allowable.

Verified against a cantilever with a known closed-form answer before being
used on anything - see `verify()`.
"""

from cad import THREADS

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spl

# Gmsh's 10-node tet: corners 0-3, then mid-edge nodes on these pairs.
EDGES = [(0, 1), (1, 2), (0, 2), (0, 3), (2, 3), (1, 3)]

# 4-point rule, exact to degree 2. B is linear in a straight-sided tet, so
# B.T D B is quadratic and this integrates the stiffness exactly.
_A, _B = 0.5854101966249685, 0.1381966011250105
GAUSS = np.array([[_A, _B, _B], [_B, _A, _B], [_B, _B, _A], [_B, _B, _B]])
GW = np.full(4, 1.0 / 24.0)


def shape_derivs(rst):
    """dN/d(r,s,t) at parametric points. (P, 10, 3)."""
    r, s, t = rst[:, 0], rst[:, 1], rst[:, 2]
    L = np.stack([1 - r - s - t, r, s, t], axis=1)          # (P, 4)
    dL = np.array([[-1.0, -1, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    dN = np.zeros((len(rst), 10, 3))
    for i in range(4):
        dN[:, i, :] = (4 * L[:, i] - 1)[:, None] * dL[i]
    for k, (a, b) in enumerate(EDGES):
        dN[:, 4 + k, :] = 4 * (L[:, a, None] * dL[b] + L[:, b, None] * dL[a])
    return dN


def _elastic(E, nu):
    lam = E * nu / ((1 + nu) * (1 - 2 * nu))
    mu = E / (2 * (1 + nu))
    D = np.zeros((6, 6))
    D[:3, :3] = lam
    D[0, 0] = D[1, 1] = D[2, 2] = lam + 2 * mu
    D[3, 3] = D[4, 4] = D[5, 5] = mu
    return D


def _B_matrices(dNdx):
    """Strain-displacement, from dN/dx (..., 10, 3) -> (..., 6, 30)."""
    sh = dNdx.shape[:-2]
    B = np.zeros(sh + (6, 30))
    for a in range(10):
        gx, gy, gz = dNdx[..., a, 0], dNdx[..., a, 1], dNdx[..., a, 2]
        c = 3 * a
        B[..., 0, c + 0] = gx
        B[..., 1, c + 1] = gy
        B[..., 2, c + 2] = gz
        B[..., 3, c + 0] = gy
        B[..., 3, c + 1] = gx
        B[..., 4, c + 1] = gz
        B[..., 4, c + 2] = gy
        B[..., 5, c + 0] = gz
        B[..., 5, c + 2] = gx
    return B


def _grads(nodes, elems, rst):
    """dN/dx and detJ at parametric points, for every element."""
    dN = shape_derivs(rst)                                  # (P, 10, 3)
    xe = nodes[elems]                                       # (E, 10, 3)
    J = np.einsum("eai,paj->epij", xe, dN)                  # (E, P, 3, 3)
    detJ = np.linalg.det(J)
    dNdx = np.einsum("pai,epij->epaj", dN, np.linalg.inv(J))
    return dNdx, detJ


def stiffness(nodes, elems, E, nu):
    dNdx, detJ = _grads(nodes, elems, GAUSS)
    B = _B_matrices(dNdx)                                   # (E, P, 6, 30)
    D = _elastic(E, nu)
    w = GW[None, :] * detJ                                  # (E, P)
    Ke = np.einsum("epia,ij,epjb,ep->eab", B, D, B, w)      # (E, 30, 30)

    dof = (elems[:, :, None] * 3 + np.arange(3)).reshape(len(elems), 30)
    rows = np.repeat(dof, 30, axis=1).ravel()
    cols = np.tile(dof, (1, 30)).ravel()
    n = len(nodes) * 3
    return sp.csr_matrix((Ke.ravel(), (rows, cols)), shape=(n, n))


def distribute(nodes, node_ids, force, moment):
    """Nodal forces on `node_ids` that sum to `force` and, about their
    centroid, to `moment`, with the least total nodal force.

    Writing f_i = a + b x r_i and requiring both sums gives a = F/n and
    J b = M with J = sum(|r|^2 I - r r^T). Bolting the load to a single node
    instead would put a spike there and swamp the real stress field.
    """
    r = nodes[node_ids] - nodes[node_ids].mean(axis=0)
    J = np.sum(np.einsum("ij,ij->i", r, r)[:, None, None] * np.eye(3)
               - np.einsum("ai,aj->aij", r, r), axis=0)
    b = np.linalg.pinv(J) @ moment
    return force / len(node_ids) + np.cross(b, r)



def _residual(K, u, f):
    return np.linalg.norm(K @ u - f) / max(np.linalg.norm(f), 1e-12)


def _solve_spd(K, f):
    """Symmetrically scaled direct solve, with an iterative fallback.

    The stiffness matrix is symmetric positive definite, so a direct sparse
    solve should be exact - but the raw matrix spans many orders of magnitude
    on its diagonal (a 2 mm rib and a 20 mm spine in the same system), and
    SuperLU's pivoting then diverges SILENTLY: it returns finite numbers that
    are wrong by a factor of a hundred. That is what made the same geometry and
    load read 22.7 MPa in one material and 132.0 MPa in another.

    Scaling by the square root of the diagonal puts every equation on the same
    footing and the direct solve becomes both correct and fast. The CG fallback
    is kept for genuinely hard cases, but it should almost never run - it takes
    minutes where the scaled direct solve takes seconds.
    """
    d = np.sqrt(np.abs(K.diagonal()))
    d[d == 0] = 1.0
    S = sp.diags(1.0 / d)
    Ks = (S @ K @ S).tocsc()
    y = spl.spsolve(Ks, f / d)
    if np.isfinite(y).all() and _residual(Ks, y, f / d) < 1e-8:
        return y / d

    M = sp.diags(1.0 / Ks.diagonal())
    y, _ = spl.cg(Ks, f / d, rtol=1e-10, maxiter=5000, M=M)
    return y / d


def solve(nodes, elems, E, nu, fixed, loads):
    """fixed: node indices held at zero. loads: (node_ids, (n,3) forces).

    Returns (displacement (N,3), stress (N,6) nodally averaged).
    """
    K = stiffness(nodes, elems, E, nu)
    n = len(nodes) * 3
    f = np.zeros(n)
    for ids, vec in loads:
        np.add.at(f, (np.asarray(ids)[:, None] * 3 + np.arange(3)).ravel(),
                  np.asarray(vec).ravel())

    free = np.ones(n, bool)
    free[(np.asarray(fixed)[:, None] * 3 + np.arange(3)).ravel()] = False
    Kff = K[free][:, free].tocsc()
    u = np.zeros(n)
    u[free] = _solve_spd(Kff, f[free])

    # Control test on the solve itself. A sparse direct solve does not raise on
    # a near-singular system; it returns numbers, and they can be nonsense.
    # This caught the same geometry and load reading 22.7 MPa in one material
    # and 132.0 MPa in another, which is impossible in linear elasticity - the
    # field is nearly material-independent. Without it that lands in a table
    # looking like a result.
    r = np.linalg.norm(Kff @ u[free] - f[free])
    scale = max(np.linalg.norm(f[free]), 1e-12)
    if not np.isfinite(u).all() or r / scale > 1e-6:
        raise RuntimeError(
            f"the solve did not converge: residual {r / scale:.2e} of the "
            f"applied load. The constraint set is probably too small or "
            f"leaves a rigid-body mode free ({len(fixed)} nodes held).")

    return u.reshape(-1, 3), _nodal_stress(nodes, elems, u, E, nu)


def _nodal_stress(nodes, elems, u, E, nu):
    """Stress at the element's own nodes, then averaged over the elements
    sharing each node. Evaluating at nodes rather than at Gauss points keeps
    the peaks; averaging is what makes the field continuous."""
    corner = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    mid = np.array([(corner[a] + corner[b]) / 2 for a, b in EDGES])
    rst = np.vstack([corner, mid])

    dNdx, _ = _grads(nodes, elems, rst)
    B = _B_matrices(dNdx)                                   # (E, 10, 6, 30)
    ue = u.reshape(-1, 3)[elems].reshape(len(elems), 30)
    sig = np.einsum("epij,ej->epi", B, ue) @ _elastic(E, nu).T   # (E, 10, 6)

    out = np.zeros((len(nodes), 6))
    cnt = np.zeros(len(nodes))
    np.add.at(out, elems.ravel(), sig.reshape(-1, 6))
    np.add.at(cnt, elems.ravel(), 1.0)
    return out / np.maximum(cnt, 1)[:, None]


def von_mises(s):
    xx, yy, zz, xy, yz, zx = s.T
    return np.sqrt(0.5 * ((xx - yy) ** 2 + (yy - zz) ** 2 + (zz - xx) ** 2)
                   + 3 * (xy ** 2 + yz ** 2 + zx ** 2))


def interlayer(s, normal=(0, 0, 1)):
    """Tensile stress ACROSS the print layers, which is where a printed part
    actually splits. Only tension counts: compression across layers is fine."""
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    xx, yy, zz, xy, yz, zx = s.T
    sn = (xx * n[0] ** 2 + yy * n[1] ** 2 + zz * n[2] ** 2
          + 2 * (xy * n[0] * n[1] + yz * n[1] * n[2] + zx * n[2] * n[0]))
    return np.maximum(sn, 0.0)


def report(field, nodes, fixed, exclude_mm=4.0, pct=99.5, min_frac=0.35):
    """(raw peak, peak away from the supports, fraction of the part kept).

    The raw peak sits on the constrained boundary and is a singularity: it does
    not converge under refinement, it just grows. The second number ignores
    nodes within `exclude_mm` of a fixed node and takes a percentile of what is
    left, and that is the one worth quoting.

    The exclusion radius shrinks if it would eat the part. On a stub only
    14 mm long, a flat 4 mm halo removes almost everything and the "peak away
    from the supports" is then computed from a handful of nodes in the least
    stressed corner - which reads as a wonderfully low number and means
    nothing. The returned fraction is there so a caller can say so.
    """
    from scipy.spatial import cKDTree
    if not len(fixed):
        return float(field.max()), float(np.percentile(field, pct)), 1.0

    tree = cKDTree(nodes[fixed])
    r = exclude_mm
    for _ in range(5):
        mask = tree.query_ball_point(nodes, r, return_length=True) == 0
        if mask.mean() >= min_frac:
            break
        r /= 2.0
    if not mask.any():
        return float(field.max()), float(field.max()), 0.0
    return (float(field.max()), float(np.percentile(field[mask], pct)),
            float(mask.mean()))


# --- meshing -----------------------------------------------------------------

def mesh_step(path, size=3.0, order=2):
    """(nodes mm, elems) from a STEP file."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        # gmsh threads its mesher too; cap it with everything else.
        gmsh.option.setNumber("General.NumThreads", THREADS)
        gmsh.model.occ.importShapes(str(path))
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", size)
        gmsh.option.setNumber("Mesh.MeshSizeMin", size / 4)
        gmsh.option.setNumber("Mesh.ElementOrder", order)
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.model.mesh.generate(3)
        tags, coords, _ = gmsh.model.mesh.getNodes()
        etypes, _, enodes = gmsh.model.mesh.getElements(3)
        assert 11 in etypes, "expected 10-node tets"
        conn = enodes[list(etypes).index(11)]
    finally:
        gmsh.finalize()

    order_map = np.zeros(int(tags.max()) + 1, int)
    order_map[tags.astype(int)] = np.arange(len(tags))
    nodes = coords.reshape(-1, 3)[np.argsort(np.argsort(tags))]
    nodes = coords.reshape(-1, 3)
    elems = order_map[conn.astype(int)].reshape(-1, 10)
    return nodes, elems


def check_ordering(nodes, elems, tol=0.15):
    """Control test on the node ordering. If gmsh's mid-edge convention were
    not what EDGES says, every shape function would be wrong and the solve
    would return plausible-looking nonsense.

    Mid-edge nodes are NOT exactly at edge midpoints: gmsh projects them onto
    the CAD surface, so an edge across a small bore can bow by nearly half its
    own length. Testing the worst element therefore cannot separate "curved"
    from "wrongly permuted" - the two overlap. The MEAN offset can: correct
    ordering runs a few percent of edge length, a wrong permutation runs ~70%.
    """
    xe = nodes[elems]
    for k, (a, b) in enumerate(EDGES):
        L = np.linalg.norm(xe[:, a] - xe[:, b], axis=1)
        off = np.linalg.norm(xe[:, 4 + k] - (xe[:, a] + xe[:, b]) / 2, axis=1)
        ratio = (off / np.maximum(L, 1e-12)).mean()
        assert ratio < tol, (f"mid-edge node {4+k} does not lie on edge {a}-{b}: "
                             f"mean offset {ratio:.1%} of edge length")


# --- verification ------------------------------------------------------------

def verify(size=3.0, verbose=True):
    """Cantilever with a closed-form answer.

    100 x 10 x 20 mm, built in at x = 0, 50 N down at the tip. Euler-Bernoulli
    gives the tip deflection and the root bending stress. FEA should land a few
    percent stiff, because a built-in 3D face restrains Poisson contraction in
    a way beam theory does not.
    """
    import gmsh
    L, b, h, P, E, nu = 100.0, 10.0, 20.0, 50.0, 2000.0, 0.35

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        # gmsh threads its mesher too; cap it with everything else.
        gmsh.option.setNumber("General.NumThreads", THREADS)
        gmsh.model.occ.addBox(0, -b / 2, -h / 2, L, b, h)
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", size)
        gmsh.option.setNumber("Mesh.ElementOrder", 2)
        gmsh.model.mesh.generate(3)
        tags, coords, _ = gmsh.model.mesh.getNodes()
        etypes, _, enodes = gmsh.model.mesh.getElements(3)
        conn = enodes[list(etypes).index(11)]
    finally:
        gmsh.finalize()

    order_map = np.zeros(int(tags.max()) + 1, int)
    order_map[tags.astype(int)] = np.arange(len(tags))
    nodes = coords.reshape(-1, 3)
    elems = order_map[conn.astype(int)].reshape(-1, 10)
    check_ordering(nodes, elems)

    fixed = np.where(nodes[:, 0] < 1e-6)[0]
    tip = np.where(nodes[:, 0] > L - 1e-6)[0]
    u, s = solve(nodes, elems, E, nu, fixed,
                 [(tip, distribute(nodes, tip, np.array([0, 0, -P]),
                                   np.zeros(3)))])

    I = b * h ** 3 / 12
    d_beam = P * L ** 3 / (3 * E * I)
    d_fea = -u[tip, 2].mean()

    # Sample bending at MID-SPAN, not at the root. The built-in face is a
    # stress singularity: the first attempt sampled there, read +44%, and that
    # number is not a solver error - it is the real behaviour of a sharp
    # re-entrant constraint, and it does not converge under refinement. At
    # x = L/2 the moment is P*L/2 and both the support and the load are far
    # enough away for Saint-Venant to hold.
    xs = L / 2
    s_beam = P * (L - xs) * (h / 2) / I
    band = np.where(np.abs(nodes[:, 0] - xs) < size / 2)[0]
    top = band[nodes[band, 2] > h / 2 - 1e-6]
    s_fea = s[top, 0].max()

    if verbose:
        print(f"cantilever {L:.0f} x {b:.0f} x {h:.0f} mm, {P:.0f} N tip load, "
              f"{len(elems)} TET10")
        print(f"  tip deflection      beam {d_beam:.4f} mm   fea {d_fea:.4f} mm"
              f"   ({d_fea/d_beam-1:+.1%})")
        print(f"  bending at mid-span beam {s_beam:.3f} MPa  fea {s_fea:.3f} MPa"
              f"  ({s_fea/s_beam-1:+.1%})")
    return d_fea / d_beam, s_fea / s_beam


if __name__ == "__main__":
    rd, rs = verify()
    assert 0.90 < rd < 1.02, f"deflection off by {rd-1:+.1%}"
    assert 0.90 < rs < 1.10, f"stress off by {rs-1:+.1%}"
    print("\nsolver agrees with beam theory - safe to point at real parts")
