"""CAD and analysis package.

The thread cap lives here because it has to run before numpy is imported:
OpenBLAS and OpenMP read these variables at load time and ignore them
afterwards, and importing this package is the first thing that
`python -m cad.anything` does.

Left uncapped, a stress run takes every core on the machine and the desktop
stops responding. Half is enough: the FEA solve is memory-bandwidth bound well
before it is core bound, so the last eight threads buy very little.

Override with RSBOT_THREADS=n, or RSBOT_THREADS=0 for no cap at all.
"""

import os

_n = os.environ.get("RSBOT_THREADS")
if _n is None:
    _n = str(max(1, (os.cpu_count() or 2) // 2))
if _n != "0":
    for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
               "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
        os.environ.setdefault(_v, _n)

THREADS = max(1, int(_n))
