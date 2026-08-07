"""What the printed structure is made of, in one place.

docs/materials.md settles it: "Order PA6-CF, from Unionfab or Weerg. It is the
only material that passes." So the printed structure weighs PA6-CF, and every
mass here has to be quoted against PA6-CF's density, not PETG's.

It was quoted against PETG for a while - 1.270 g/cm3 in FIVE files (cad/chassis,
cad/ankle, cad/thigh, cad/shin and cad/masses), on the stated grounds that PETG
"is what those parts were costed in". Costed in, not ordered in: the order is
PA6-CF at 1.19, and quoting the mass off the cost basis made the whole robot
read about 24 g heavier than it will build. That is the repo's own rule broken
five ways - a density in five files is four places to drift - so it lives here
now, as a leaf module with no build123d, and both the CAD part files and the
physics mass table import it. Import it; do not retype it.

INFILL is here for the same reason: it was the other half of the same duplicated
line in every one of those files.
"""

DENSITY = 1.19    # g/cm3, PA6-CF solid. docs/materials.md, Unionfab / Weerg.
INFILL = 0.60     # a print is not solid; see cad/shin.py
