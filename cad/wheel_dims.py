"""The wheel's outside dimensions, and nothing else.

Split out of cad/wheel.py for the same reason cad/servo_dims.py was split out
of cad/servo.py: so that src/rsbot/model.py can import them without pulling in
build123d, which is a dev-group dependency and has no business being required
to load a physics model.

THE REASON THIS FILE EXISTS, found 2026-08-02. These numbers were in three
places each and one of them was wrong by 5 mm:

    WHEEL_R       sim 0.040   cad/wheel.py 40.0   cad/envelope.py 40.0    agreed
    WHEEL_HALF_W  sim 0.012   cad/wheel.py 12.0   cad/envelope.py 12.0    agreed
    tyre wall     sim 0.008   cad/wheel.py  3.0                       DISAGREED

The sim's `WHEEL_TIRE_T = 0.008` said the tread is 8 mm thick. The CAD builds
3 mm, and the whole wheel design turns on that number: the rigid rim stops at
R - TYRE_T = 37, and cad/printability.py asserts there is no rigid material
outside r = 38.5 because the part has to stand on rubber, not on plastic. At
8 mm the rim would stop at 32 and the assertion would be about a different
wheel.

It survived because **nothing read it.** One definition, zero uses, wrong by
5 mm, sitting in the file that describes the robot to the physics. A constant
nobody uses is not harmless: it is a wrong answer waiting for the first person
who trusts it, and it reads as verified because it sits beside numbers that are.

The two that agreed had comments saying "the sim's WHEEL_R", which is an author
noticing the duplication, writing it down, and copying the number anyway.

These are the only copy. Import them; do not retype them.
"""

R = 40.0                 # outer radius, on the tread
HALF_W = 12.0            # half width, so the disc is 24 mm across
TYRE_T = 3.0             # tread thickness, so the rigid rim stops at r = 37

# The sole has to be rubber everywhere it can touch the floor, so no rigid
# feature may reach past this. cad/printability.py checks it.
RIGID_MAX_R = R - 1.5
