"""The STS3215's dimensions, and nothing else.

Split out of cad/servo.py so that src/rsbot/model.py can import them without
pulling in build123d, which is a dev-group dependency and has no business being
required to load a physics model.

THE REASON THIS FILE EXISTS is that these numbers were in six places with three
different values: cad/servo.py had the STEP's 45.40 x 24.80 x 39.60, while
cad/envelope.py, cad/assemble_check.py, cad/robot.py and cad/assembly.py all
still carried 45.2 x 24.7 x 35.4 - the original product-listing guesses,
superseded twice and never updated. envelope.py is the file that solved where
the ankle bearing is allowed to live.

These are the manufacturer's drawing, for the 12 V C018 part. They are the only
copy. Import them; do not retype them.
"""

LENGTH = 45.23           # x, along the case
WIDTH = 24.73            # y, across it
HEIGHT = 36.50           # z, ALONG the output shaft, tip to tip

# and the shaft-axis breakdown, which adds up to HEIGHT exactly
SPLINE_PROUD = 3.40      # spline above the case's top face
BODY = 29.00             # the case itself
REAR_BOSS_PROUD = 4.10   # rear pivot boss below the bottom face

SHAFT_X = 12.50          # shaft axis, from the case centre, along LENGTH
SHAFT_R = 2.95           # 25T spline, 5.9 mm across
IDLER_R = 3.00           # Ø6 rear pivot boss

assert abs(SPLINE_PROUD + BODY + REAR_BOSS_PROUD - HEIGHT) < 1e-9
