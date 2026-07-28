"""The RS-BOT side decal.  uv run python -m cad.decal

Generates cad/out/decal.png, which src/rsbot/model.py hangs on the outside of
each chassis side plate as a textured visual geom.

Purely cosmetic and deliberately so: it carries no mass, no collision, and it
is not in MESH_STEM or any of the fit checks. It exists because a robot you are
going to look at for months should look like something, and because a name on
the side is how you tell two builds apart in a photo.

Caution-tape stripes because the real thing has a 132 N belt and a 2.9 N.m
servo in it, and hazard stripes on a machine that can pinch you are honest.
"""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).parent / "out"

W, H = 768, 768          # SQUARE: MuJoCo cube textures require it,
                         # and a 2d texture on a box tiles instead
                         # of mapping once. 60 x 60 mm on the side.
BAND = 150               # hazard band depth, top and bottom
STRIPE = 88              # stripe pitch along the band
YELLOW = (247, 181, 22)
BLACK = (24, 24, 26)
TEXT = (24, 24, 26)

FONTS = ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
         "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")


def _font(size):
    for p in FONTS:
        if Path(p).exists():
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def build():
    im = Image.new("RGB", (W, H), YELLOW)
    d = ImageDraw.Draw(im)

    # Hazard bands top and bottom. The stripes lean the same way on both, so
    # the panel reads as one piece of tape rather than two.
    for y0, y1 in ((0, BAND), (H - BAND, H)):
        d.rectangle([0, y0, W, y1], fill=YELLOW)
        for x in range(-BAND, W + STRIPE, STRIPE):
            d.polygon([(x, y1), (x + STRIPE // 2, y1),
                       (x + STRIPE // 2 + BAND, y0), (x + BAND, y0)],
                      fill=BLACK)

    # Name, fitted to the width rather than guessed from the height. Guessing
    # put "RS-BOT" wider than the sticker and clipped both ends.
    txt = "RS-BOT"
    size, target = 10, W * 0.80
    while True:
        f = _font(size + 4)
        l, t, r, b = d.textbbox((0, 0), txt, font=f)
        if r - l > target or size > H:
            break
        size += 4
    f = _font(size)
    l, t, r, b = d.textbbox((0, 0), txt, font=f)
    d.text(((W - (r - l)) / 2 - l, (H - (b - t)) / 2 - t), txt, font=f,
           fill=TEXT)

    # Thin keyline so the decal has an edge against an orange chassis.
    d.rectangle([0, 0, W - 1, H - 1], outline=BLACK, width=5)
    return im


def main():
    """Two files, and both transforms are needed.

    MuJoCo maps a cube texture onto each face with its own fixed convention,
    which lands this artwork rotated a quarter turn and mirrored. Rather than
    fight it with geom eulers - which also rotate the geom's box, so the
    sticker stops being flush - the correction is baked into the image.

    And the two sides need opposite handedness: the same texture on both faces
    reads correctly on one and backwards on the other, because you are looking
    at them from opposite directions.
    """
    OUT.mkdir(exist_ok=True)
    im = build().transpose(Image.ROTATE_270)
    left = im.transpose(Image.FLIP_LEFT_RIGHT)
    out = []
    for name, img in (("decal.png", left), ("decal_r.png", im)):
        f = OUT / name
        img.save(f)
        out.append(f)
        print(f"wrote {f}  ({img.width} x {img.height})")
    return out


if __name__ == "__main__":
    main()
