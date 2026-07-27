# Materials you can actually order

The operator does not own a printer. Everything gets sourced from a print
service, which makes this a different question from the one `docs/stress.md`
originally answered: a material that scores well and cannot be bought is not a
result.

The four filaments the FEA started with (PETG, PLA, ABS, PA6-CF) are what you
put on a printer you own. This file adds what a service sells, re-runs every
part against it, and comes to a conclusion that is not the obvious one.

## The answer first

**Order PA6-CF, from Unionfab or Weerg. It is the only material that passes
every part, and it turns out you can buy it as a service.**

The obvious move is JLCPCB or PCBWay, and it does not work. Their strong
plastics are PA12-class, and PA12-class fails both the shin and the roll
bracket. The materials that pass are the same one the design already assumed,
sold by a different set of vendors.

## What the services offer

| material | process | vendor | in-plane | interlayer | E MPa | source |
|---|---|---|---|---|---|---|
| **PA6-CF** | **FDM** | **Unionfab, Weerg** | **95** | **38** | **6000** | matches Bambu's 102/48 measured |
| PA12-HP | MJF | JLCPCB | 48 | 48 | 1800 | vendor datasheet |
| PA12 | SLS | PCBWay | 47 | 47 | 1900 | vendor page |
| PA12 + 35% GF | SLS | PCBWay | 45 | 45 | 2600 | vendor page |
| PA12-CF | FDM | JLC3DP | ~70 | ~30 | ~4000 | **not a datasheet** |
| TPU 95A | FDM | JLC3DP, most others | 45-50 | - | - | vendor blog |

Prices are quote-driven and none publish a per-part figure. The anchors they do
publish: JLC3DP from $0.30 a part, PA12-HP nylon from $1.00. The real number
comes from uploading the STLs.

### Two things that are easy to get backwards

**Powder-bed parts are solid.** MJF and SLS have no infill, so they take no 60%
knockdown, and no meaningful layer plane, so their in-plane and interlayer
allowables are the same number. FDM takes the knockdown on both and has a weak
axis. `cad/stress.py` splits these into `FDM` and `POWDER` sets for exactly
this reason; swapping a material between them is a silent factor of 1/0.6.

**The headline strength is not the comparison.** PA6-CF at 95 MPa looks far
ahead of MJF PA12 at 48. After knockdown it is 57 against 48, and in the
interlayer direction it is 22.8 against 48 - so MJF is *twice as strong as
PA6-CF in the direction that fails*. That is why MJF looks so attractive right
up until you check deflection.

### The FDM PA12-CF row is still fiction

JLC3DP publishes no datasheet for it. The 70 / 30 / 4000 here is a conservative
reading across 52 MPa (Weerg PA12-CF), 83.5 / 32.7 XZ-vs-ZX (Stratasys Nylon
12CF) and 88.5 MPa for parameter-optimised PA12-CF in the literature. It is the
one row that could move a part between pass and fail.

## Every part, every material

At the factored design loads. in-plane / interlayer utilisation, bold is over.

| part | PA6-CF | MJF PA12 | SLS PA12GF | FDM PA12CF |
|---|---|---|---|---|
| thigh | 37 / 18% | 44 / 8% | 47 / 9% | 51 / 22% |
| shin | 86 / 83% | **102%** | **109%** | **116 / 105%** |
| ankle yoke | 19 / 38% | 23 / 18% | 24 / 19% | 26 / 48% |
| roll bracket | 94 / 60% | **112%** | **119%** | **128 / 76%** |
| wheel body | 4 / 3% | 5 / 2% | 5 / 2% | 5 / 4% |
| **whole leg, fused** | 37 / 32% | 44 / 15% | 47 / 16% | 50 / 40% |
| chassis | 18 / 18% | 22 / 9% | 23 / 9% | 25 / 23% |

**PA6-CF passes everything.** It is the only column with no bold in it. The
roll bracket used to fail here too, at 101 / 128%, and came back to 94 / 60%
across two changes made for completely different reasons: moving its arm 0.8 mm
outboard for wheel running clearance, and then 0.5 mm inboard so the wheel
servo seats flat instead of standing off. See docs/assembled-strength.md.

The fused-leg row is a global load-path model on a deliberately coarse mesh, so
it under-reads local peaks. It is there for deflection, not for stress.

**Strength is not what rules out MJF. Stiffness is.** E is 1800 against 6000,
and deflection scales as 1/E:

| part | PA6-CF | MJF PA12 |
|---|---|---|
| shin | 8.7 mm | **29.0 mm** |
| chassis | 4.9 mm | 16.5 mm |
| **whole leg, fused** | **13.2 mm** | **44.1 mm** |

At the factored flip load an MJF leg moves its foot 44 mm. Nothing in this
geometry survives that; `cad/envelope.py` argues over single millimetres.

**The wheel body is nowhere near its limit** - 5% at worst. It is a stubby disc
with a short load path, so its material is free. Print it in whatever the rest
of the order is in.

## Tolerance, which is a separate question from strength

The services quote **+/-0.3 mm** (JLCPCB MJF, PCBWay SLS), and two printed
parts facing each other can each be 0.3 mm out. So any drawn gap under about
0.8 mm is inside the tolerance band and is not really a gap.

This mattered. The roll bracket's arm ran parallel to the whole face of the
wheel at **0.1 mm**, against a part that turns at 40 rad/s - a drawing
clearance, not a manufacturing one, in a section whose own comments reason in
0.05 mm steps. `cad/assemble_check.py` reported it as clean, because it only
tested *interference*, so 0.1 mm and 20 mm read identically. It now reports
minimum distance as well, and distinguishes a running clearance from a bolted
face that is supposed to touch.

## Recommendation

1. **Structure: PA6-CF, from Unionfab or Weerg.** The only material that passes
   every part. Specify infill: the 60% knockdown is an assumption about a print
   you are not doing yourself, and service FDM often defaults far lower.
2. **Tyre: TPU 95A**, any FDM service.
3. **Do not order MJF PA12 for the legs.** It passes on strength and fails on
   deflection, which is the trap - the utilisation table alone says yes.
4. **If you must use a PA12-class service**, the shin and the roll bracket need
   redesigning first, and nothing else does.

## Sources

- [Unionfab PA6-CF FDM](https://www.unionfab.com/materials/fdm/pa6-cf)
- [Weerg PA6-CF](https://www.weerg.com/3d-printing-materials/nylon/pa-6-cf-carbon-fiber)
- [JLCPCB PA12-HP Nylon](https://jlc3dp.com/help/article/200-PA12-HP-Nylon)
- [JLC3DP FDM materials](https://jlc3dp.com/3d-printing/fdm)
- [PCBWay PA12 (SLS)](https://www.pcbway.com/rapid-prototyping/3d-printing/plastic/nylon/PA12/)
- [PCBWay glass fiber nylon PA12+35%GF](https://www.pcbway.com/rapid-prototyping/3d-printing/plastic/nylon/Glass-fiber-nylon/)
- [Weerg PA12-CF](https://www.weerg.com/3d-printing-materials/nylon/pa-12-cf-carbon-fiber)
- [CNC Kitchen, carbon fibre nylon PA6 vs PA12 tested](https://www.cnckitchen.com/blog/carbon-fiber-nylon-in-3d-printing-pa6-vs-pa12-tested)
