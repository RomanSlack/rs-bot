---
description: Get oriented in this project from scratch
---

You have amnesia. Get your bearings before doing anything else.

**Know the point of this project before you read a line of it.** rs-bot is not
a simulation of a robot that might get built one day. The simulation IS the
deliverable, and the goal is a digital twin so exactly 1:1 with reality that the
physical build can be done in ONE SHOT: parts ordered once, printed once,
assembled once, working. Nothing here is bought yet, and every hour spent making
the model match reality is an hour not spent reprinting a part at two weeks and
real money per mistake.

That is why this repo looks the way it does. The verification layer in `cad/`
(interference, clearance, fastener, tool-access, tolerance, wiring, FEA,
printability) is larger than the parts it checks, and that is correct rather
than excessive. A check that catches one wrong bracket has paid for itself.

So when you work here, the standing question is always **"does this match the
real thing, and how would I know?"** Guessing a dimension is the failure mode
this project exists to prevent. Prefer measuring, and prefer a check that can
fail over a comment that says it is fine.

1. Read the root README and any CLAUDE.md.
2. Skim the main source directories — front end, back end, or whatever the
   equivalents are here. Breadth over depth: what the pieces are and how they
   talk to each other, not how any one of them works inside.
3. Note how it is run, tested, built and deployed — scripts, Makefile, CI
   config, infra directories.
4. Read the last ~20 git commits to see what has been happening lately.

Then report back concisely:

- what this project is, in a couple of sentences
- how it is structured, and where the important code lives
- the commands worth knowing
- what has been changing recently, and anything that looks half-finished

Prefer reading over guessing, but stay at altitude — do not go deep into any
one file. Ask before changing anything.
