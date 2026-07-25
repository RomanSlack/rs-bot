# Mass budget

Target is the **final** robot, arms and all. The sim model carries the
not-yet-built parts as torso ballast so the legs are sized and the balancer is
tuned against real inertia from day one. On the stage-1 hardware build this is
literal: a ~600 g block where the arms and head will go.

| Item | Count | Each | Total |
|---|---|---|---|
| Leg servos (hip, knee, wheel, ankle x2) | 8 | 60 g | 480 g |
| Arm servos (shoulder pitch/roll, elbow, gripper x2) | 8 | 60 g | 480 g |
| Head servos (pan/tilt) | 2 | 60 g | 120 g |
| Printed structure (PETG) | | | 400 g |
| Pi 5 + cameras + IMU + harness | | | 250 g |
| 3S pack | | | 200 g |
| **Total** | | | **1930 g** |

## How that maps onto the sim model

| Body | Mass | Stands in for |
|---|---|---|
| torso | 1250 g | structure, Pi, battery, **plus arm/head ballast** |
| thigh x2 | 130 g | hip + knee servo, thigh shell |
| shin x2 | 100 g | wheel servo, shin shell |
| wheel x2 | 60 g | wheel, tyre |
| sole x2 | 20 g | rocker plate |
| **Total** | **1870 g** | |

## Torso fore/aft trim

The torso mass sits **8.5 mm forward of the hip axis**.

This is not cosmetic. The retracted soles park about 40 mm behind the wheel
axle, and without the trim the standing CoM lands 5.7 mm aft of the contact
patch. The balancer then has to hold a permanent 1.2 deg forward lean just to
stand still, which eats lean authority and shows up as a standing position
offset. Nulling it in the model dropped the residual standing lean from
2.1 deg to 0.8 deg and the standing drift from 38 mm to 18 mm.

On the real build, this is where the battery goes. Expect to trim it again on
hardware once the arms are on, since they move the CoM forward.
