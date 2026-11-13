# Hardware proposal and qualification record

This is a **provisional 2.65 kg engineering candidate**, not a qualified physical robot. No parts have been purchased, weighed, thrust-tested or proof-loaded. The historical `hardware-results/` evidence describes commit e9f4af1 and must not be read as evidence for this heavier candidate.

## Final recorded results

[Machine-readable acceptance summary](results/SUMMARY.json).

| Check | Result |
|---|---|
| Full missions on frozen runtime source | **10/10**: three nominal repeats, five actuator/friction/mass sensitivity cases, one fresh-image mission and one load-recorded mission |
| Python tests / native checks | **61 passed**, plus C++ finite-actuator and latch checks |
| Numerical flight uncertainty sweep | **60/60 passed**, zero crashes, nominal motor cap 0.33 Nm |
| Interface / signed steering | Seven interface checks passed; requested +/-0.2 rad/s measured **+0.1949 / -0.1997 rad/s** |
| Fault injection | Physical latch break fault observed; takeoff refused |
| Mission bounds | Worst reported goal error **6 cm**, cross-track **5.0 cm** (0.1 cm log precision); minimum reported buffered footprint margin **20.7 mm** |
| Geometry | **2028** chassis/rim-carrier checks, **1248** motor-case assembly checks and **1801** rear-support samples pass; crown enclosure check passes |
| CAD / load screen | Seven valid exported CAD parts; updated 7075/300 N structural and unchanged mass-budget screens pass |
| Podman image | `ringquad-hardware-candidate`, image `3c861c5ec85e0f2175c447a7209ca2918e9899da3672933b0526f1200bbd54c2`; dependencies resolve after ROS setup, fresh-image full mission passed |

The release mission runners recorded unchanged source hashes throughout their runs. Earlier `validated-*` folders are development runs from before the final steering slew limiter; only `release-*` folders feed the final summary. The nominal and matrix runs preserve `e9f4af1`'s historical evidence separately. Simulation acceptance is **true**; physical release readiness is **false**.

## Proposed procurement specification

| Subsystem | Candidate / required specification | Allocation and verification |
|---|---|---|
| Flight motor | Four SunnySky X2216 V3, 880 KV, 4S | Manufacturer lists 67.5 g and 32 A / 30 s. This is a short-duration limit, not a continuous rating. |
| Propeller | Standard APC 9 x 4.5 MR, correct CW/CCW pairs, positive retained adapter | 1090 rad/s command ceiling = 10,409 RPM; MR limit = 11,667 RPM. Exclude self-tightening ST variants with active braking. |
| Flight ESC | Four 50 A, 4S, current/RPM/temperature telemetry; controllable current limit | <=60 g total allocation. Verify startup, desynchronization, braking, telemetry latency and thermal envelope with installed motors. |
| Battery | 4S 3000 mAh, nominal 60C, <=330 g, measured DC resistance <=15 mOhm | C label is not evidence of usable current. Pack voltage, current and cell temperature must be monitored. |
| Main power | AWG10 short harness, loop resistance <=1.2 mOhm; connector <=0.8 mOhm | Provisional total pack/harness resistance 17 mOhm. 90 A aggregate cap, reduced as open-circuit voltage falls; 13.2 V loaded floor is provisional. |
| Wheel motor | Four Pololu 3474, 25:1, 12 V, 20D, 44 g | 550 RPM no-load, 1.9 kg cm stall, 1.6 A stall. Stall ratings do not establish continuous duty. |
| Wheel transmission | 13 mm radius friction roller on 103 mm mean-radius rear V-race | 7.923:1 additional reduction; assumed efficiency 0.8. Spring preload >=10 N, measured minimum friction >=0.4 gives 0.412 Nm. Model demand capped at 0.33 Nm. |
| Steering | Two Hitec D85MG, 6 V, 4:1 reduction, efficiency >=0.8 | 24 g servo, <=40 g complete assembly each. About 1.35 Nm peak and 1.87 rad/s no-load by calculation. Continuous thermal rating remains unmeasured. |
| Steering supply | Separate 6 V BEC >=4 A continuous, >=10 A peak | Each servo has a listed 1.4 A stall current. Brownout test with both steering against stops required. |
| Fold drive | Self-locking actuator and equalized linkage: 8 Nm aggregate continuous, 40 Nm peak for <=5 s, 3.8 RPM output, <=300 g complete | Procurement requirement, **no selected compliant actuator**. Model has four independent 10 Nm hinge caps; it does not prove a single real linkage distributes torque correctly. |
| Ground rail | Regulated 12 V, >=15 A peak, four >=2 A continuous H bridges, fold driver current limiting | Props must stop before folding; coasting wheel axles during folding. |
| Carrier / standoff | CNC 7075-T651 carrier with hollow 16 x 18 x 1 mm ribs; 17 OD / 10 ID x 100 mm 7075-T6 seamless tube | Material certificates, root radii, attachment details, fastener preload and strain/proof tests required. |
| Rear wheel support | Three POM V rollers on 625 bearings; rear CFRP V-race behind propeller plane | CAD includes nominal roller and race profiles. Bearing preload, alignment, wear, fastener retention and tolerance stack require assembly development. |
| Latch | 4 mm steel pin in double shear, 3 mm 7075-T651 aluminium ears, 12 mm lever | Simulated stiffness 300 Nm/rad, backlash 0.05 degrees, break threshold 8 Nm. 16 Nm static proof target; no physical proof completed. |

Sources: [SunnySky motor](https://sunnyskyusa.com/products/sunnysky-x2216), [APC MR propeller](https://www.apcprop.com/product/9x4-5mr/), [APC RPM limits](https://www.apcprop.com/technical-information/rpm-limits/), [APC ST braking restriction](https://www.apcprop.com/product/9x4-5mrst/), [Pololu 3474](https://www.pololu.com/product/3474/specs), [Hitec D85MG](https://www.hiteccs.com/actuators/product-details/D85MG).

The thrust coefficient 8.4e-6 N/(rad/s)^2 is a conservative **provisional estimate**, informed by adjacent manufacturer 9-inch propeller data. It is not an installed APC MR measurement. Carrier obstruction, propeller flex, low pack voltage, temperature and manufacturing can change it. `calibration.py` fits separate measured rotor maps, mass and CG; the controller consumes those parameters. Synthetic fixtures cannot qualify hardware.

## Mass contract

Body/power/fold assembly: 1102 g. Four arm/motor/drive assemblies: 4 x 280 g. Four finished crowns/races/tabs: 4 x 72 g. Four props/adapters: 4 x 15 g. Two steering assemblies: 2 x 40 g. Total **2650 g**. Allocations are design ceilings, not weighed masses. The CAD volumes file records calculated component masses using declared nominal densities. The 280 g arm allocation includes 120 g flight motor/standoff/mounts and 160 g carrier, gearmotor, rollers, bearings and fasteners. Detailed BOM closure remains required.

## Design and software evidence

- `results/final-flight-finite033.json`: 60/60 numerical flight runs pass at a 0.33 Nm nominal motor cap, zero crashes; payload, thrust loss, CG offsets, delay, noise, torque/inertia changes and gusts. This model does not couple battery sag or thermal derating into thrust.
- `results/design-checks.json`: beam/pin/drive screening at 300 N concentrated on one carrier, using the actual 95 mm maximum CAD span and a 200 MPa assumed aluminium allowable. Carrier root approximately 138 MPa, standoff combined approximately 130 MPa; carrier deflection approximately 0.49 mm. The original 6061/150 N screen is retained as `results/initial-6061-screen.json`. These elementary calculations exclude detailed joints, local buckling, fatigue, laminate failure and impacts.
- `results/power-thermal.json`: the assumed 17 mOhm circuit reaches only 12.62 V at an unrestricted 128 A request, so current limiting is a hardware requirement. A lumped motor thermal screen passes 180 s at 35 C ambient with assumed 4 K/W resistance and 10 W heat, but fails at 45 C or doubled thermal resistance. These are sensitivity screens with assumed thermal constants.
- `results/rear-support.json`: 1801 level-body fold samples check all rear collision-piece radial envelopes against the floor. The inboard race/tabs leave a minimum approximately 1.70 mm clearance in this idealized check.
- `cad/*.step`, `cad/*.stl`, `cad/volumes.json`: seven valid parametric CAD parts, with pin/bore interference checks. These are component proposals, not a released full manufacturing assembly.
- Finite latch model now applies bounded spring torque and latches a break fault. Steering servo and latch have one force-command writer, avoiding Gazebo controller overwrite. A latch fault inhibits takeoff and requests landing if airborne.
- Flight and wheel drives use explicit bounded torque rather than the multicopter motor model's velocity command. Fold transitions coast the wheels; steering references obey the requested servo slew rate.

## Load-driven material revision

The complete load-recorded mission logged 52,773 valid front-knuckle samples at 250 Hz. The largest sampled force norm was **141.09 N**, moment norm **9.26 Nm**, and latch torque **0.72 Nm**; no latch broke. These are joint-frame samples, not continuous peak values or strain measurements. One incomplete trailing CSV row was discarded. A 150 N design load would leave too little margin against this observed transient.

The revised procurement specification therefore uses **7075-T651 machined carriers/ears and 7075-T6 seamless standoffs**, with a **300 N design/proof target** and 16 Nm torsion screen. The drawing dimensions stay unchanged. The carrier's actual maximum 95 mm CAD span is used; the earlier 130 mm beam span was a conservative preliminary envelope. The material specification requires room-temperature certified yield >=460 MPa. A project-assumed 0.87 temperature derating and yield safety factor 2 give a rounded 200 MPa allowable. **That derating is an assumption requiring elevated-temperature coupons**, not a manufacturer's certified hot-service rating. Steel pin shear retains a separate 100 MPa assumed allowable.

[Kaiser plate property limits](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1010/Kaiser_Aluminum_Sheet___Plate.pdf) and [seamless-tube limits](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1009/Kaiser_Aluminum_Hard_Alloy_Drawn_Seamless_Tube_Capabilities.pdf) support the specified room-temperature material strength for appropriate stock sizes; obtain certificates for the actual stock and temper. [Kaiser's 7075 data](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1017/Kaiser_Aluminum_7075_Sheet_Coil_and_Plate.pdf) lists a typical modulus near 71 GPa. Use machined/bolted construction, protect against corrosion and isolate aluminium from CFRP; welding or unqualified heat treatment invalidates this strength assumption.

Updated CAD densities are 2.81 g/cm3. The component budget check confirms the stronger material fits the existing 120 g flight-arm and 160 g wheel-module allocations, so the simulated **2.65 kg mass budget is unchanged**. This is still an allocation, not a weighed assembled BOM.

## Simulator repairs and limits

The finite mechanism stalled around 68 degrees in DART, including reduced-friction, broken-latch and self-collision-disabled diagnostics. Bullet Featherstone folded it to approximately 89.7 degrees. A 0.25 ms DART run also remained near 66 degrees at 10.7 simulated seconds before its wall watchdog; it did not establish time-step convergence. The archived failures remain available. Moving the rear race inward separately eliminates an analytically identified second floor-contact surface; it did not by itself cure DART's stall.

Stock Bullet capped every propeller at 100 rad/s despite the 1090 rad/s URDF limit. The pinned engine patch respects the configured limit. The observed hover then used approximately 866-893 rad/s and held about 1.10 m height. The repair also removes a 10 mm implicit mesh margin, reads the converted friction coefficients, and applies torsional coefficient times an explicit, **unmeasured 3 mm contact patch radius**. See [the exact patch and setup](physics/README.md). This is a RingQuad-specific adapter: anisotropic tyre friction assumes the axle is local Z. The engine sources are pinned, but their contact model still requires floor calibration.

The stock pose-difference odometry produced NaN angular velocities. Native pose and world velocities now supply odometry, transformed into the base frame. The steering checker now rejects non-finite rates. `results/final-steering/result.json` is explicitly invalidated; its original NaN readings are not passes.

Propeller blades have an aerodynamic and inertia model plus visual geometry, but blade contact and flex are not simulated. The convex tread contact hull fills the wheel centre; blade clearance is a geometric/manufacturing check, not a demonstrated contact event. Generic Python plant benchmarks retain the historical 0.4 Nm default; the current candidate sweep explicitly uses 0.33 Nm, matching Gazebo.

Masses and inertias are lumped approximations. CAD parts omit a complete assembled frame, hinge equalizer, motor shaft adapters, electronics and wiring routing. In particular, the wheel-motor shaft/roller adapter and carrier attachment need a detailed tolerance/retention design before fabrication. The 270 mm crown envelope describes the finished rubber-coated part: a provisional foam core, thin CFRP skin and rubber tread can meet the mass allocation, but laminate, wear and containment properties have not been measured. The nominal propeller-tip radial clearance is about 5.7 mm; runout and dynamic flex must be included in the tolerance test.

## Physical qualification that remains mandatory

1. Select a compliant fold actuator/linkage, ESCs, battery, regulators, connectors, bearings and fasteners; reconcile actual mass, CG and all interfaces against the CAD assembly. Check prop flex clearance to the rear carrier and tabs.
2. Installed thrust stand: at least eight steady speed points per rotor from <=500 to >=1035 rad/s, voltage/current/temperature logging, measured mass/CG, calibrated instruments and raw provenance. Fit using `calibration.py`. Fit success never implies physical release.
3. Structural proof with fixture and strain measurements: 300 N in relevant directions on each carrier, tube torsion/bending, 16 Nm latch proof, hinge/linkage proof, followed by fatigue and fastener-retention tests. Obtain actual material and laminate properties.
4. Installed continuous thermal/current tests at declared ambient, pack state of charge, dirty bearing and degraded cooling conditions. Implement and test voltage/current/temperature enforcement in the actual autopilot and ESC interface. The present repository has no physical telemetry driver proving these limits.
5. Ground friction, preload, slip, braking, steering holding torque, fold current, rough-floor and obstacle tests. Measure complete linkage loads; a numerical hinge force is not an actuator acceptance result.
6. Rotor/rim burst and debris containment testing in a protective test fixture. The lightweight crown is not established as a fragment containment guard. Flight simulation and beam equations cannot close this gate.

No claim that all hardware blockers are gone is supported until these measurements exist. Do not manufacture or fly this candidate on the strength of the numerical results alone.

## Reproducing the checks

Start the dev environment with `./run.sh` (or build `Containerfile` as `ringquad-hardware-candidate`). Inside the ROS container, from `/home/ubuntu/ringquad_ws`:

```bash
source /opt/ros/jazzy/setup.bash
bash hardware-blockers/physics/build.sh build/ringquad-physics
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q --basetemp=/tmp/ringquad-repeat-tests src/ringquad_control/test hardware-blockers/test_commissioning.py
c++ -std=c++17 hardware-blockers/latch_test.cc -o /tmp/ringquad-latch-test
/tmp/ringquad-latch-test
c++ -std=c++17 hardware-blockers/actuator_test.cc -o /tmp/ringquad-actuator-test
/tmp/ringquad-actuator-test
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src/ringquad_control:/usr/lib/python3/dist-packages python3 scripts/separate_drive_flight_sweep.py --output /tmp/ringquad-flight.json
python3 scripts/demo_test.py --runs 3 --timeout 900 --output /tmp/ringquad-repeat-missions
python3 scripts/redesign_validation.py --workers 2 --output /tmp/ringquad-repeat-matrix
python3 scripts/redesign_steering_validation.py --output /tmp/ringquad-repeat-steering
python3 hardware-blockers/fault_inhibit.py --output /tmp/ringquad-repeat-fault
```

Use fresh output folders for the matrix and steering checks. The mission runners fail on abort, timeout, missing completion, out-of-tolerance position/cross-track/clearance or source changes. Gazebo's wrapper can require SIGTERM during cleanup even after a completed mission; the harness terminates the isolated process group. Uniformly clean middleware shutdown is not claimed.

Regenerate the CAD on the host with `uv run --python 3.12 --with cadquery==2.6.1 hardware-blockers/cad/build.py`. Run `design_checks.py` and `power_thermal.py` from the repository root for the engineering screens. The thrust CSV fixture is explicitly synthetic; never use it as bench data.
