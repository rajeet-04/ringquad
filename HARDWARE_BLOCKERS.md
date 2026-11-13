# Remaining hardware blockers

Updated: 2026-11-14. Evidence baseline: `048ba06` on `experiment/tunnel-control`.

**Status: simulation acceptance passed; physical hardware release remains blocked.**

The provisional candidate has a 2.65 kg mass budget. Recorded results include 10/10 full missions, 61 Python tests, native actuator/latch checks, and 60/60 numerical flight runs. These results do not qualify a manufactured vehicle. Specifications, assumptions and supporting artifacts are in the [hardware report](hardware-blockers/REPORT.md) and [acceptance summary](hardware-blockers/results/SUMMARY.json).

All items below are open. Close each item with revisioned design files, raw measurements and a test report; a checked box alone is not evidence.

## 1. Final parts, complete assembly and mass closure

- [ ] Select actual ESCs, battery, regulators, connectors, bearings, fasteners and shaft adapters. Confirm the proposed motors, propellers and servos against the complete assembly.
- [ ] Finish the chassis, hinges, carrier attachments, wheel-motor shaft/roller coupling, bearing retention, steering reduction, wiring and electronics layout. Existing CAD contains seven component proposals, not a released assembly.
- [ ] Resolve tolerance stacks, fastener preload/retention, service access, corrosion protection and electrical isolation between aluminium and CFRP.
- [ ] Weigh the finished parts and vehicle; measure centre of mass and update the model/controller. The 2.65 kg figure is an allocation, not a measured assembled mass.

**Closure evidence:** released assembly drawings and manufacturing files, purchased-parts BOM, interface/tolerance review, and measured mass/CG within the validated design envelope. Revalidate any changed envelope.

## 2. Fold actuator and real linkage

- [ ] Select a compliant self-locking fold actuator. No compliant part has been selected yet. The provisional requirement is 8 Nm aggregate continuous torque, 40 Nm peak for at most 5 s, approximately 3.8 RPM output and at most 300 g for the complete mechanism.
- [ ] Design and verify the equalized linkage, end stops, backlash, holding behaviour and load distribution across all four hinges.
- [ ] Measure fold current, torque, travel and timing under uneven loading, obstruction and repeated operation; implement overload/duty protection.

**Closure evidence:** selected actuator data, completed linkage CAD, measured mechanism loads and endurance/obstruction results. Four independently capped 10 Nm simulated hinges do not establish that one real actuator/linkage can perform the same motion.

## 3. Installed propulsion calibration and clearances

- [ ] Measure each installed motor/ESC/propeller combination with the actual crown, carrier and power system. The current thrust coefficient is provisional.
- [ ] Collect at least eight steady points per rotor, covering speeds at or below 500 rad/s through at least 1035 rad/s, with thrust, voltage, current and temperature measurements and calibrated-instrument provenance.
- [ ] Fit per-rotor maps using [calibration.py](hardware-blockers/calibration.py); load measured mass, CG and thrust coefficients into the controller. Synthetic fixtures are software tests only.
- [ ] Measure installed propeller runout, dynamic flex and clearance through folding, steering and representative loads. Nominal radial tip clearance is approximately 5.7 mm; blade contact and flex are not simulated.

**Closure evidence:** raw thrust-stand data, calibration output, instrument records, clearance measurements and repeat flight-control simulations with the measured maps. Calibration fit success alone is not release approval.

## 4. Power, thermal limits and physical safety enforcement

- [ ] Measure battery/harness/connector resistance and voltage sag across the intended state-of-charge range. Confirm the battery and regulator current capability using measurements rather than C labels.
- [ ] Implement the physical autopilot/ESC telemetry interface for pack voltage/current, motor RPM/current/temperature and battery temperature. The repository does not yet demonstrate physical enforcement of those limits.
- [ ] Implement and test current limiting, low-voltage landing, temperature protection, brownout recovery and actuator duty limits against the selected components' ratings.
- [ ] Run installed endurance/thermal tests at the declared ambient temperature, including degraded cooling and increased bearing drag; test both steering servos against their stops without resetting the controller.

**Closure evidence:** measured electrical and thermal parameters, driver implementation, fault-injection logs and endurance results. The proposed 90 A pack cap, 13.2 V loaded floor and temperature trips remain provisional. The current flight sweep does not couple battery sag or thermal derating into available thrust.

## 5. Structure, material properties and fatigue

- [ ] Obtain certificates for the specified 7075 stock and temper; verify the required room-temperature yield strength and actual manufacturing process.
- [ ] Validate the assumed elevated-temperature derating. The 200 MPa aluminium allowable is a project assumption, not a certified hot-service material rating.
- [ ] Complete joint, fastener, root-radius, local buckling and CFRP laminate analysis beyond the existing elementary beam/pin screens.
- [ ] Proof-test each carrier in relevant directions to the 300 N target, verify standoff bending/torsion and the 16 Nm latch proof target, and test the complete hinge/linkage assembly with strain/deflection measurements.
- [ ] Perform fatigue, impact and fastener-retention tests; inspect for permanent deformation, cracks, loosening and loss of clearance.

**Closure evidence:** material/coupon results, detailed analysis, calibrated proof-load records and fatigue/inspection reports. The largest sampled front-knuckle force was 141.09 N; sampled simulation loads are not continuous peak loads or physical proof.

## 6. Ground contact, wheel drive and steering

- [ ] Measure tyre/floor friction and torsional contact behaviour on the intended surfaces. The simulated 3 mm contact patch radius is unmeasured.
- [ ] Validate roller preload, friction-drive efficiency, slip, bearing alignment, wear and retention. The proposed 10 N preload and minimum drive friction of 0.4 are requirements to verify.
- [ ] Measure steering holding/continuous torque, rate and temperature, including loaded stops; verify latch stiffness, backlash, engagement and break behaviour.
- [ ] Test braking, rough floors, obstacles and repeated fold/drive/unfold transitions with the completed assembly.

**Closure evidence:** floor/contact datasets, measured wheel/steering performance, wear/endurance results and recalibrated ground simulations. Existing friction/servo sensitivity passes do not qualify unmeasured surfaces or components.

## 7. Rotor/rim integrity and debris containment

- [ ] Establish the manufactured crown/laminate properties and validate rotor/rim integrity at the specified speed and operating conditions.
- [ ] Develop and perform burst/debris containment testing in a protective fixture, including the finished crown, attachments and surrounding structure.
- [ ] Redesign the guard or operating envelope if containment is inadequate; reconcile any additional mass with propulsion and mission performance.

**Closure evidence:** documented integrity and containment tests with inspected hardware. The lightweight wheel crown is not established as a fragment-containment guard; flight simulation and beam calculations cannot close this blocker.

## 8. Final model and fault-response validation

- [ ] Replace provisional mass, inertia, thrust, contact and thermal parameters with measured values; add coupled electrical/thermal derating to the flight model.
- [ ] Represent the real fold linkage and completed assembly, including measured limits and interfaces, rather than relying only on independent hinge caps and lumped inertias.
- [ ] Verify airborne latch-fault landing and telemetry-loss/low-voltage/overtemperature responses end to end. Ground takeoff inhibition after a simulated latch break has been verified; it does not establish every airborne failure response.
- [ ] Rerun the mission, steering, disturbance, geometry and fault checks against the final measured configuration, recording source hashes, hardware revision and calibration identity.

**Closure evidence:** reproducible calibrated-model results and end-to-end fault-response records, followed by a documented review of all physical qualification evidence above.

## Completion rule

Hardware release remains blocked until every applicable item has supporting evidence and the final manufactured configuration matches the validated configuration. Current simulation passes remain useful design evidence, but must not be reported as completed physical qualification.
