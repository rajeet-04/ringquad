# Pinned physics engine and contact repair

The candidate requires the patched Bullet Featherstone engine. DART stalled around 68 degrees with the finite actuator model, including a lower-friction and a self-collision-disabled diagnostic. Bullet folded the same candidate to about 89.7 degrees. This comparison identifies an engine/contact limitation; it is not a physical hardware proof.

Unmodified Bullet then held the propeller joints at exactly 100 rad/s even though their URDF velocity limit was 1090 rad/s. The patch raises the multibody coordinate cap to the largest finite configured joint limit, at SDF construction and when updating a joint's maximum velocity. It preserves the existing 100 default for models without faster joints. It does not increase motor torque, thrust coefficient or requested speed.

Upstream source is pinned to gz-physics 7.8.0, commit `189471c86fa06ffad9c9bdb199f6928eb2fc02f3`. See [SDF construction](https://github.com/gazebosim/gz-physics/blob/189471c86fa06ffad9c9bdb199f6928eb2fc02f3/bullet-featherstone/src/SDFFeatures.cc) and [joint limits](https://github.com/gazebosim/gz-physics/blob/189471c86fa06ffad9c9bdb199f6928eb2fc02f3/bullet-featherstone/src/JointFeatures.cc). [Gazebo's engine documentation](https://gazebosim.org/api/sim/8/physics.html) describes the supported engine-selection mechanism.

Build inside the Linux ROS container:

```bash
bash hardware-blockers/physics/build.sh build/ringquad-physics
```

`run.sh` bootstraps existing containers, and `Containerfile` includes the build for new images. The launch file selects the uniquely named `gz-physics-ringquad-bullet-plugin` from the workspace build or image installation. It fails explicitly if the required engine is absent. The unique filename avoids accidentally loading the stock engine before a custom search path.

`version.txt` records the upstream revision and patch hash. The generated shared libraries stay in ignored build output; no engine binaries are committed.

Flight propellers and wheels use bounded force commands in `SteeringLock.cc`; they do not rely on ideal velocity constraints. The propeller model integrates through the real joint inertia, applies thrust and aerodynamic drag, and lets the physics engine transmit motor reaction and gyroscopic loads through the mechanism.

The patch also replaces the mesh backend's implicit 10 mm GImpact margin with 0.2 mm; the generated collision geometry already encloses the physical crown. It reads URDF-generated ODE friction fields when explicit Bullet fields are absent, maps the tyre axle to local Z for anisotropy, and avoids multiplying the coefficient twice. Rubber now uses equal longitudinal/lateral coefficients rather than the historical skid-steer approximation.

Bullet spinning friction takes an effective length, whereas SDF torsional friction specifies a coefficient and contact patch radius. The adapter multiplies them, using an explicit **unmeasured 3 mm patch-radius assumption** when none is specified. This assumption requires physical contact calibration; it is not a measured rubber property. Native pose and linear/angular velocities provide odometry, avoiding NaN angular rates in the stock pose-difference publisher. The steering acceptance check explicitly rejects non-finite rates; the earlier NaN result is marked invalid in its artifact.

The extra `ringquad_bullet.sdf` and `ringquad_precision.sdf` worlds retain stock-Bullet and finer-step DART diagnostics. They are not the current candidate defaults.
