# Hardware portability and commissioning limits

No physical-robot commissioning has been performed. Neither this source subset nor the original experimental simulator is ready to drive an assembled robot.

A deployment needs independently verified adapters for camera capture, lidar serial decoding, encoder increments, IMU packets and motor commands. Preserve explicit timestamps, axes, units, calibration identifiers, epoch resets and validity states. Missing or invalid readings must never become clear space or zero motion.

A shared controller boundary should accept immutable measured observations and return bounded command proposals. The actuator side needs an independent local watchdog, command expiry, communication-loss stopping, verified current limiting and a physical emergency stop that remain effective without the UI, perception process or semantic model. Revalidate observation age and command ownership immediately before actuator application.

Before powered movement:

1. Capture and replay real sensor streams with motors disabled. Verify polarity, units, clock alignment, dropped packets, reordering and failure handling.
2. Measure camera intrinsics/extrinsics and lidar mount/timing. Account for motion during a rotating scan rather than assuming every beam was acquired simultaneously.
3. Measure assembled mass, effective wheel radius/track width, encoder scale, motor deadband, braking, traction and current limits. The included numerical estimates are not substitutes for these measurements.
4. Benchmark complete perception/control latency on the actual target under CPU and thermal load. The x86 dependency lock and simulated Pi-oriented profile do not demonstrate ARM compatibility or real-time performance.
5. Validate the independent stop chain in supervised, low-speed tests in a clear area. Planar lidar does not detect every obstacle height, stair or drop-off.
6. Evaluate sustained navigation and following with real lighting, occlusion, changed clothing, people entering/leaving, lost observations and explicit stop events. Record failures and safety interventions alongside successes.

Simulation tests and synthetically injected semantic responses establish limited software contracts. They do not prove general room-finding, stable human following or safe autonomous household operation.
