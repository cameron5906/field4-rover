# FIELD4 House Lab v07.3: reusable source subset

Experimental robot-simulation, perception and control work. This is a deliberately incomplete, source-only extraction of v07.3, prepared for public review. It is **not the original runnable House Lab application**, a Windows release, or deployable Raspberry Pi firmware.

## Included

- Godot 4 GDScript: occupancy mapping, bounded scan matching, exploration memory, semantic-plan validation, landmark navigation, person-follow and gesture-hold logic, sensor overlays, and the local perception-process client.
- Python: camera/lidar evidence fusion, short-lived anonymous person tracks, detector continuation, gesture classification, worker scheduling, replay utilities and synthetic regression tests.
- Simulation physics: attitude filter, IMU generator, drive/contact approximations, estimated robot mass properties and uncalibrated camera profiles.
- Exact dependency locks, upstream artifact URLs and SHA-256 values, plus optional dependency/model acquisition scripts.
- Procedural chair and texture source. No texture binaries are required by the included test suites.

## Intentional omissions

The furnished house layout, application entry scene, world-specific orchestration, actor trajectory/bounds, captured imagery/maps and historical run outputs are omitted. The house layout's origin was not established as synthetic, so those coordinates are not published. The retained map defaults describe an empty grid, not a floor plan; tests create small synthetic grids or explicitly artificial sensor observations.

The original native API transport, credential-entry UI and account-specific test configuration are omitted. `decisions_adapter.gd` and `semantic_planner.gd` retain offline request/response and freshness contracts; they do not send network requests themselves.

Combined robot CAD/GLB assets are omitted because the embedded third-party sensor mesh's redistribution terms were not established. Runtime binaries, export templates, downloaded wheels and model weights are also omitted. The model metadata and acquisition source preserve upstream provenance without redistributing those files.

The modules-only `project.godot` intentionally has no main scene. Opening it supports source inspection and standalone tests; pressing Run will not launch the original application. Recreating an application requires a new demonstrably synthetic scene, appropriately licensed robot assets, a reviewed orchestration layer and explicit runtime setup. The missing material must not be inferred to be publicly available elsewhere.

## Status and limitations

- Sustained person following remains unverified.
- End-to-end semantic bedroom-mission success remains unverified.
- Native Windows acceptance and Raspberry Pi portability remain unverified.
- Hardware commissioning has not been done. Physical sensor drivers, encoder interfaces, motor transport, independent watchdog and emergency-stop integration are absent.
- Mass, inertia, friction, camera projection and IMU settings are engineering assumptions, not calibration measurements.
- `pi4_budget` names a simulated compute-time profile. It is not measured Raspberry Pi throughput or a financial budget.
- Simulation tests cannot establish physical safety, braking distance, perception accuracy or ordinary human walking-speed capability.

See [architecture and interfaces](docs/ARCHITECTURE.md), [hardware-portability limits](docs/HARDWARE_PORTABILITY.md), [dependency provenance](third_party/DEPENDENCIES.md), and [verification](docs/VERIFICATION.md).

## Offline synthetic tests

From this directory, using Python 3.12 or later:

```sh
python -B -m unittest discover -s perception/tests -p 'test_*.py'
(cd perception && python -B -m unittest test_runtime_packaging)
```

These tests use standard-library logic and explicit image/model doubles. Optional model/runtime packaging checks skip when their upstream payloads are absent. No dependency installation, model download, real inference, API call or motor action is part of these commands.

With a suitable Godot 4.6 installation:

```sh
godot --headless --editor --path . --import
godot --headless --path . --script tests/test_attitude_estimator_v073.gd
godot --headless --path . --script tests/test_exploration_memory.gd
godot --headless --path . --script tests/test_room_search_integration.gd
python -B tests/build_follow_contract_fixture_v073.py
godot --headless --path . --script tests/test_follow_tracker_contract_v073.gd
```

The fixture builder produces only explicitly synthetic observations under the ignored `test_results/` directory. The optional perception-cadence test additionally uses `/bin/sleep` as an idle local child and therefore assumes a Unix-like host. Full scene, real-model, capture-based and hardware acceptance suites are outside this subset.

## Optional perception dependencies

The archived Linux lock targets CPython 3.12 x86-64; it is not a Pi/ARM lock. `perception/setup_linux.sh` creates a local virtual environment and downloads pinned packages and model weights. `perception/build_windows_runtime.py` assembles a private Windows runtime from pinned upstream artifacts. These scripts are provided for inspection and are not run by the offline tests. Review upstream terms and your platform first; current download availability is not asserted by this source snapshot.

Files generated from actual camera, scan or replay inputs may contain private images and spatial data. Keep them local and review any proposed publication individually. The ignore rules are a precaution, not a guarantee that arbitrary future files are safe to publish.

No new license grant is implied for upstream components or excluded assets. Refer to the repository's licensing statement and the applicable upstream terms.
