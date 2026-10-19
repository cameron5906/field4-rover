# Verification of the public source subset

Checked on 2026-10-19 in a Linux review environment with CPython 3.12.14 and Godot 4.6.3 stable. These results apply to the source subset, not the omitted complete application.

## Completed

- Parsed every retained Python file and JSON document successfully.
- Godot headless editor import completed with no script parse errors.
- Python synthetic perception/tracking/replay contracts: **74 tests passed**.
- Offline runtime-packaging tests: **5 passed, 2 skipped**. The optional model-weight and assembled-Windows-runtime checks skipped because those upstream payloads are deliberately absent.
- Godot attitude estimator: **55/55 checks**.
- Godot exploration memory: **730 checks passed**.
- Godot synthetic room-search integration: **21/21 checks**.
- Synthetic Python tracker → Godot follow/gesture contract: **84/84 frames passed**.
- Godot perception capture cadence, using a local idle process and synthetic image: **24/24 checks**.
- Static resource-reference review: retained source dependencies resolve; output paths refer to locally generated test/replay data rather than bundled captures.
- Publication review excluded home-layout coordinates, scene-derived fixtures/captures, account configuration, original runtime history and binary/model/CAD payloads. Retained files were scanned for private absolute paths, contact identifiers and common credential formats.

The first editor import encountered a non-writable default settings/cache location. Repeating with dedicated temporary review-only settings/data/cache directories completed without that environment error. No source algorithm change was needed.

No network access, dependency installation, model acquisition, real model inference or physical motor operation was used for these checks.

## Not established

- An integrated launchable House Lab application or application-time motor safety.
- Sustained following or a completed live semantic bedroom mission.
- Native Windows acceptance, Raspberry Pi/ARM dependency compatibility or onboard timing.
- Calibration, independent emergency stopping, hardware commissioning or safe autonomous operation.
- Upstream artifact availability or redistribution permission merely from listed URLs/hashes.

The recorded-map, rendered-capture and full-scene acceptance tests were excluded with their scene-specific dependencies. Synthetic passing tests must not be described as real-image, real-person or real-robot acceptance.

## Public-subset changes

- Replaced the launchable project configuration with a modules-only project with no main scene.
- Made missing optional model weights an explicit skip in the packaging test.
- Removed personal-choice wording from the PLA engineering assumption and corrected the optional setup-script comment.
- Replaced original release/setup/verification documents with subset-specific caveats and instructions.

Control, perception and estimation algorithms were otherwise retained unchanged. `SOURCE_MANIFEST.json` records the published files and content hashes; it does not contain private paths or excluded-source metadata.
