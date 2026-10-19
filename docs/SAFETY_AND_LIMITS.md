# Safety and validation limits

This repository records prototype design work. It does not certify that the robot is safe or ready for unattended operation.

## Mechanical

- Disconnect every power source during fit checks and assembly.
- Print and pass small coupons before committing the large lid and shelf.
- Check real sensor revision, motor-face relief, screw insertion, hubs, tires, connectors and tool access.
- Do not force a nut pocket, bow a sensor foot, bottom a screw in a motor or LiDAR, or use lid screws to overcome an obstructed fit.
- Model-based checks do not establish print strength, fatigue, shock, cooling, vibration or environmental protection.
- The simplified sensor display is for illustrations only. The current aperture profile and C02/T10 geometry are withheld pending reference-model redistribution clarification. The real sensor must still pass the fit gates; no replacement opening can be inferred from these illustrations.

## Electrical

Battery selection, current limiting, encoder voltage compatibility, fuses, wire/connector ratings, regenerative-energy handling, watchdog/stop implementation, safe startup and loss-of-command behavior remain unresolved or uncommissioned.

Hardware diagrams are engineering proposals. Do not connect motors or encoders directly to Raspberry Pi GPIO. A motor-driver voltage range does not establish motor or battery compatibility. A bench supply's current ceiling is not a replacement for protection or a verified stop circuit. Keep power disconnected until the completed design and harness have been reviewed.

## Software

The archived v07.3 work is experimental simulation and perception code. It is not physical-robot firmware. Synthetic/replay checks do not establish native Windows execution, sustained people-following, complete room-navigation performance, real-time behavior on a Pi or hardware safety.

Physical-robot work needs sensor/motor adapters, timestamp alignment, measured wheel/encoder calibration, deployment-specific dependency validation and a separate safety controller. Any learned or cloud-based planner must remain outside motor timing and the safety-stop path.

## Public edition

Private purchase records, personal histories, credentials, machine-specific paths, potentially private environment layouts and captures, and third-party assets without established redistribution terms are not part of this edition. Omitting them does not imply that the remaining source is a complete runnable application. Each subsystem documents its scope.
