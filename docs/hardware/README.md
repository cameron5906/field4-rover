# Hardware proposal

The technical notes and three vector diagrams record the Pi 4/Pico design proposal. They are not finished schematics, wiring instructions, or evidence that the protection circuit has been built and tested.

- `architecture_proposal.txt`: sanitized technical design and staged validation discussion
- `diagrams/01_system_architecture.svg`: sensing/navigation/motor-controller split
- `diagrams/02_power_and_stop.svg`: proposed power and stop structure
- `diagrams/03_pinmap_and_motor_cell.svg`: proposed signal allocation and one driver channel

The proposal's component ratings and numeric targets must be checked against actual components and current manufacturer documentation. Final fuse, wire, contactor, watchdog, energy absorption, ADC protection, battery and calibration decisions remain open. The public mechanical manual is restricted to unpowered assembly.

Manufacturer source links are listed in `../../resources/manufacturer_sources.json`.
