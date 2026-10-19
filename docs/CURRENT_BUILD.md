# Current mechanical selection

**Publication limit:** the intended current build uses C02 and C04 together, but the C02 lid, T10 aperture coupon and derived aperture profile are withheld from this public edition pending clarification of the external reference model's redistribution terms. Their omission does not change the private design. Do not substitute the older B02 lid or improvise a replacement from illustrations.

## Use as a matched set

- `C02_recessed_A1M8_lid_PROTOTYPE.stl` (current design; withheld from this public edition)
- `C04_A1M8_rearranged_shelf_PROTOTYPE.stl`
- `B01_lower_tub.stl`
- Four labeled B05 lid pillars: FL, FR, RL, RR
- `B09_side_switch_panel_BLANK.stl`

The C02 lid and C04 shelf are the recessed-A1M8 v0.2 pair. B02/B04 are superseded. B08 is omitted because the selected M4×100 socket-head fastener arrangement collides with the former buck-platform layout. Battery restraints and power hardware are not a validated selection.

## Nominal dimensions and fit

- C02 print-oriented lid envelope: 196 × 236 × 43 mm
- C04 print-oriented shelf envelope: 183 × 212 × 11 mm
- Original wheelbase: 180 mm; wheel-center track: 229 mm
- Wheels: nominal diameter 96 mm
- Four LiDAR feet seat at Z = 119 mm in the CAD assembly
- LiDAR mounting-hole centers: X = −90, Y = ±28; X = −20, Y = ±20 mm
- Roof top: Z = 153 mm; nominal lower rotating-head boundary: Z = 154.5 mm

These are CAD/design-basis dimensions, not measured acceptance results. STL has no unit metadata: import in millimeters at 100%. Do not autoscale. The LiDAR revision, camera carrier, cable envelopes and all connector/tool clearances require physical checking.

## Mechanical hardware scope

- Final LiDAR mounting proposal: four M2.5×16 screws and nominal 0.5 mm thin washers through an 11 mm printed stack
- T09 mounting-coupon check: M2.5×6 screws with those thin washers through a 3 mm coupon
- Shelf-to-tub arrangement: four M4×100 socket-head screws, eight thin flats, four split washers and four plain nuts
- Pillar/lid arrangement: eight M3×12 screws and eight plain M3 nuts, without extra flat washers in those stacks
- Side blank: four M3×12 screws, eight thin M3 flat washers and four plain M3 nuts

Measure actual screw protrusion, thread depth, washer thickness and printed stacks. Never substitute the long final LiDAR screws for short coupon screws. If M2.5×6 screws are unavailable, only the gentle no-screw alignment check can proceed.

Motor, wheel, camera, controller-hanger and electronics fasteners need separate verification. The C16 hangers assume a 90 × 70 mm carrier with an 80 × 60 mm hole pitch; this is not a verified mounting solution for four loose motor-driver boards.

## Current stopping point

Power-off fit tests and a removable mechanical dry build. There is no approved battery, complete protective electrical design or commissioned motor system. Read the manual's stop conditions before any assembly.
