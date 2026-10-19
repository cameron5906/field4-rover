# FIELD4 CAD

`FIELD4_Mechanical_Subset.blend` is an editable mechanical subset prepared with Blender 4.3.2. It retains C04 and the unaffected mechanical/reference shapes. It is not a complete assembly: the C02 lid and T10 aperture coupon are withheld pending clarification of the external reference model's redistribution terms. B08 and unresolved battery/power placeholders are also removed.

The file contains a scene and its dependencies, without saved editor layouts. Blender may report that it is a library file and initialize a default editor layout when opened; the named `FIELD4 public mechanical subset` scene contains the mechanical objects. This scene was opened and its 266 objects and retained mesh coordinates independently checked.

To inspect it, open the file in Blender 4.3.2, then select `FIELD4 public mechanical subset` in the scene selector if needed. Alternatively, start a blank file, use File > Append, enter this file's `Scene` folder, and append that named scene. Keep automatic Python execution disabled. Save any personal working copy separately; editor layouts saved later can retain machine-specific paths.

The LiDAR display is an original cylinder-and-box schematic named `REF_A1M8_SCHEMATIC_PROXY_NOT_FOR_FIT`. It is deliberately approximate, is not printable, and must not be used as a collision or physical-fit reference. The exact third-party sensor STL, display cache and manufacturer drawing are not redistributed.

## Print selection

- `stl/current/`: B01 tub, B05 pillars, B09 blank and C04 shelf; its required matched C02 lid is not distributed here
- `stl/coupons/`: small fit gauges
- `stl/conditional/`: motor mounts, camera parts and C16 controller hangers; only use after verifying actual parts and required hardware

STLs are in nominal millimeters at 100% scale and retain their earlier print orientations. Run a real slicer with the actual machine/material profile. B02/B04 are superseded, and B08 is not included. Do not use a historical lid as a substitute for the withheld current lid.

## Audit

Run `python3 source/audit_stls.py` from any directory. It writes `validation/independent_stl_audit.json`, checking binary STL length, repeated edges, degenerate triangles, positive signed volume, bounding boxes and SHA-256 hashes. It does not establish all-triangle self-intersection freedom, printability, physical fit or strength.

## Historical generation source

`source/historical/` preserves earlier first-party generators for study. The exact-reference-derived numerical aperture profile is withheld. These sources are not a one-command build of the public mechanical subset. In particular:

- `build_sleek.py` and `build_original_reference.py` generate the earlier RevB arrangement, including obsolete B02/B04/B08 and unapproved power placeholders. Generated outputs are not the current print selection.
- `build_recess_v02.py` expects an earlier RevB editable input, the withheld aperture profile and separately obtained exact sensor reference files. Those inputs are intentionally absent here, and the old generator still describes a buck-platform relocation that has since been superseded by omission.
- `verify_sensor_envelope.py` requires the exact historical third-party sensor mesh, which is not included.
- `build_dimensioned_views.py` expects the earlier full design manifest, not the current STL audit JSON.

Use a new output folder when studying these generators. Do not overwrite the current scene or mix historical generated parts into the current assembly. Rebuilding a changed design requires a fresh geometric audit, appropriate source permissions, slicer review and physical fit checks.
