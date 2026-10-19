# FIELD4 beginner visual build manual

This public edition describes a **power-off mechanical prototype**. The assembly is uncommissioned. The PDF includes actual-fit checks and explicit stopping points; it is not approval to wire or power the robot.

## Contents

- `FIELD4_Beginner_Visual_Build_Manual.pdf`: illustrated, 16-page beginner guide.
- `make_manual.py`: editable ReportLab PDF authoring source.
- `render_steps.py`: editable Blender rendering source.
- `annotations.json`: normalized callout coordinates for the supplied views.
- `renders/`: public, model-derived PNG illustrations used by the PDF.

The intended assembly uses the matched C02 lid and C04 shelf, plus retained B01 tub, four B05 pillars, and B09 blank. **The current C02 lid, T10 aperture gauge, and aperture profile are withheld pending reference-model redistribution clarification.** Their geometry and affected illustrations are not distributed in this public subset. The public files cannot complete the intended lid/aperture fit gates. Do not substitute old B02/B04 parts, invent a new lid from the illustrations, or treat a visual proxy as a fit design. B08 remains excluded.

## Rebuild the PDF

Requirements: Python 3, ReportLab, and DejaVu Sans regular/bold fonts. With these installed:

```sh
python3 make_manual.py
```

The PDF is written beside the script. Optional output and font directories:

```sh
python3 make_manual.py --output-dir ./output --font-dir /path/to/dejavu-fonts
```

`--font-dir` must contain `DejaVuSans.ttf` and `DejaVuSans-Bold.ttf`. The script also checks `FIELD4_FONT_DIR`, a local `fonts/` directory, and the standard Linux DejaVu directory. Included PNGs are sufficient to rebuild the PDF without Blender.

## Regenerate the illustrations

The renderer targets the sanitized scene at `../../cad/FIELD4_Mechanical_Subset.blend`. Run from this directory with Blender 4.3 or a compatible newer version:

```sh
blender --background --threads 4 --python render_steps.py
```

To override the input scene or render folder:

```sh
blender --background --python render_steps.py -- --scene /path/to/FIELD4_Mechanical_Subset.blend --output-dir ./renders
```

The renderer rejects scenes containing withheld C02/T10 objects and requires the original simple geometric object `REF_A1M8_SCHEMATIC_PROXY_NOT_FOR_FIT`. This sensor proxy is intentionally approximate; it is not manufacturer CAD, an exact envelope, or a collision/fit reference. All views that contain it are labeled schematic in the PDF. The cover shows an open chassis with no lid; the gauge image contains T09 only. The lid step uses a text notice rather than an affected image. The original third-party sensor mesh is not included or used by these public illustrations.

The callouts assume the current component coordinates, camera framing, 1400 × 900 resolution, and render transforms. Recalculate and visually verify the callouts after changing geometry or camera settings.

## Essential limits

- C02/T10-dependent steps remain on hold. Withholding files does not authorize a substitute design or bypass any fit gate.
- Physical fit, strength, vibration, thermal performance and electrical commissioning are unverified.
- Check actual component revisions, screw engagement, print dimensions, cable routes and clearances. Rendered references do not establish compatibility.
- T09's screw-fastened coupon check requires separate M2.5 × 6 screws with specified 0.5 mm washers. Never substitute the final M2.5 × 16 screws through the 3 mm coupon.
- Keep all power sources disconnected. Resolve battery/protection, E-stop, driver current limits, encoder compatibility, safe boot and loss-of-command behavior before any separately reviewed powered test.
- Motor, hub, wheel and electronics steps are conditional on real-component checks. A simulator does not constitute hardware validation or ready-to-run Pi firmware.

The renderer widens framing where needed to keep visible geometry inside the image. It writes the corresponding `annotations.json` beside the chosen render directory. This adjusts the baseline callouts for framing changes only; it does not relocate labels for edited geometry.
