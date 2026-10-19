import bpy
import argparse
import sys
import struct
import json
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
# Run: blender --background --python render_steps.py -- [--scene PATH]
R = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description="Render public FIELD4 manual illustrations.")
parser.add_argument("--scene", type=Path, default=R.parent.parent / "cad" / "FIELD4_Mechanical_Subset.blend")
parser.add_argument("--output-dir", type=Path, default=R / "renders")
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
args.output_dir.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.open_mainfile(filepath=str(args.scene.resolve()))
def strip_png_metadata(path):
    """Remove text/EXIF fields without decoding or changing the rendered pixels."""
    raw = path.read_bytes()
    if raw[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("Expected a PNG render")
    result = bytearray(raw[:8])
    at = 8
    while at < len(raw):
        size = struct.unpack(">I", raw[at:at + 4])[0]
        kind = raw[at + 4:at + 8]
        end = at + size + 12
        if kind not in {b"tEXt", b"zTXt", b"iTXt", b"eXIf"}:
            result.extend(raw[at:end])
        at = end
    path.write_bytes(result)

s=bpy.context.scene
s.render.engine='CYCLES';s.cycles.samples=16;s.cycles.use_denoising=False
s.cycles.device='CPU'
s.render.resolution_x=1400;s.render.resolution_y=900;s.render.resolution_percentage=100
s.render.film_transparent=True
orig={o.name:o.location.copy() for o in bpy.data.objects}
# Original, deliberately approximate proxy; never use it for fit or collision checks.
sensor='REF_A1M8_SCHEMATIC_PROXY_NOT_FOR_FIT'
if sensor not in bpy.data.objects:
 raise RuntimeError('Public schematic sensor proxy missing; use the sanitized public CAD scene.')
base=['B01_lower_tub']; pillars=[o.name for o in bpy.data.objects if o.name.startswith('B05')]
shelf=['C04_A1M8_rearranged_shelf_PROTOTYPE']
# C02 lid and T10 aperture gauge are withheld; never render them into public assets.
if any(name in bpy.data.objects for name in ['C02_recessed_A1M8_lid_PROTOTYPE', 'T10_A1M8_aperture_coupon']):
 raise RuntimeError('Withheld C02/T10 geometry is present. Use the current public scene.')
wheel=[o.name for o in bpy.data.objects if o.name.startswith(('REF_tire','REF_wheel','REF_hub','REF_motor','REF_shaft','B10','tread'))]
views={
'cover':(base+pillars+shelf+[sensor]+wheel,(360,-460,390),(0,0,85),390,{}),
'parts':(base+pillars+shelf,(360,-460,400),(0,0,120),380,{shelf[0]:(0,0,45),**{p:(0,0,45) for p in pillars}}),
'coupons':(['T09_A1M8_four_foot_coupon'],(140,-200,250),(0,0,0),180,{}),
'lower':(base,(320,-420,480),(0,0,60),350,{}),
'pillars':(shelf+pillars,(300,-420,440),(0,0,130),320,{}),
'sensor':(shelf+[sensor],(240,-350,390),(-25,0,120),300,{sensor:(0,0,25)}),
'shelf':(base+pillars+shelf+[sensor],(330,-440,450),(0,0,100),430,{shelf[0]:(0,0,35),sensor:(0,0,35),**{p:(0,0,35) for p in pillars}}),
'blank':(['B09_side_switch_panel_BLANK'],(140,270,190),(0,100,89),110,{}),
'motor':([o.name for o in bpy.data.objects if o.name.endswith('_FR') and o.name.startswith(('REF_motor','REF_shaft','B10'))],(250,-270,220),(90,-55,55),150,{}),
'wheel':([o.name for o in bpy.data.objects if (o.name.endswith('_FR') and o.name.startswith(('REF_tire','REF_wheel','REF_hub','REF_motor','REF_shaft','B10'))) or (o.name.startswith('tread.') and 32<=int(o.name.split('.')[1])<64)],(260,-360,230),(90,-85,48),190,{}),
'electronics':(shelf+[sensor,'REF_Pi4_PCB','REF_Pi_IO16','REF_Pi_IO36','REF_Pi_IO55','REF_Pico','C16_controller_hanger_front','C16_controller_hanger_rear','REF_controller_perfboard'],(290,-430,510),(0,0,110),330,{}),
'orientation':(base+pillars+shelf,(0,0,700),(0,0,100),340,{})}
# Coordinates for the baseline cameras above. Camera-safe-fit below updates
# these normalized seat/pillar callouts along with each image's framing.
base_annotations = {'pillars': [['FL', [0.83530193567276, 0.5976012945175171]], ['FR', [0.5919083952903748, 0.32468459010124207]], ['RL', [0.40809187293052673, 0.8420044183731079]], ['RR', [0.16469836235046387, 0.5690876841545105]]], 'lower': [['1', [0.24199874699115753, 0.5581774115562439]], ['2', [0.5398256778717041, 0.9367045164108276]], ['3', [0.4601743221282959, 0.3972090780735016]], ['4', [0.7580012679100037, 0.7757362127304077]], ['FRONT', [0.7590835094451904, 0.25667619705200195]]], 'shelf': [['1', [0.29069775342941284, 0.5810167193412781]], ['2', [0.5306978225708008, 0.8482613563537598]], ['3', [0.46930238604545593, 0.46914684772491455]], ['4', [0.7093024253845215, 0.7363914847373962]]], 'sensor': [['1', [0.27092111110687256, 0.5415263772010803]], ['2', [0.38360029458999634, 0.6628039479255676]], ['3', [0.47304508090019226, 0.4440711438655853]], ['4', [0.553530216217041, 0.5306980013847351]]]}
annotations = {}
for name,(visible,pos,target,scale,offsets) in views.items():
 for o in bpy.data.objects:
  o.location=orig[o.name];o.hide_render=o.type not in ['LIGHT','CAMERA'] and o.name not in visible
 for k,v in offsets.items(): bpy.data.objects[k].location+=Vector(v)
 cam=s.camera;cam.location=pos;cam.rotation_euler=(Vector(target)-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.type='ORTHO';cam.data.ortho_scale=scale
 # Keep every visible mesh bounding box inside the frame with 5% margins.
 bpy.context.view_layer.update()
 points = [world_to_camera_view(s, cam, o.matrix_world @ Vector(corner))
           for o in bpy.data.objects if o.type == 'MESH' and not o.hide_render
           for corner in o.bound_box]
 factor = max(1.0, max(max(abs(p.x - .5), abs(p.y - .5)) for p in points) * 2 / .9)
 cam.data.ortho_scale *= factor
 bpy.context.view_layer.update()
 if name in base_annotations:
  annotations[name] = [[label, [.5 + (u - .5) / factor, .5 + (v - .5) / factor]]
                       for label, (u, v) in base_annotations[name]]
 s.render.filepath=str(args.output_dir/(name+'.png'));bpy.ops.render.render(write_still=True)
 strip_png_metadata(args.output_dir/(name+'.png'))
(args.output_dir.parent / 'annotations.json').write_text(json.dumps(annotations, indent=2) + '\n')
print('RENDERS COMPLETE')
