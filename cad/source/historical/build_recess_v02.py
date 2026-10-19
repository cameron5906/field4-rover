"""FIELD4 recessed A1M8 prototype v0.2. Print coupons before full parts.
Run blender -b --python source/build_recess_v02.py -- ORIGINAL_BLEND OUTPUT_DIR
Numerical units mm. Actual sensor revision remains a physical fit gate.
Exact sensor STL and drawing drive coordinates; display mesh is decimated only.
"""
import bpy,bmesh,math,os,sys,json,struct,hashlib
from mathutils import Vector,Matrix
args=sys.argv[sys.argv.index('--')+1:]; original,out=args
bpy.ops.wm.open_mainfile(filepath=original)
scene=bpy.context.scene
for d in ['previews','validation','stl']:os.makedirs(os.path.join(out,d),exist_ok=True)
# Reuse the original modeling utilities, without running its scene construction.
src=open(os.path.join(os.path.dirname(__file__),'build_original_reference.py')).read()
PRINT=[]; REFS=[]
COLPRINT=bpy.data.collections.get('01 PRINTABLE PARTS — millimetres'); COLREF=bpy.data.collections.get('02 PURCHASED REFERENCES — NOT FOR PRINT');COLRES=bpy.data.collections.get('03 KEEP-OUT VOLUMES — hidden');COLRENDER=bpy.data.collections.get('04 RENDER STUDIO')
exec(src[src.index('def mat'):src.index('ORANGE=mat')])
ORANGE=bpy.data.materials['Warm ochre PETG'];DARK=bpy.data.materials['Graphite PETG'];GRAY=bpy.data.materials['Purchased metal'];WHITE=bpy.data.materials.get('Labels and nylon') or mat('Labels and nylon',(.82,.86,.82));GREEN=bpy.data.materials['PCB reference']
exec(src[src.index('def move_col'):src.index('# Final dimensions')])
exec(src[src.index('def clean_print_mesh'):src.index('for item in PRINT:clean_print_mesh')])
exec(src[src.index('def write_stl'):src.index('failed=[]')])
# Verified drawing/STL parameters. Feet datum0, rotorbottom35.5, top55.
CX,CY=-62.,0.; FEET_Z=119.; FOOT_REL=[(-28,-28),(-28,28),(42,-20),(42,20)]
profile=json.load(open(os.path.join(os.path.dirname(__file__),'aperture_profile.json')))
def prism(name,xy,z0,z1):
 verts=[(x,y,z) for z in [z0,z1] for x,y in xy];n=len(xy)
 faces=[tuple(reversed(range(n))),tuple(range(n,n*2))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 me=bpy.data.meshes.new(name);me.from_pydata(verts,[],faces);me.update();ob=bpy.data.objects.new(name,me);scene.collection.objects.link(ob)
 bm=bmesh.new();bm.from_mesh(me);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(me);bm.free();return ob
lid=bpy.data.objects['B02_lift_off_shell'];lid.name='C02_recessed_A1M8_lid_PROTOTYPE'
# Source STL uses Z as width; invert to worldY. The profile is symmetric.
cutxy=[(CX+x,CY-z) for x,z in profile['vertices']]
boolean(lid,prism('sensor lift aperture',cutxy,140,180))
for ob in list(bpy.data.objects):
 if ob.name.startswith('REF_KEEP_OUT_optional_sensor') or ob.type=='FONT':bpy.data.objects.remove(ob,do_unlink=True)
controller=bpy.data.objects['REF_controller_perfboard'];controller.rotation_euler[0]=math.pi/2;controller.location=(0,-74,94)
for ob in bpy.data.objects:
 if ob.name.startswith('REF_driver'):
  dx=ob.location.x+50;dy=ob.location.y+22;ob.location=(dx,-79.2,94+dy);ob.rotation_euler[0]=math.pi/2
 if ob.name in ['B08_buck_platform','REF_DFR0831','REF_KEEP_OUT_buck']:ob.location.y+=11.5
shelf=bpy.data.objects['B04_internal_component_shelf'];shelf.name='C04_A1M8_rearranged_shelf_PROTOTYPE'
for x in [-90,-10]:
 for y in [-52,8]:boolean(shelf,box('remove old board boss',(10,10,8),(x,y,116.01)))
boolean(shelf,box('controller slot',(94,4.6,12),(0,-74,110)))
for x in [-61,-19]:
 for y in [51.5,78.5]:slot(shelf,3.4,10,20,(x,y,112),along='X')
for dx,dy in FOOT_REL:
 x,y=CX+dx,CY+dy
 union(shelf,cylinder('lidar foot boss',5,7.2,(x,y,115.4)))
 hole(shelf,1.45,20,(x,y,113))
# Hangers are separate flat-extruded parts, bolted to the existing-height shelf.
hangers=[]
for x,label in [(-40,'rear'),(40,'front')]:
 rail=box('controller rail',(24,6,71),(x,-69,94.5),0)
 union(rail,box('shelf flange',(24,16,5),(x,-62,114.5),0))
 for zz in [64,124]:
  union(rail,cylinder('board spacer',3,1.4,(x,-72.5,zz),'Y'))
  hole(rail,1.7,18,(x,-71,zz),'Y')
 for dx in [-5,5]:
  hole(rail,1.7,18,(x+dx,-60,114))
  # Flat-bottom2.2mm counterbore lets capheads sit below nearby PCB.
  hole(rail,3.1,2.4,(x+dx,-60,115.9))
  hole(shelf,1.7,12,(x+dx,-60,110))
 rail.name='C16_controller_hanger_'+label;rail.data.materials.append(DARK);move_col(rail,COLPRINT);hangers.append(rail)
# External SLAMTEC R6 reference: transformation checked against the drawing.
stl=os.path.join(out,'reference','RPLIDAR A1M8 kit.STL')
cache=os.path.join(out,'reference','RPLIDAR_A1M8_display_only.blend')
if os.path.exists(cache):
 with bpy.data.libraries.load(cache,link=False) as (source,target):target.objects=source.objects
 sensor=target.objects[0];scene.collection.objects.link(sensor)
else:
 bpy.ops.wm.stl_import(filepath=stl);sensor=bpy.context.object
 mod=sensor.modifiers.new('Display decimation only','DECIMATE');mod.ratio=.045
 bpy.context.view_layer.objects.active=sensor;bpy.ops.object.modifier_apply(modifier=mod.name)
sensor.name='REF_SLAMTEC_A1M8_R6_actual_mesh_RENDER_ONLY'
sensor.rotation_euler[0]=math.pi/2;sensor.location=(CX-37.10319,CY+36.53911,FEET_Z-.07303969)
sensor.data.materials.append(DARK);move_col(sensor,COLREF);sensor['PRINTABLE']=False
sensor['source']='External RPLIDAR A1M8 kit.STL. Header R6; reference not redistributed in this public edition. Display mesh decimated4.5%; exactmesh used for aperture calculation.'
sensor['physical_gate']='Actual A1M8 revision must be checked; pass the mounting coupon before the full build.'
# Add visible removable fasteners only as references.
for dx,dy in FOOT_REL:ref(cylinder('lidar screw head',2.3,1.8,(CX+dx,CY+dy,106.6)),'M2_5_lidar_screw'+str(dx)+str(dy),'M2.5x16 +0.5mm washer','4.5mm nominal engagement in6mm blind thread. Verify screw/washer/printed thickness.',GRAY)
for x in [-40,40]:
 for zz in [64,124]:ref(cylinder('controller bolt head',2.9,1.7,(x,-75.7,zz),'Y'),'controller_bolt'+str(x)+str(zz),'M3x16 +washers+nuts','RearPCB face; confirm components and tool access.',GRAY)
# True four-hole coupon, in same rotor-axis coordinate system as drawing.
outer=[(-34,-34),(48,-26),(48,26),(-34,34)];inner=[(-22,-22),(36,-15),(36,15),(-22,22)]
coupon=prism('mount coupon',outer,0,3);boolean(coupon,prism('open middle',inner,-2,5))
for dx,dy in FOOT_REL:hole(coupon,1.45,8,(dx,dy,1.5))
coupon.name='T09_A1M8_four_foot_coupon';coupon.data.materials.append(ORANGE);move_col(coupon,COLPRINT);coupon.hide_render=True
# Matching upper-aperture ring is a separate gauge, not a fitted cover.
xy=profile['vertices'];outer2=[(x*1.11,z*1.11) for x,z in xy]
ring=prism('aperture ring',outer2,0,3);boolean(ring,prism('aperture inner',xy,-2,5));ring.name='T10_A1M8_aperture_coupon';ring.data.materials.append(ORANGE);move_col(ring,COLPRINT);ring.hide_render=True
outputs=[(lid,(math.pi,0,math.pi/2)),(shelf,(0,0,math.pi/2))]+[(h,(0,math.pi/2,0)) for h in hangers]+[(coupon,(0,0,0)),(ring,(0,0,0))]
checks={}
for ob,rot in outputs:
 clean_print_mesh(ob);checks[ob.name]=mesh_stats(ob);checks[ob.name]['print_bounds_mm']=write_stl(ob,os.path.join(out,'stl',ob.name+'.stl'),rot)
 ob['PRINTABLE']=True;ob['release_status']='Prototype. Print and pass small coupons first; verify physical electronics fit.'
 if checks[ob.name]['non_manifold_edges'] or checks[ob.name]['connected_components']!=1 or checks[ob.name]['signed_volume_mm3']<=0:raise RuntimeError('Invalid mesh '+ob.name)
checks['design']={'units':'mm','rotor_axis_xy':[CX,CY],'feet_z':FEET_Z,'head_lower_line_z':FEET_Z+35.5,'roof_z':153,'minimum_nominal_full_head_above_roof':1.5,'foot_holes_xy':[[CX+x,CY+y] for x,y in FOOT_REL],'hole_clearance_diameter':2.9,'sensor_screw':'M2.5x16,0.5washer,11mm printedstack,4.5 nominal engagement in6mm threaded depth','aperture_method':profile['basis'],'controller_board_center':[0,-74,94],'buck_shift_y':11.5}
checks['gates']=['Owned unit revision unconfirmed; mounting and aperture coupons first.','Actual PCB, fuse and cable envelopes require dry-fit.','No thermal/load/vibration certification.','Slicer required forhangercounterbores andshelfunderside screwaccess.']
with open(os.path.join(out,'validation','mesh_validation.json'),'w') as f:json.dump(checks,f,indent=2)
scene.render.engine='CYCLES';scene.cycles.samples=48;scene.cycles.use_denoising=False
scene.render.resolution_x=1400;scene.render.resolution_y=1050;scene.render.resolution_percentage=100
label=text3d('FIELD / 4  -  A1',(30,-32,153.12),5,material=DARK)
cam=scene.camera;cam.data.ortho_scale=410;cam.location=(360,-460,390);cam.rotation_euler=(Vector((-5,0,92))-cam.location).to_track_quat('-Z','Y').to_euler()
scene.render.filepath=os.path.join(out,'previews','recessed_A1M8_actual_sensor_assembled.png')
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out,'FIELD4_Recessed_A1M8_v0.2_editable.blend'));bpy.ops.render.render(write_still=True)
lid.hide_render=True;label.hide_render=True
for ob in bpy.data.objects:
 if ob.name.startswith('REF_M3_lid_head'):ob.hide_render=True
cam.data.ortho_scale=350;cam.location=(260,-350,450);cam.rotation_euler=(Vector((-10,0,110))-cam.location).to_track_quat('-Z','Y').to_euler();scene.render.filepath=os.path.join(out,'previews','historical_reference_open.png');bpy.ops.render.render(write_still=True)
lid.hide_render=False;label.hide_render=False
for ob in bpy.data.objects:
 if ob.name.startswith('REF_M3_lid_head'):ob.hide_render=False
cam.data.ortho_scale=410;cam.location=(360,-460,390);cam.rotation_euler=(Vector((-5,0,92))-cam.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out,'FIELD4_Recessed_A1M8_v0.2_editable.blend'));print('DONE',checks)
