"""FIELD / 4: millimetre-native, editable Blender rover prototype.
Run: blender -b --python build_sleek.py -- /absolute/output/root
All dimensions numerical mm. Source contracts and validation accompany exports.
"""
import bpy, bmesh, math, json, os, sys, struct
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
ROOT = sys.argv[sys.argv.index('--')+1] if '--' in sys.argv else os.path.dirname(os.path.dirname(__file__))
for d in ('stl','previews','validation'): os.makedirs(os.path.join(ROOT,d),exist_ok=True)
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
for dat in list(bpy.data.materials): bpy.data.materials.remove(dat)
scene=bpy.context.scene
scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=0.001; scene.unit_settings.length_unit='MILLIMETERS'
PRINT=[]; REFS=[]; manifest={'design':'FIELD / 4','revision':'B sleek two-piece prototype','units':'millimetres','parts':[], 'references':[], 'interfaces':[], 'clearances':[]}

def mat(name,color,metal=0,rough=.45):
    m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=(*color,1);bs.inputs['Metallic'].default_value=metal;bs.inputs['Roughness'].default_value=rough
    return m
ORANGE=mat('Warm ochre PETG',(.87,.255,.055)); DARK=mat('Graphite PETG',(.065,.083,.099)); GRAY=mat('Purchased metal',(.46,.53,.59),.8,.27); RUBBER=mat('Purchased black rubber',(.022,.026,.03),0,.85); GREEN=mat('PCB reference',(.045,.24,.13)); BLUE=mat('Battery reference',(.035,.21,.36)); WHITE=mat('Labels and nylon',(.82,.86,.82)); LENS=mat('Optical lens',(.008,.025,.038),.75,.11)
COLPRINT=bpy.data.collections.new('01 PRINTABLE PARTS — millimetres');scene.collection.children.link(COLPRINT)
COLREF=bpy.data.collections.new('02 PURCHASED REFERENCES — NOT FOR PRINT');scene.collection.children.link(COLREF)
COLRES=bpy.data.collections.new('03 KEEP-OUT VOLUMES — hidden');scene.collection.children.link(COLRES)
COLRENDER=bpy.data.collections.new('04 RENDER STUDIO');scene.collection.children.link(COLRENDER)

def move_col(o,col):
    for c in list(o.users_collection):c.objects.unlink(o)
    col.objects.link(o)

def apply(o):
    bpy.context.view_layer.objects.active=o;o.select_set(True)
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    o.select_set(False)

def box(name,size,center=(0,0,0),bevel=0,material=None):
    bpy.ops.mesh.primitive_cube_add(size=1,location=center);o=bpy.context.object;o.name=name;o.dimensions=size;apply(o)
    if bevel:
        mod=o.modifiers.new('Manufacturing edge radii','BEVEL');mod.width=bevel;mod.segments=3
        bpy.context.view_layer.objects.active=o;bpy.ops.object.modifier_apply(modifier=mod.name)
    if material:o.data.materials.append(material)
    return o

def cylinder(name,r,depth,center=(0,0,0),axis='Z',material=None,vertices=64):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=r,depth=depth,location=center)
    o=bpy.context.object;o.name=name
    if axis=='X':o.rotation_euler[1]=math.pi/2
    elif axis=='Y':o.rotation_euler[0]=math.pi/2
    bpy.context.view_layer.objects.active=o;bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
    if material:o.data.materials.append(material)
    return o

def boolean(a,b,operation='DIFFERENCE'):
    bpy.context.view_layer.objects.active=a
    mod=a.modifiers.new(operation.title(),'BOOLEAN');mod.operation=operation;mod.solver='EXACT';mod.object=b
    bpy.ops.object.modifier_apply(modifier=mod.name);bpy.data.objects.remove(b,do_unlink=True)
    return a

def union(a,b):return boolean(a,b,'UNION')
def hole(o,r,depth,center,axis='Z'):return boolean(o,cylinder('drill',r,depth,center,axis))
def slot(o,width,length,depth,center,axis='Z',along='X'):
    # One analytic capsule extrusion avoids tangent Boolean seams from a box +2cylinders.
    ai='XYZ'.index(axis);ui='XYZ'.index(along);vi=next(i for i in range(3) if i not in [ai,ui]);r=width/2;d=(length-width)/2
    ring=[]
    for side,a0 in [(1,-90),(-1,90)]:
        for j in range(33):
            a=math.radians(a0+j*180/32);ring.append((side*d+r*math.cos(a),r*math.sin(a)))
    verts=[]
    for z in [-depth/2,depth/2]:
        for u,v in ring:
            q=list(center);q[ai]+=z;q[ui]+=u;q[vi]+=v;verts.append(q)
    n=len(ring);faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]
    for i in range(n):j=(i+1)%n;faces.append((i,j,n+j,n+i))
    me=bpy.data.meshes.new('capsule');me.from_pydata(verts,[],faces);me.update();cut=bpy.data.objects.new('capsule',me);scene.collection.objects.link(cut)
    bm=bmesh.new();bm.from_mesh(me);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(me);bm.free()
    return boolean(o,cut)

def mesh_stats(o):
    bm=bmesh.new();bm.from_mesh(o.data);bmesh.ops.triangulate(bm,faces=list(bm.faces));bm.normal_update()
    edge_bad=sum(not e.is_manifold for e in bm.edges);vol=bm.calc_volume(signed=True)
    seen=set();components=0
    for v in bm.verts:
        if v in seen:continue
        components+=1;stack=[v];seen.add(v)
        while stack:
            vv=stack.pop()
            for ed in vv.link_edges:
                q=ed.other_vert(vv)
                if q not in seen:seen.add(q);stack.append(q)
    result={'vertices':len(bm.verts),'triangles':len(bm.faces),'non_manifold_edges':edge_bad,'connected_components':components,'signed_volume_mm3':round(vol,3)};bm.free();return result

def bounds(o):
    pts=[o.matrix_world@Vector(p) for p in o.bound_box]
    return [[round(min(p[i] for p in pts),4) for i in range(3)],[round(max(p[i] for p in pts),4) for i in range(3)]]

def add_part(o,id,qty=1,orientation=(0,0,0),notes='',explode=(0,0,0),material=ORANGE):
    o.name=id;o.data.materials.clear();o.data.materials.append(material);move_col(o,COLPRINT)
    o['PRINTABLE']=True;o['part_id']=id;o['print_quantity']=qty;o['notes']=notes
    PRINT.append({'object':o,'id':id,'qty':qty,'orientation':orientation,'notes':notes,'explode':explode})
    return o

def ref(o,name,source,notes='',material=GRAY,keepout=False):
    o.name='REF_'+name;o.data.materials.clear();o.data.materials.append(material);move_col(o,COLRES if keepout else COLREF)
    o['PRINTABLE']=False;o['source']=source;o['verification_note']=notes
    if keepout:o.hide_render=True;o.hide_viewport=True
    REFS.append(o);return o

def text3d(body,loc,size=6,rotation=(0,0,0),material=WHITE):
    cr=bpy.data.curves.new('label','FONT');cr.body=body;cr.size=size;cr.extrude=.015;cr.align_x='CENTER';ob=bpy.data.objects.new(body,cr);COLREF.objects.link(ob);ob.location=loc;ob.rotation_euler=rotation;ob.data.materials.append(material);return ob

# Final dimensions and construction follow below.
# Revision B: TWO MAIN BODY PRINTS. +X forward, +Y left; all dimensions mm.
FLOOR_Z=25.; FLOOR_TOP=29.5; AXLE_X=90.; AXLE_Z=48.; DECK_Z=108.; DECK_T=4.

def outline(w,d,r,steps=10):
    pts=[]
    for cx,cy,a0 in [(w/2-r,d/2-r,0),(-w/2+r,d/2-r,90),(-w/2+r,-d/2+r,180),(w/2-r,-d/2+r,270)]:
        for j in range(steps+1):
            a=math.radians(a0+j*90/steps);pts.append((cx+r*math.cos(a),cy+r*math.sin(a)))
    return pts

def loft(name,levels):
    verts=[]
    for z,w,d,r in levels:verts.extend([(x,y,z) for x,y in outline(w,d,r)])
    n=len(verts)//len(levels);faces=[tuple(reversed(range(n))),tuple(range(n*(len(levels)-1),n*len(levels)))]
    for k in range(len(levels)-1):
        for i in range(n):j=(i+1)%n;faces.append((k*n+i,k*n+j,(k+1)*n+j,(k+1)*n+i))
    me=bpy.data.meshes.new(name);me.from_pydata(verts,[],faces);me.update();ob=bpy.data.objects.new(name,me);scene.collection.objects.link(ob)
    bm=bmesh.new();bm.from_mesh(me);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(me);bm.free();return ob

def round_prism(name,w,d,r,z0,z1):return loft(name,[(z0,w,d,r),(z1,w,d,r)])
# Lower tub, flat underside,3.2mm walls,4.5mm floor. Large windows isolate serviceable motor mounts.
tub=round_prism('lower tub',236,196,12,25,110)
boolean(tub,round_prism('interior',229.6,189.6,8.8,29.5,120))
for x in [-90,90]:
    for sy in [-1,1]:
        boolean(tub,box('motor service window',(60,40,57),(x,sy*82,58),0)) # Z29.5..86.5
        for dx in [-18,18]:
            for yy in [69,86]:hole(tub,2.2,20,(x+dx,sy*yy,29))
# Integrated shelf seats with wall ribs. Removable internal shelf is not part of the lift-off lid.
for x in [-48,48]:
    for y in [-86,86]:
        union(tub,cylinder('shelf post',5,78.7,(x,y,68.65)))
        union(tub,box('seat to wall rib',(8,8,78.5),(x,math.copysign(92,y),68.75)))
        hole(tub,2.25,100,(x,y,67))
# Located battery seat:116x76 interior, adjustable endstops, padding and two20mm straps.
for y in [-39.5,39.5]:union(tub,box('battery side guide',(114,3,12),(0,y,35.3),.6))
for x in [-35,35]:
    for y in [-45,45]:slot(tub,4,23,20,(x,y,29),along='X')
for x in [-72,72]:
    for y in [-25,25]:slot(tub,4.4,13,20,(x,y,29),along='X')
# Three fuse-holder locations. Shape is an allowance, held with ties rather than a guessed snap fit.
for x,y in [(-26,62.5),(26,62.5),(0,-62.5)]:
    for dx in [-14,14]:
        for dy in [-17,17]:slot(tub,3.5,7,20,(x+dx,y+dy,29),along='X')
# Side power panel opening and fourM3 through bolts.
boolean(tub,box('switch panel opening',(60,16,34),(0,97,89),2))
for x in [-36,36]:
    for z in [77,101]:hole(tub,1.7,16,(x,96,z),'Y')
# Discreet low-tub side ventilation above fuse bays, clear of motor bracket corners.
for sy in [-1]:
    for x in [-42,-14,14,42]:
        for z in [76,86,96]:slot(tub,3.5,19,16,(x,sy*97,z),axis='Y',along='X')
# Locating ledge is integrated;male rim has0.5mm nominal per-side clearance to lid interior.
ledge=round_prism('locating ledge',236,196,12,107,110)
boolean(ledge,round_prism('ledge opening',224.6,184.6,7,105,114));union(tub,ledge)
rim=round_prism('locating rim',228.6,188.6,9,109.5,113.5)
boolean(rim,round_prism('rim opening',224.6,184.6,7,108,115));union(tub,rim)
add_part(tub,'B01_lower_tub',orientation=(0,0,math.pi/2),notes='Main body1of2. Print floor down, rotate90deg onP1Sbed;6mmbrim and painted supports under60mm motorwindow headers. Integrated battery guides and shelf seats. Do not print all parts until fit coupons pass.',explode=(0,0,0),material=DARK)
# Lift-off upper shell, tapered shoulders and planar roof for clean inverted printing.
lid=loft('lift-off upper shell',[(110,236,196,12),(135,236,196,18),(153,216,176,24)])
boolean(lid,loft('shell inner',[(107,229.6,189.6,8.8),(133.5,229.6,189.6,14.8),(150,212,172,21)]))
# Front camera window and left-side Pi ports, both intentionally generous/untethered.
boolean(lid,box('camera sight opening',(35,43,30),(116,0,134),5))
boolean(lid,box('Pi USB ethernet opening',(64,20,33),(35,97,121),0))
for sy in [-1,1]:
    for x in [-64,-38,-12]:
        for z in [122,130,138]:slot(lid,3.2,19,16,(x,sy*97,z),axis='Y',along='X')
for x in [-84,84]:
    for y in [-67,67]:hole(lid,1.7,12,(x,y,151.5))
add_part(lid,'B02_lift_off_shell',orientation=(math.pi,0,math.pi/2),notes='Main body2of2. Print roof outer-face down, rotated90degXY.43mm tapered shell;use support only at side windows if slicer flags. FourM3x12 top screws release lid. No electronics or camera wires attach to lid.',explode=(0,0,120),material=ORANGE)
# Low adjustable battery endstops.
for side,label in [(1,'front'),(-1,'rear')]:
    stop=box('stop foot',(21,82,4),(side*70.5,0,31.5),1)
    union(stop,box('battery stop wall',(4,82,19),(side*60,0,39),1))
    for y in [-25,25]:slot(stop,4.4,11,15,(side*72,y,32),along='X')
    add_part(stop,'B03_battery_stop_'+label,notes='Print foot down. Padding againstbattery;twoM4x20 bottom-up each. Adjuststopspan to actualpack;never hard-clamp.',explode=(side*6,0,25),material=ORANGE)
# Internal service shelf with component-specific seating.
deck=round_prism('internal shelf',212,183,6,108,112)
for x in [-48,48]:
    for y in [-86,86]:
        union(deck,cylinder('raised bolt seat',5,5.3,(x,y,114.35)));hole(deck,2.25,20,(x,y,112))
for x in [-84,84]:
    for y in [-67,67]:hole(deck,1.7,20,(x,y,112))
PI_HOLES=[(10.5,-11.5),(59.5,-11.5),(10.5,46.5),(59.5,46.5)]
for x,y in PI_HOLES:
    union(deck,cylinder('Pi4 seat',3,6.3,(x,y,114.85)));hole(deck,1.45,20,(x,y,113))
for x in [24.5,71.5]:
    for y in [-59.7,-48.3]:union(deck,cylinder('Pico seat',2.6,6.3,(x,y,114.85)));hole(deck,1.2,20,(x,y,113))
for x in [-90,-10]:
    for y in [-52,8]:union(deck,cylinder('perfboard seat',4,6.3,(x,y,114.85)));hole(deck,1.7,20,(x,y,113))
for x in [-61,-19]:
    for y in [40,67]:slot(deck,3.4,10,20,(x,y,112),along='X')
for x in [77,99]:
    for y in [-18,18]:hole(deck,2.2,20,(x,y,112))
# Cable pass-throughs on either side of the shelf, not through PCB footprints.
for x,y in [(-99,30),(96,-40),(0,-76)]:slot(deck,7,20,20,(x,y,112),along='Y' if x else'X')
slot(deck,15,42,20,(0,0,112),along='X')
add_part(deck,'B04_internal_component_shelf',notes='Fixed tolower body. Exact Pi4 58×49mm pattern on6mm high bosses;Pico47×11.4mm. Fabricated90×70mm perfboard uses80×60mm holes. FourM4x100 top-down,specified thinwasher/standardnut stack.',explode=(0,0,45),material=DARK)
# Removable lid pillars with side-loadable captiveM3hexnuts atbothends.
for x in [-84,84]:
    for y in [-67,67]:
        p=box('lid pillar',(12,12,38),(x,y,131),1)
        hole(p,1.7,45,(x,y,131))
        for zz in [116,146]:boolean(p,box('M3 nut side slot',(5.8,11,2.7),(x,y-2,zz)))
        add_part(p,f'B05_lid_pillar_{"F" if x>0 else "R"}{"L" if y>0 else "R"}',notes='Print upright withbrim. SlideM3standardnuts5.5AF×2.4mm into5.8×2.7mm side pockets. LowerM3x12 fromunderdeck;upperM3x12 throughlid. Verifycapturecouponfirst.',explode=(0,0,65),material=DARK)
# Camera fixed tolower shelf;upper shell lifts clear of all wiring.
fork=box('low camera fork',(30,62,5),(88,0,114.5),2)
for y in [-27,27]:
    union(fork,box('camera ear',(22,6,26),(90,y,129),2));hole(fork,2.2,14,(90,y,133),'Y')
for x in [77,99]:
    for y in [-18,18]:hole(fork,2.2,18,(x,y,115))
add_part(fork,'B06_camera_fork',notes='FourM4x20 bottom-up;twoM4x20 pivots. Camera stays withfixedshelf when shelllifts.',explode=(20,0,60),material=DARK)
carrier=box('camera carrier',(8,41.2,29),(88,0,133),1)
for y in [-21,21]:union(carrier,box('pivot tab',(12,5.2,14),(90,y,133),1));hole(carrier,2.2,12,(90,y,133),'Y')
for y in [-10.5,10.5]:
    for z in [126.75,139.25]:slot(carrier,2.5,6,18,(88,y,z),axis='X',along='Z')
add_part(carrier,'B07_camera_carrier',orientation=(0,-math.pi/2,0),notes='CommonrearX84 prints flat.21×12.5 slottedcamera pattern forCamera2/3;3mm nylon spacers;checkactualmodule. Fork gap0.4mm/side. Tilt range must be checked againstshellwindow.',explode=(35,0,75),material=ORANGE)
# Buck tie platform, specified mounting coordinates do not assume unknown PCB holes.
buck=box('buck platform',(64,36,3),(-40,53.5,118.5),2)
for x in [-61,-19]:
    for y in [40,67]:hole(buck,1.7,12,(x,y,118.5))
for x in [-58,-22]:
    for y in [44,63]:slot(buck,2.6,7,12,(x,y,118.5),along='X')
add_part(buck,'B08_buck_platform',notes='Four5mm nylon standoffs;M3x18. Secureboardwithties/nonconductivepad;leavehotcomponents exposed.',explode=(0,0,60),material=DARK)
# Side switch adapter remains blank until the actual toggle cutout is measured.
panel=box('side switch blank',(82,3,40),(0,99.5,89),2)
for x in [-36,36]:
    for z in [77,101]:hole(panel,1.7,14,(x,99.5,z),'Y')
add_part(panel,'B09_side_switch_panel_BLANK',orientation=(math.pi/2,0,0),notes='Measure BlueSea8210 cutout beforeediting/drilling;no guessedcutout. FourM3x12, nutsaccessibleinsidebeforeinternal shelf.',explode=(0,25,15),material=DARK)

# Motor mounts. The 31mm PCD is centered on the GEARBOX,7mm ABOVE the output shaft.
# Five millimetre face intentionally permits M3x8 +0.5mm washer to engage gearbox2.5mm(max3).
def triangular_gusset(name,x0,x1,sy):
    coords=[]
    # Broad foot to face; gussets outside the37mm diameter envelope.
    for x in [x0,x1]:coords.extend([(x,sy*65,29.5),(x,sy*98,29.5),(x,sy*98,81)])
    faces=[(0,2,1),(3,4,5),(0,1,4,3),(1,2,5,4),(2,0,3,5)]
    me=bpy.data.meshes.new(name);me.from_pydata(coords,[],faces);me.update();ob=bpy.data.objects.new(name,me);scene.collection.objects.link(ob)
    bm=bmesh.new();bm.from_mesh(me);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(me);bm.free();return ob
for x,labelx in [(90,'F'),(-90,'R')]:
    for sy,labely in [(1,'L'),(-1,'R')]:
        mount=box('motor foot',(56,34,6),(x,sy*81,32.5),1)
        union(mount,box('motor face',(56,5,54),(x,sy*95.5,56.5),1))
        for a,b in [(-28,-24),(24,28)]:union(mount,triangular_gusset('gusset',x+a,x+b,sy))
        hole(mount,6.4,14,(x,sy*96,48),'Y')
        for a in range(0,360,60):
            rad=math.radians(a);xx=x+15.5*math.cos(rad);zz=55+15.5*math.sin(rad)
            hole(mount,1.65,14,(xx,sy*96,zz),'Y')
        # Unmeasured gearbox face screws: generous diameter and1.2mm recess require coupon test.
        for a in [30,90,150,210,270,330]:
            rad=math.radians(a);xx=x+15.5*math.cos(rad);zz=55+15.5*math.sin(rad)
            hole(mount,3.5,2.6,(xx,sy*92.9,zz),'Y')
        for dx in [-18,18]:
            for yy in [69,86]:hole(mount,2.2,16,(x+dx,sy*yy,32))
        add_part(mount,'B10_motor_mount_'+labelx+labely,notes='Print foot flat; support only horizontal boss aperture if slicer needs it.5mm face,6×3.3mm holes on31mmPCD centeredZ55; boss12.8mm atZ48. Gearbox screw relief pockets7mm×1.2mm are provisional: test coupon before full print.',explode=(0,sy*35,0),material=ORANGE)

# Fit coupons: use the exact interface before committing to large chassis prints.
# Motor coupon duplicates mount face, with the same screwhead relief.
coupon=box('motor face coupon',(45,5,46),(0,0,0),1)
for a in range(0,360,60):
    rad=math.radians(a);hole(coupon,1.65,15,(15.5*math.cos(rad),0,15.5*math.sin(rad)),'Y')
hole(coupon,6.4,15,(0,0,-7),'Y')
for a in [30,90,150,210,270,330]:
    rad=math.radians(a);hole(coupon,3.5,2.6,(15.5*math.cos(rad),-2.6,15.5*math.sin(rad)),'Y')
add_part(coupon,'T01_motor_face_coupon',orientation=(math.pi/2,0,0),notes='Print broad face flat; test6 holes,7mm offset and12.8mm boss hole. The relief-pocket side contacts gearbox. Printed face must seat flat without touching existing faceplate screwheads.',material=DARK)
coupon.hide_render=True;coupon.hide_viewport=True
# Fastener coupon prints numerical bore ladder plus edge-open nut pockets.
coupon=box('fastener coupon',(84,30,6),(0,0,0),1)
for xx,diam in [(-32,2.4),(-20,2.9),(-8,3.3),(4,3.5),(17,4.3),(30,4.5)]:hole(coupon,diam/2,16,(xx,-5,0))
for xx,across in [(-26,5.7),(-7,5.9),(14,7.2),(32,7.4)]:
    # Hex recess4mm deep, open at top; an underlying small bore shows fit without captivation.
    c=cylinder('hex',across/math.sqrt(3),4.2,(xx,8,1),vertices=6);boolean(coupon,c)
    hole(coupon,1.5 if across<6 else 2.0,20,(xx,8,0))
add_part(coupon,'T02_fastener_clearance_coupon',notes='Read left-to-right: bores2.4,2.9,3.3,3.5,4.3,4.5mm. Rear hex flats5.7,5.9,7.2,7.4mm. Test actual filament/printer; choose reaming or compensate source dimensions before large prints.',material=DARK)
coupon.hide_render=True;coupon.hide_viewport=True
# Small four-hole bracket-floor registration coupon, same36×17mm foot pattern.
coupon=box('bracket base coupon',(56,34,4.5),(0,0,0),1)
for x in [-18,18]:
    for y in [-8.5,8.5]:hole(coupon,2.2,15,(x,y,0))
add_part(coupon,'T03_bracket_floor_coupon',notes='36×17mm M4 foot pattern. Test against one motor mount using M4x20 bottom-up, washers and nyloc nuts.',material=DARK)
coupon.hide_render=True;coupon.hide_viewport=True
# A full carrier is already a small inexpensive fit test; duplicate it as coupon without tabs.
coupon=box('camera pattern coupon',(36,30,3),(0,0,0),1)
for x in [-10.5,10.5]:
    for y in [-6.25,6.25]:slot(coupon,2.5,7,15,(x,y,0),along='Y')
add_part(coupon,'T04_camera_pattern_coupon',notes='21×12.5mm nominal pattern,7mm long slots. For Camera2/3 only; check actual module before carrier print.',material=DARK)
coupon.hide_render=True;coupon.hide_viewport=True
# Nut/rim fit coupon is the exact cross-section used by the lid fasteners.
nutc=box('nut slot test',(12,12,15),(0,0,7.5),1);hole(nutc,1.7,20,(0,0,7.5));boolean(nutc,box('nut slot',(5.8,11,2.7),(0,-2,9)))
add_part(nutc,'T05_M3_captive_nut_coupon',notes='Test actual5.5AF×2.4mm M3nut sliding in5.8×2.7mm pocket;M3x12 shouldturnwithoutnutspinning. Tune clearanceifneeded.',material=DARK);nutc.hide_render=True;nutc.hide_viewport=True
rimc=box('rim clearance coupon',(44,15,8),(0,0,4),0)
# Three channels accept a3.2mm wall with0.3/0.4/0.5mm per-sideclearance.
for x,w in [(-14,2.6),(0,2.8),(14,3.0)]:boolean(rimc,box('fit gap',(w,19,5),(x,0,6)))
add_part(rimc,'T06_lid_clearance_coupon',notes='Left to right: channels2.6,2.8,3.0mm for the2mm T07 tongue;0.3,0.4,0.5mm clearance per side. This is a localfit coupon;fullshellwarpstillrequiresdryfit.',material=DARK);rimc.hide_render=True;rimc.hide_viewport=True
tongue=box('rim tongue test',(28,14,3),(0,0,1.5),0);union(tongue,box('rim tongue',(24,2,3.7),(0,0,4.65),0))
add_part(tongue,'T07_lid_tongue_coupon',notes='2mm tongue with3.5mm exposedheight;testinT06 channels. Samewallthicknessasbodylocatingrim.',material=DARK);tongue.hide_render=True;tongue.hide_viewport=True
MOTOR_URL='https://image.dfrobot.com/image/data/FIT0185/FIT0185_Dimension.PNG'
WHEEL_URL='https://www.gobilda.com/hogback-traction-wheel-96mm-diameter-50a-durometer/'
HUB_URL='https://www.gobilda.com/1309-series-sonic-hub-6mm-bore/'
PI_URL='https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.pdf'
BAT_URL='https://www.bioennopower.com/products/12v-6ah-lifepo4-battery-pvc'
for x,lx in [(90,'F'),(-90,'R')]:
    for sy,ly in [(1,'L'),(-1,'R')]:
        tag=lx+ly
        ref(cylinder('gearcase',18.5,28,(x,sy*79,55),'Y'),'motor_gearcase_'+tag,MOTOR_URL,'37mm OD verified;28mm axial segment illustrative,80mm fullrear reserve provisional.')
        ref(cylinder('motor can',17.25,52,(x,sy*39,55),'Y'),'motor_can_'+tag,MOTOR_URL,'34.5mm canOD;bodylength visualwithin80mmreserve.')
        ref(cylinder('boss',6,5.8,(x,sy*95.9,48),'Y'),'motor_boss_'+tag,MOTOR_URL,'12mm boss;5.8±0.3projection;7mm downwards offset.')
        ref(cylinder('shaft',3,15.2,(x,sy*106.4,48),'Y'),'shaft_'+tag,MOTOR_URL,'6mm D shaft rendered round;tip21±0.5fromface.')
        ref(cylinder('hub',16,8,(x,sy*106.5,48),'Y'),'hub_'+tag,HUB_URL,'8mm body;back102.5,front110.5mm.')
        ref(cylinder('pilot',7,2,(x,sy*111.5,48),'Y'),'hub_pilot_'+tag,HUB_URL,'14mm pilot,2mm long.')
        tire=cylinder('rubber',46,24,(x,sy*114.5,48),'Y');boolean(tire,cylinder('rimhole',31,30,(x,sy*114.5,48),'Y'))
        ref(tire,'tire_'+tag,WHEEL_URL,'96mm overall OD withtread,24mmwidth;visualtreadapproximate.',RUBBER)
        rim=cylinder('wheel web',32,4,(x,sy*112.5,48),'Y');hole(rim,7,12,(x,sy*112.5,48),'Y')
        for dx in [-8,8]:
            for dz in [-8,8]:hole(rim,2.2,12,(x+dx,sy*112.5,48+dz),'Y')
        ref(rim,'wheel_web_'+tag,WHEEL_URL,'4mm web,8mm inset facesmotor,16mm squareM4pattern.',DARK)
        for j in range(32):
            a=j*math.tau/32;t=box('tread',(6.8,24,4),(x+46*math.sin(a),sy*114.5,48+46*math.cos(a)),.7,RUBBER);t.rotation_euler[1]=a;move_col(t,COLREF)
        for a in range(0,360,60):
            rad=math.radians(a);ref(cylinder('screw head',2.85,3,(x+15.5*math.cos(rad),sy*100,55+15.5*math.sin(rad)),'Y'),'M3_motor_head_'+tag+str(a),'Generic3mm caphead+0.5mmwasher','Worst-case referenceleaves1mm tiregap;prefer≤2mm buttonheads andspin-check.',DARK)
        for dx in [-8,8]:
            for dz in [-8,8]:ref(cylinder('wheel screw',3.5,3,(x+dx,sy*116.8,48+dz),'Y'),'M4_wheel_'+tag+str(dx)+str(dz),'M4x10+0.8washer','Nominal5.2mm hubengagement.',GRAY)
        ref(cylinder('motor full reserve',18.5,80,(x,sy*53,55),'Y'),'KEEP_OUT_motor80_'+tag,MOTOR_URL,'Actualtotalbodylengthunverified;mustfit≤80mmrear envelope.',keepout=True)
        ref(box('encoder lead route',(24,23,28),(x,sy*13,88)),'KEEP_OUT_encoder_'+tag,MOTOR_URL,'Wiresturnup;reservebottom74top102,deck108.',keepout=True)
ref(box('battery',(112,72,67),(0,0,65),3),'battery',BAT_URL,'Makerdimensionsconflict;reserve116×76×70,softpaddingandstraps;measureactualpack.',BLUE)
ref(box('battery reserve',(116,76,70),(0,0,64.5)),'KEEP_OUT_battery',BAT_URL,'Top99.5;deck108leaves8.5mm nominal.',keepout=True)
ref(box('vertical power connectors',(18,20,57),(-65,0,78)),'KEEP_OUT_PP30_vertical',BAT_URL,'ReserveX-74..-56,Y±10,Z49.5..106.5. OrientpairedPP30connectorverticallyin26mmcentralmotorcorridor;measurecableexit.',keepout=True)
for x in [-35,35]:
    for y in [-37.1,37.1]:ref(box('strap',(20,1.5,70),(x,y,65)),'strap_side'+str(x)+str(y),'20×400mm hook-and-loop','Notprinted.',DARK)
    ref(box('strap',(20,76,1.5),(x,0,99.7)),'strap_top'+str(x),'20×400mm hook-and-loop','Do not overtighten.',DARK)
for x,y,lab in [(-26,62.5,'main'),(26,62.5,'motor'),(0,-62.5,'logic')]:ref(box('fuse',(45,30,25),(x,y,44)),'fuse_'+lab,'Powerwerx IFH12/18 electricalselection','Provisional45×30×25 envelope;2mm softpadandties;checklead/capaccess.',DARK)
ref(box('Pi4',(56,85,1.6),(35,27.5,118.8)),'Pi4_PCB',PI_URL,'Rotated85mm axisalong+Y. Boardoriginworld(7,-15).Mounts58×49mm.',GREEN)
for cx,sz in [(16,(16,18,13)),(36,(16,18,15)),(55,(14,21,15))]:ref(box('Pi I/O',sz,(cx,68,126)),'Pi_IO'+str(cx),PI_URL,'Simplifiedconnectors;routeUSB/Ethernetthroughleftside opening.')
ref(box('Pi cooling',(30,30,13),(35,14,126)),'Pi_lowprofile_cooling',PI_URL,'Illustrative13mm cooling allowance;actualcooler mustbechecked.',DARK)
ref(box('Pi USB plugs',(56,25,20),(35,92,126)),'KEEP_OUT_Pi_ports',PI_URL,'25mm outwardplugspace;leftside openingandwheelheightclear.',keepout=True)
ref(box('Pi right-angle USB-C',(17,30,10),(71.5,-10,123)),'KEEP_OUT_Pi_power',PI_URL,'Requires≤17mm projectionfromboardedgeX63,leadturnstoward−Y. Straightplugmayhitcamera.',keepout=True)
ref(box('Pico',(51,21,1.6),(48,-54,118.8)),'Pico','https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf','Holes47×11.4.',GREEN)
ref(box('perfboard',(90,70,1.6),(-50,-22,118.8)),'controller_perfboard','Fabricationinterface','Cut90×70 FR4;drill80×60patternafterelectrical layout.',GREEN)
for x in [-73,-29]:
    for y in [-39,-5]:ref(box('driver',(15.24,17.78,9),(x,y,124)),'driver'+str(x)+str(y),'https://www.pololu.com/product/4035','Assemblyheight9mm allowance.',GREEN)
ref(box('buck',(43.5,20.3,5.5),(-40,53.5,123.75)),'DFR0831','https://www.dfrobot.com/product-2240.html','43.5×20.3×5.5 board;mountingholespatterndeliberatelyunassumed.',GREEN)
ref(box('buck connectors',(60,35,20),(-40,53.5,130)),'KEEP_OUT_buck','https://www.dfrobot.com/product-2240.html','Connectorallowance.',keepout=True)
ref(box('camera board',(1.6,25,24),(96,0,133)),'PiCamera_example','https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf','Camera 3 example; verify the actual camera module.',GREEN)
ref(cylinder('lens',6,11,(102.3,0,133),'X'),'Camera_lens','https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf','LensmaxX107.8,fullyrecessedbehindlidinnerfront≥114.8onremovalsweep.',LENS)
ref(box('master switch clearance',(40,50,30),(0,70,89)),'KEEP_OUT_switch','https://www.bluesea.com/products/8210','Provisionalbody/crimpallowance;blankplatecutoutmustbemeasured.',keepout=True)
ref(box('future sensor',(80,80,70),(-50,0,188)),'KEEP_OUT_optional_sensor','Designallowance','Flat80×80 rearroofspace;nospecificLiDARfitormountpatternclaimed.',keepout=True)
text3d('FIELD / 4',(-47,0,153.12),8,material=DARK)
text3d('LIFT TO SERVICE',(-47,-14,153.12),2.7,material=DARK)
# Visiblelidfasteners communicate realserviceaccess.
for x in [-84,84]:
    for y in [-67,67]:ref(cylinder('lid screw',2.9,1.8,(x,y,154)),'M3_lid_head'+str(x)+str(y),'M3x12 buttonhead','Remove4toservicelid.',DARK)

def clean_print_mesh(ob):
    # Preserve coplanar edge segmentation until after limited dissolve;neverdropT-junctiontriangles blindly.
    bm=bmesh.new();bm.from_mesh(ob.data)
    bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-5)
    bmesh.ops.dissolve_limit(bm,angle_limit=1e-5,verts=list(bm.verts),edges=list(bm.edges),use_dissolve_boundaries=False,delimit={'NORMAL'})
    bmesh.ops.triangulate(bm,faces=list(bm.faces));bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=1e-4)
    bmesh.ops.triangulate(bm,faces=list(bm.faces));bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(ob.data);bm.free();ob.data.update()
for item in PRINT:clean_print_mesh(item['object'])

def write_stl(ob,path,rot):
    bm=bmesh.new();bm.from_mesh(ob.data);bmesh.ops.triangulate(bm,faces=list(bm.faces));bm.normal_update()
    rotation=Matrix.Rotation(rot[2],4,'Z')@Matrix.Rotation(rot[1],4,'Y')@Matrix.Rotation(rot[0],4,'X')
    transform=rotation@ob.matrix_world
    coords={v:transform@v.co for v in bm.verts}
    mins=Vector([min(p[i] for p in coords.values()) for i in range(3)]);maxs=Vector([max(p[i] for p in coords.values()) for i in range(3)])
    with open(path,'wb') as f:
        f.write(b'FIELD4 STL millimetres. Individual printable prototype.'.ljust(80,b'\0'));f.write(struct.pack('<I',len(bm.faces)))
        for face in bm.faces:
            pts=[coords[v]-mins for v in face.verts];n=(pts[1]-pts[0]).cross(pts[2]-pts[0]).normalized()
            f.write(struct.pack('<12fH',*n,*pts[0],*pts[1],*pts[2],0))
    bm.free();return [round(q,3) for q in maxs-mins]
failed=[]
for e in PRINT:
    o=e['object'];stats=mesh_stats(o);dims=write_stl(o,os.path.join(ROOT,'stl',e['id']+'.stl'),e['orientation'])
    manifest['parts'].append({'id':e['id'],'quantity':1,'file':'stl/'+e['id']+'.stl','assembly_location':list(o.location),'assembly_matrix':[list(row) for row in o.matrix_world],'assembly_bounds_mm':bounds(o),'print_rotation_radians':e['orientation'],'print_bounds_mm':dims,'brim_envelope_6mm':[dims[0]+12,dims[1]+12,dims[2]],'notes':e['notes'],**stats})
    if stats['non_manifold_edges'] or stats['connected_components']!=1 or stats['signed_volume_mm3']<=0:failed.append(e['id'])
    print('PART',e['id'],json.dumps(stats),dims,flush=True)
for o in REFS:manifest['references'].append({'id':o.name,'bounds_mm':bounds(o),'source':o.get('source'),'verification_note':o.get('verification_note'),'is_keepout':o.hide_render})
manifest['dimensions_mm']={'main_body_length':236,'main_body_width':196,'body_floor_z':25,'lower_tub_seam_z':110,'roof_z':153,'wheelbase':180,'track_center_to_center':229,'overall_tire_width':253,'overall_tire_length':276,'wheel_diameter':96,'tire_width':24,'axle_z':48,'battery_pocket':[116,76,70],'Pi_board':[85,56],'Pi_hole_pitch':[58,49],'upper_shell_locating_clearance_each_side':0.5,'minimum_ground_clearance_fasteners_approx':17.5,'nominal_floor_ground_clearance':25}
manifest['interfaces']=[
 {'id':'motors','centers_x':[-90,90],'gearbox_center_z':55,'output_axis_z':48,'offset_z':-7,'face_abs_y':93,'face_thickness':5,'bolt_circle_diameter':31,'bolt_angles_deg':[0,60,120,180,240,300],'printed_m3_bores':3.3,'boss_aperture':12.8,'factory_head_relief_angles_deg':[30,90,150,210,270,330],'factory_relief_diameter':7,'factory_relief_depth':1.2,'fasteners':'6 M3x8 with0.5mmwashers per motor;max3mm thread engagement'},
 {'id':'motorfeet','hole_pitch':[36,17],'clearance':4.4,'fasteners':'4 M4x20 bottom-up per bracket;washersandnylocs'},
 {'id':'wheelhubs','hub_back_abs_y':102.5,'contact_abs_y':110.5,'pilot_end_abs_y':112.5,'tire_inner_abs_y':102.5,'tire_outer_abs_y':126.5,'fasteners':'4 M4x10 +0.8mmwashers per wheel;5.2mm nominal engagement'},
 {'id':'Pi4','board_origin_xy':[7,-15],'board_axis_85':'world+Y','board_axis_56':'world+X','hole_centers_xy':PI_HOLES,'hole_diameter':2.9,'boss_height':6,'boss_diameter':6,'fasteners':'4 M2.5x16 withnylonwashers andnuts'},
 {'id':'Pico','hole_pitch':[47,11.4],'hole_diameter':2.4,'boss_diameter':5.2,'fasteners':'4 M2x16'},
 {'id':'camera','nominal_pattern':[21,12.5],'slot_width':2.5,'slot_length':6,'plate_thickness':8,'pivot_y_gap':0.4,'fasteners':'4 M2x16 with3mm nylonspacers;2 M4x20 frictionpivots'},
 {'id':'lid','pillar_xy':[[-84,-67],[-84,67],[84,-67],[84,67]],'pillar_size':[12,12,38],'nut_pocket':[5.8,11,2.7],'fasteners':'4 M3x12 top plus4 M3x12 belowdeck;8 M3standardhexnuts'},
 {'id':'shelf','post_xy':[[-48,-86],[-48,86],[48,-86],[48,86]],'post_top_z':108,'raised_seat_top_z':117,'fasteners':'4 M4x100 top-down with2×0.5mmflatwashers,1mm splitwasher,3.2mm standardnut;measure actualstack'},
 {'id':'optional_lidar','reserve':[80,80],'center_xy':[-50,0],'status':'Genericflatspaceonly,noholesorspecificdevicefit'},
 {'id':'master_switch','blank_panel':'B09','cutout':'Not yetdefined;measure actualBlueSea8210','clearance_box':[40,50,30]}
]
manifest['clearances']=[{'pair':'tire inner to body/motor face','nominal_mm':4.5},{'pair':'tire to3mm motorhead +0.5washer','nominal_mm':1.0,'note':'Preferlowprofile≤2mmhead;side-loadspincheckmandatory'},{'pair':'shaftmin tip pastpilot','nominal_mm':1.0},{'pair':'motor underside tofoot','nominal_mm':1.0},{'pair':'battery reserved top to shelf','nominal_mm':8.5},{'pair':'motor rear ends acrossY','nominal_mm':26},{'pair':'camera tabs to fork eachside','nominal_mm':0.4},{'pair':'lid collar eachside','nominal_mm':0.5},{'pair':'shelf width to rim opening eachside','nominal_mm':0.8},{'pair':'camera lens tovertical lid-removal base innerfront','nominal_mm':7.0,'note':'Recessedfixedcamera;actualcameraandtilt must bechecked'}]
manifest['validation']={'failed_parts':failed,'mesh_check':'Coplanar edges dissolved beforetriangulation;vertices welded0.00001mm and degenerate edges dissolved0.0001mm;watertightnessandvolume checkedpermesh.IndependentactualSTLauditseparate.','self_intersections':'No formal all-triangle self-intersection certification;independent assemblyintersectionsrequired.','physical':'No physicalprint,assembly,strength,traction,coolingorwaterprotection certification.','P1S':'Mainparts exported196×236mmXY;6mmbrim gives208×248mm, centeredclearof18×28mmstockexclusion. UseactualP1Sprofile;neverdisableexcludedzones.'}
with open(os.path.join(ROOT,'validation','manifest.json'),'w') as f:json.dump(manifest,f,indent=2)
# Render setup. Low-sample first preview;finalqualitycanbe rendered from the savededitableblend.
world=scene.world or bpy.data.worlds.new('World');scene.world=world;world.use_nodes=True;world.node_tree.nodes['Background'].inputs[0].default_value=(.045,.056,.069,1);world.node_tree.nodes['Background'].inputs[1].default_value=.4
studio=box('studio',(1600,1600,3),(0,0,-5),0,mat('Studio',(.08,.095,.115),0,.6));move_col(studio,COLRENDER)
for name,loc,power,size in [('Key',(300,-400,600),5500000,450),('Fill',(-400,-150,350),3250000,400),('Rim',(0,450,480),5500000,300)]:
    dat=bpy.data.lights.new(name,'AREA');dat.energy=power;dat.shape='DISK';dat.size=size;o=bpy.data.objects.new(name,dat);COLRENDER.objects.link(o);o.location=loc;o.rotation_euler=(Vector((0,0,70))-o.location).to_track_quat('-Z','Y').to_euler()
camdat=bpy.data.cameras.new('Camera');cam=bpy.data.objects.new('Camera',camdat);COLRENDER.objects.link(cam);scene.camera=cam;camdat.type='ORTHO';camdat.ortho_scale=440;camdat.clip_start=.1;camdat.clip_end=5000
cam.location=(410,-520,350);cam.rotation_euler=(Vector((0,0,72))-cam.location).to_track_quat('-Z','Y').to_euler()
scene.render.engine='CYCLES';scene.cycles.samples=int(os.environ.get('RENDER_SAMPLES','24'));scene.cycles.use_denoising=False
scene.render.resolution_x=1200;scene.render.resolution_y=900;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.view_settings.view_transform='AgX'
for area in bpy.context.screen.areas:
    if area.type=='VIEW_3D':area.spaces.active.region_3d.view_distance=430;area.spaces.active.region_3d.view_location=(0,0,75);area.spaces.active.clip_end=10000
scene.render.filepath=os.path.join(ROOT,'previews','sleek_assembled.png');bpy.ops.wm.save_as_mainfile(filepath=os.path.join(ROOT,'FIELD4_sleek_editable.blend'))
bpy.ops.render.render(write_still=True)
# Liftedlid preview with fixed internals, demonstrating un-tetheredserviceaccess.
original=[]
for e in PRINT:
    ob=e['object'];original.append((ob,ob.location.copy()));ob.location+=Vector(e['explode'])
for ob in COLREF.objects:
    if ob.type=='FONT' or ob.name.startswith('REF_M3_lid_head'):original.append((ob,ob.location.copy()));ob.location.z+=120
camdat.ortho_scale=540;cam.location=(440,-570,430);cam.rotation_euler=(Vector((0,0,122))-cam.location).to_track_quat('-Z','Y').to_euler()
scene.render.filepath=os.path.join(ROOT,'previews','sleek_exploded.png');bpy.ops.render.render(write_still=True)
for ob,loc in original:ob.location=loc
# Openbody view shows seating;roof removed,not an additionalassembled tier.
lid.hide_render=True
for ob in COLREF.objects:
    if ob.type=='FONT' or ob.name.startswith('REF_M3_lid_head'):ob.hide_render=True
camdat.ortho_scale=370;cam.location=(260,-330,550);cam.rotation_euler=(Vector((0,0,80))-cam.location).to_track_quat('-Z','Y').to_euler()
scene.render.filepath=os.path.join(ROOT,'previews','sleek_internal_seats.png');bpy.ops.render.render(write_still=True)
lid.hide_render=False
for ob in COLREF.objects:
    if ob.type=='FONT' or ob.name.startswith('REF_M3_lid_head'):ob.hide_render=False
camdat.ortho_scale=440;cam.location=(410,-520,350);cam.rotation_euler=(Vector((0,0,72))-cam.location).to_track_quat('-Z','Y').to_euler();scene.render.filepath=os.path.join(ROOT,'previews','sleek_assembled.png')
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(ROOT,'FIELD4_sleek_editable.blend'))
print('FINISHED',ROOT,failed,flush=True)
