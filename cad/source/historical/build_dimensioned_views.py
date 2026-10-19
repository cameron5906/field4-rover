#!/usr/bin/env python3
"""Generate separate FIELD / 4 engineering annotation sheets from the CAD manifest.

No Blender geometry or STL is touched. PDF/SVG use vector dimensions. All model
values are millimetres, sheet coordinates are physical millimetres on A3.
Run --output-root /tmp/field4-dimensions while geometry is being revised.
Only use --final after the design owner confirms final geometry.
"""
import argparse, ast, hashlib, html, json, math, os, subprocess
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.pdfbase.pdfmetrics import stringWidth, registerFont
from reportlab.pdfbase.ttfonts import TTFont
registerFont(TTFont("F4Sans", "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"))
registerFont(TTFont("F4Sans-Bold", "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"))

W,H=420,297
INK='#172c39'; MUTED='#526874'; DIM='#146286'; LIGHT='#e8edf0'; ORANGE='#b75218'; OPALE='#fff2e7'; WHITE='#ffffff'; GRID='#c7d2d8'; GREEN='#edf3ef'

def num(v):
    return str(int(round(v))) if abs(v-round(v))<1e-6 else ('%.3f'%v).rstrip('0').rstrip('.')

def rgb(h):return tuple(int(h[i:i+2],16)/255 for i in (1,3,5))

class Sheet:
    def __init__(self,pdf,title,index):
        self.c=pdf; self.svg=[];self.title=title;self.index=index
        self.rect(0,0,W,H,fill=WHITE,stroke=None)
    def style(self,stroke,width,fill,dash):
        c=self.c;c.setLineWidth(width*mm)
        if stroke:c.setStrokeColorRGB(*rgb(stroke))
        if fill:c.setFillColorRGB(*rgb(fill))
        c.setDash([v*mm for v in dash] if dash else [])
        return f'stroke="{stroke or "none"}" stroke-width="{width}" fill="{fill or "none"}"'+(f' stroke-dasharray="{" ".join(map(str,dash))}"' if dash else '')
    def line(self,x1,y1,x2,y2,stroke=INK,width=.3,dash=None):
        st=self.style(stroke,width,None,dash);self.c.line(x1*mm,(H-y1)*mm,x2*mm,(H-y2)*mm)
        self.svg.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" {st}/>')
    def poly(self,pts,stroke=INK,width=.3,fill=None,close=True,dash=None):
        st=self.style(stroke,width,fill,dash);p=self.c.beginPath();p.moveTo(pts[0][0]*mm,(H-pts[0][1])*mm)
        for x,y in pts[1:]:p.lineTo(x*mm,(H-y)*mm)
        if close:p.close()
        self.c.drawPath(p,stroke=int(bool(stroke)),fill=int(bool(fill)))
        tag='polygon' if close else 'polyline';self.svg.append(f'<{tag} points="'+ ' '.join(f'{x},{y}'for x,y in pts)+f'" {st}/>')
    def rect(self,x,y,w,h,stroke=INK,width=.3,fill=None,r=0,dash=None):
        st=self.style(stroke,width,fill,dash)
        if r:self.c.roundRect(x*mm,(H-y-h)*mm,w*mm,h*mm,r*mm,stroke=int(bool(stroke)),fill=int(bool(fill)))
        else:self.c.rect(x*mm,(H-y-h)*mm,w*mm,h*mm,stroke=int(bool(stroke)),fill=int(bool(fill)))
        self.svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" {st}/>')
    def circle(self,x,y,r,stroke=INK,width=.3,fill=None,dash=None):
        st=self.style(stroke,width,fill,dash);self.c.circle(x*mm,(H-y)*mm,r*mm,stroke=int(bool(stroke)),fill=int(bool(fill)))
        self.svg.append(f'<circle cx="{x}" cy="{y}" r="{r}" {st}/>')
    def text(self,x,y,s,size=9,color=INK,font='F4Sans',anchor='start',rotate=0):
        c=self.c;c.saveState();c.setFillColorRGB(*rgb(color));c.setFont(font,size);c.translate(x*mm,(H-y)*mm);c.rotate(-rotate)
        {'start':c.drawString,'middle':c.drawCentredString,'end':c.drawRightString}[anchor](0,0,s);c.restoreState()
        weight='bold' if 'Bold'in font else 'normal';s=html.escape(s)
        transform=f' transform="rotate({rotate} {x} {y})"' if rotate else ''
        self.svg.append(f'<text x="{x}" y="{y}" font-family="Arial, Helvetica, sans-serif" font-size="{size/mm}" font-weight="{weight}" fill="{color}" text-anchor="{anchor}"{transform}>{s}</text>')
    def multiline(self,x,y,rows,size=9,leading=4.5,color=INK,font='F4Sans'):
        for i,row in enumerate(rows):self.text(x,y+i*leading,row,size,color,font)
    def label(self,x,y,s,size=9,rotate=0,color=DIM):
        # White text masks keep dimension lines clear and fully legible.
        w=stringWidth(s,'F4Sans',size)/mm+3;h=size/mm+1.7
        if rotate:self.rect(x-h/2,y-w/2,h,w,stroke=None,fill=WHITE)
        else:self.rect(x-w/2,y-h+1,w,h,stroke=None,fill=WHITE)
        self.text(x,y,s,size,color,anchor='middle',rotate=rotate)
    def arrow(self,x,y,dx,dy,color=DIM,size=1.8):
        n=math.hypot(dx,dy);ux,uy=dx/n,dy/n;v=(-uy,ux)
        self.poly([(x,y),(x+ux*size+v[0]*.55,y+uy*size+v[1]*.55),(x+ux*size-v[0]*.55,y+uy*size-v[1]*.55)],stroke=color,width=.15,fill=color)
    def dimh(self,x1,x2,feature_y,dim_y,label,size=9):
        sign=1 if dim_y>feature_y else -1
        for x in (x1,x2):self.line(x,feature_y+sign*1,x,dim_y+sign*2,DIM,.19)
        self.line(x1,dim_y,x2,dim_y,DIM,.22);self.arrow(x1,dim_y,1,0);self.arrow(x2,dim_y,-1,0);self.label((x1+x2)/2,dim_y-.9,label,size)
    def dimv(self,y1,y2,feature_x,dim_x,label,size=9):
        sign=1 if dim_x>feature_x else -1
        for y in (y1,y2):self.line(feature_x+sign*1,y,dim_x+sign*2,y,DIM,.19)
        self.line(dim_x,y1,dim_x,y2,DIM,.22);self.arrow(dim_x,y1,0,1);self.arrow(dim_x,y2,0,-1)
        # Text runs bottom to top, offset rather than masking the dimension spine.
        self.text(dim_x-1.7,(y1+y2)/2,label,size,DIM,anchor='middle',rotate=-90)
    def leader(self,pts,label,x,y,size=8.5,color=DIM):
        self.poly(pts,stroke=color,width=.22,close=False);self.arrow(*pts[0],pts[1][0]-pts[0][0],pts[1][1]-pts[0][1],color=color,size=1.5);self.text(x,y,label,size,color)
    def axes(self,x,y,labels=('+X','+Y')):
        self.line(x,y,x+13,y,MUTED,.3);self.arrow(x+13,y,-1,0,MUTED)
        self.line(x,y,x,y-13,MUTED,.3);self.arrow(x,y-13,0,1,MUTED)
        self.text(x+15,y+1,labels[0],8,MUTED);self.text(x-3,y-16,labels[1],8,MUTED)
    def header(self,subtitle,revision):
        self.text(14,18,'FIELD / 4',25,INK,'F4Sans-Bold');self.text(14,27,subtitle,10,MUTED)
        self.text(406,17,'DIMENSIONED DESIGN VIEWS',10,INK,'F4Sans-Bold',anchor='end')
        self.text(406,24,'REV B  /  TWO-PIECE BODY  /  MILLIMETRES',8,MUTED,anchor='end')
        self.line(14,34,406,34,INK,.6)
    def footer(self):
        self.line(14,282,406,282,GRID,.25)
        self.text(14,288,'All dimensions mm. Dimensions govern; do not scale screenshots. A3 landscape PDF is vector artwork.',8,MUTED)
        self.text(406,288,f'{self.index} / 2',8,MUTED,anchor='end')
    def finish(self,svgpath):
        svgpath.write_text(f'<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" width="420mm" height="297mm" viewBox="0 0 420 297"><title>{html.escape(self.title)}</title>'+''.join(self.svg)+'</svg>')
        self.c.showPage()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--manifest',type=Path,required=True);ap.add_argument('--output-root',type=Path,required=True);ap.add_argument('--final',action='store_true');a=ap.parse_args()
    source_path=Path(__file__).with_name('build_sleek.py')
    tree=ast.parse(source_path.read_text());shell_profile=None
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='lid' for t in node.targets) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Name) and node.value.func.id=='loft':
            shell_profile=ast.literal_eval(node.value.args[1]);break
    if not shell_profile:raise ValueError('No shell loft profile found in source builder')
    raw=a.manifest.read_bytes();m=json.loads(raw);d=m['dimensions_mm'];parts={p['id']:p for p in m['parts']};refs={r['id']:r for r in m['references']};interfaces={i['id']:i for i in m['interfaces']}
    if a.final and m['validation']['failed_parts']:raise RuntimeError('Cannot publish dimensions for failed print meshes: '+str(m['validation']['failed_parts']))
    B=refs['REF_KEEP_OUT_battery']['bounds_mm'];P=refs['REF_Pi4_PCB']['bounds_mm'];pi=interfaces['Pi4'];tub=parts['B01_lower_tub'];lid=parts['B02_lift_off_shell'];deck=parts['B04_internal_component_shelf']
    def span(bounds,axis):return bounds[1][axis]-bounds[0][axis]
    def check(actual,expected,label):
        if abs(actual-expected)>.02:raise ValueError(f'{label}: {actual} differs from expected {expected}')
    checks={}
    check(span(lid['assembly_bounds_mm'],0),d['main_body_length'],'body length')
    check(span(lid['assembly_bounds_mm'],1),d['main_body_width'],'body width')
    check(lid['assembly_bounds_mm'][1][2],d['roof_z'],'roof height')
    check(shell_profile[0][0],d['lower_tub_seam_z'],'source shell seam')
    check(shell_profile[-1][0],d['roof_z'],'source shell roof')
    check(max(v[1]for v in shell_profile),d['main_body_length'],'source shell length')
    check(max(v[2]for v in shell_profile),d['main_body_width'],'source shell width')
    for i in range(3):check(span(B,i),d['battery_pocket'][i],'battery reserve axis '+str(i))
    check(span(P,0),d['Pi_board'][1],'Pi X axis');check(span(P,1),d['Pi_board'][0],'Pi Y axis')
    check(d['wheelbase']+d['wheel_diameter'],d['overall_tire_length'],'nominal tire length')
    check(d['track_center_to_center']+d['tire_width'],d['overall_tire_width'],'nominal tire width')
    xs=sorted(set(v[0]for v in pi['hole_centers_xy']));ys=sorted(set(v[1]for v in pi['hole_centers_xy']))
    check(xs[-1]-xs[0],d['Pi_hole_pitch'][1],'Pi hole X pitch');check(ys[-1]-ys[0],d['Pi_hole_pitch'][0],'Pi hole Y pitch')
    lidhead=max(r['bounds_mm'][1][2]for r in refs.values()if 'M3_lid_head'in r['id'])
    floor_top=B[0][2];shelf_bottom=deck['assembly_bounds_mm'][0][2];reserve_top=B[1][2];gap=shelf_bottom-reserve_top
    pdfdir=a.output_root/'docs';pngdir=a.output_root/'previews';pdfdir.mkdir(parents=True,exist_ok=True);pngdir.mkdir(parents=True,exist_ok=True)
    pdfpath=pdfdir/'FIELD4_sleek_dimensions.pdf';c=canvas.Canvas(str(pdfpath),pagesize=(W*mm,H*mm));c.setTitle('FIELD / 4 - dimensioned envelope and component interfaces');c.setAuthor('FIELD / 4 design');c.setSubject('Millimetre nominal design dimensions; separate from printable geometry')
    s=Sheet(c,'FIELD / 4 - nominal assembled envelope',1);s.header('Nominal assembled envelope and wheel spacing',m['revision'])
    s.text(14,45,'01  TOP VIEW',12,INK,'F4Sans-Bold');s.text(14,51,'Orthographic nominal envelope  /  scale 1:2 on A3',8.5,MUTED)
    s.line(221,42,221,251,GRID,.25)
    cx,cy,sc=113,151,.5;L=d['main_body_length'];BW=d['main_body_width'];WB=d['wheelbase'];TR=d['track_center_to_center'];D=d['wheel_diameter'];TW=d['tire_width']
    X=lambda x:cx+x*sc;Y=lambda y:cy-y*sc
    s.rect(X(-L/2),Y(BW/2),L*sc,BW*sc,INK,.45,LIGHT,r=shell_profile[0][3]*sc)
    # The top drawing is explicitly an envelope, not an invented hole/vent outline.
    panel=next(p for k,p in parts.items()if k.startswith('B09_'))['assembly_bounds_mm']
    s.rect(X(panel[0][0]),Y(panel[1][1]),span(panel,0)*sc,span(panel,1)*sc,INK,.3,LIGHT)
    s.text(cx,cy-3,'BODY',10,INK,'F4Sans-Bold',anchor='middle');s.text(cx,cy+3,f'{num(L)} x {num(BW)}',9,MUTED,anchor='middle')
    for x in [-WB/2,WB/2]:
        for y in [-TR/2,TR/2]:
            s.rect(X(x-D/2),Y(y+TW/2),D*sc,TW*sc,INK,.4,LIGHT,r=.6)
            s.line(X(x),Y(y)-8,X(x),Y(y)+8,DIM,.18,[2,1,.5,1]);s.line(X(x)-3,Y(y),X(x)+3,Y(y),DIM,.18)
    s.line(X(-L/2)-4,cy,X(L/2)+4,cy,MUTED,.18,[3,1,.6,1]);s.line(cx,Y(BW/2)-5,cx,Y(-BW/2)+5,MUTED,.18,[3,1,.6,1])
    s.dimh(X(-WB/2),X(WB/2),Y(TR/2),75,f'{num(WB)} wheelbase')
    s.dimh(X(-L/2),X(L/2),Y(-BW/2),226,f'{num(L)} body')
    s.dimh(X(-d['overall_tire_length']/2),X(d['overall_tire_length']/2),Y(-d['overall_tire_width']/2),244,f'{num(d["overall_tire_length"])} tire envelope')
    s.dimv(Y(d['overall_tire_width']/2),Y(-d['overall_tire_width']/2),X(-WB/2-D/2),20,f'{num(d["overall_tire_width"])} tire envelope')
    s.dimv(Y(TR/2),Y(-TR/2),X(-WB/2-D/2),34,f'{num(TR)} wheel-centre track')
    s.dimv(Y(BW/2),Y(-BW/2),X(L/2),200,f'{num(BW)} body')
    s.leader([(X(WB/2),Y(TR/2+TW/2)),(X(WB/2)+12,60),(190,60)],f'{num(TW)} tire width',169,57,8)
    s.text(X(WB/2),Y(-TR/2)-10,'FRONT',8,MUTED,anchor='middle');s.axes(196,244)
    s.text(234,45,'02  SIDE VIEW',12,INK,'F4Sans-Bold');s.text(234,51,'Looking toward -Y  /  scale 1:2 on A3',8.5,MUTED)
    s.rect(234,58,172,33,stroke=None,fill=OPALE,r=2)
    s.text(240,66,'NOMINAL ASSEMBLED SIZE',9,ORANGE,'F4Sans-Bold')
    s.text(240,74,f'{num(d["overall_tire_length"])} L x {num(d["overall_tire_width"])} W x {num(lidhead)} H',15,INK,'F4Sans-Bold')
    s.text(240,82,'Height includes modeled lid screw heads; roof is '+num(d['roof_z'])+'.',8,MUTED)
    s.text(240,87,'No optional sensor or external cables included.',8,MUTED)
    sx,ground=324,188;xx=lambda x:sx+x*.5;zz=lambda z:ground-z*.5
    s.rect(xx(-L/2),zz(d['lower_tub_seam_z']),L*.5,(d['lower_tub_seam_z']-d['body_floor_z'])*.5,INK,.45,LIGHT)
    # Body envelope shoulders from Rev B profile. Shell dimensions are checked above.
    s.poly([(xx(-shell_profile[0][1]/2),zz(shell_profile[0][0]))]+[(xx(w/2),zz(z))for z,w,d_,r_ in shell_profile]+[(xx(-w/2),zz(z))for z,w,d_,r_ in reversed(shell_profile[1:])],INK,.45,OPALE)
    lidheads=[r['bounds_mm']for r in refs.values()if 'M3_lid_head'in r['id']]
    for hb in lidheads[:]:
        s.rect(xx(hb[0][0]),zz(hb[1][2]),span(hb,0)*.5,span(hb,2)*.5,INK,.25,LIGHT,r=.3)
    s.line(xx(-L/2),zz(d['lower_tub_seam_z']),xx(L/2),zz(d['lower_tub_seam_z']),ORANGE,.35)
    for x in [-WB/2,WB/2]:
        s.circle(xx(x),zz(d['axle_z']),D*.25,INK,.5,LIGHT);s.circle(xx(x),zz(d['axle_z']),D*.165,GRID,.35,WHITE)
        s.line(xx(x)-4,zz(d['axle_z']),xx(x)+4,zz(d['axle_z']),DIM,.2);s.line(xx(x),zz(d['axle_z'])-4,xx(x),zz(d['axle_z'])+4,DIM,.2)
    s.line(239,ground,406,ground,INK,.4);s.text(237,ground+5,'Z0 ground',8,MUTED)
    s.dimv(zz(d['roof_z']),ground,xx(L/2),404,f'{num(d["roof_z"])} roof')
    s.dimv(zz(d['axle_z']),ground,xx(-WB/2),247,f'{num(d["axle_z"])} axle',8)
    s.dimh(xx(-WB/2),xx(WB/2),zz(d['axle_z']),204,f'{num(WB)} wheelbase')
    s.leader([(xx(0),zz(d['lower_tub_seam_z'])),(xx(0)+4,101),(357,101)],f'Z{num(d["lower_tub_seam_z"])} lid seam',341,99,8)
    s.leader([(xx(WB/2)+16,zz(d['axle_z'])-17),(388,140),(396,140)],f'DIA {num(D)}',382,137,8)
    s.rect(234,216,172,36,stroke=GRID,width=.25,fill=WHITE,r=1)
    s.multiline(240,224,[f'Body floor underside: Z{num(d["body_floor_z"])}  |  nominal clearance {num(d["nominal_floor_ground_clearance"])}',f'Lowest fastener clearance: approximately {num(d["minimum_ground_clearance_fasteners_approx"])}',f'Lower shell + lid max XY: {num(L)} x {num(BW)}',f'P1S print XY, rotated 90 deg: {num(tub["print_bounds_mm"][0])} x {num(tub["print_bounds_mm"][1])}; 6 mm brim: {num(tub["brim_envelope_6mm"][0])} x {num(tub["brim_envelope_6mm"][1])}',f'+Y side panel projects {num(panel[1][1]-BW/2)} beyond the body; within tire envelope.'],8.2,5.4)
    s.line(14,256,406,256,GRID,.25)
    s.text(14,263,'DESIGN STATUS',8.5,ORANGE,'F4Sans-Bold')
    s.multiline(48,263,['Purchased wheels and screw heads are nominal references. Tread shape, load compression and real hardware can change the assembled envelope.','These are assembly-layout drawings, not fabrication tolerances. Check the actual P1S profile, stock exclusion zone, and fit coupons before full prints.'],8.5,5)
    s.footer();s.finish(pdfdir/'FIELD4_sleek_envelope.svg')

    s=Sheet(c,'FIELD / 4 - battery reserve and Raspberry Pi 4 mounting interface',2);s.header('Battery reserve and Raspberry Pi 4 mounting interface',m['revision'])
    s.text(14,45,'03  BATTERY POCKET / RESERVE',12,INK,'F4Sans-Bold');s.text(14,51,'Unverified pack allowance; measure the actual battery',8.5,ORANGE)
    s.line(216,42,216,274,GRID,.25)
    bx,by=111,116;bl,bw,bh=d['battery_pocket'];X=lambda x:bx+x;Y=lambda y:by-y
    s.rect(X(-bl/2),Y(bw/2),bl,bw,ORANGE,.5,OPALE,dash=[2,1])
    # The lighter body illustrates only the battery reference, never a verified pack fit.
    battery=refs['REF_battery']['bounds_mm'];actualL=span(battery,0);actualW=span(battery,1)
    s.rect(X(-actualL/2),Y(actualW/2),actualL,actualW,GRID,.3,WHITE,r=3)
    s.text(bx,111,'RESERVED CLEAR SPACE',10,ORANGE,'F4Sans-Bold',anchor='middle')
    s.text(bx,118,f'{num(bl)} x {num(bw)} x {num(bh)} high',12,INK,'F4Sans-Bold',anchor='middle')
    s.text(bx,126,'Soft padding + straps; never hard-clamp.',8.3,MUTED,anchor='middle')
    s.dimh(X(-bl/2),X(bl/2),Y(-bw/2),169,num(bl)+' clear X')
    s.dimv(Y(bw/2),Y(-bw/2),X(-bl/2),40,num(bw)+' clear Y')
    s.axes(189,169);s.text(58,70,'TOP VIEW  /  scale 1:1 on A3',8.5,MUTED)
    s.text(14,182,'HEIGHT SECTION',10,INK,'F4Sans-Bold');s.text(14,188,'Scale 1:2 on A3; no cable/connector fit implied',8,MUTED)
    bx,base=103,252;sc=.5;XX=lambda x:bx+x*sc;ZZ=lambda z:base-(z-floor_top)*sc
    s.rect(XX(-bl/2),ZZ(reserve_top),bl*sc,bh*sc,ORANGE,.4,OPALE,dash=[2,1])
    s.rect(XX(-bl/2)-4,ZZ(floor_top),bl*sc+8,2.25,INK,.3,LIGHT)
    s.rect(XX(-bl/2)-4,ZZ(shelf_bottom+4),bl*sc+8,2,INK,.3,LIGHT)
    s.dimv(ZZ(reserve_top),ZZ(floor_top),XX(-bl/2),62,num(bh)+' reserved height',8)
    s.line(XX(bl/2)+3,ZZ(reserve_top),154,ZZ(reserve_top),DIM,.2)
    s.line(XX(bl/2)+3,ZZ(shelf_bottom),154,ZZ(shelf_bottom),DIM,.2)
    s.line(151,ZZ(reserve_top),151,ZZ(shelf_bottom),DIM,.2)
    s.multiline(157,211,[num(gap)+' nominal gap',f'Shelf underside Z{num(shelf_bottom)}'],8,4.5,DIM)
    s.multiline(143,239,[f'Reserve top Z{num(reserve_top)}',f'Floor top Z{num(floor_top)}'],8,5,MUTED)
    s.text(14,267,'Connector exits, padding and strap thickness must be checked on the real pack.',8.2,ORANGE)
    s.text(14,272,'Reserve is centered at world X0, Y0. Light outline is an illustrative battery only.',8.2,MUTED)

    s.text(229,45,'04  RASPBERRY PI 4 INTERFACE',12,INK,'F4Sans-Bold');s.text(229,51,'Top view  /  scale 1:1 on A3  /  all coordinates are world XY',8.5,MUTED)
    pcx,pcy=302,126;px0,py0=pi['board_origin_xy'];pcbx=(P[0][0]+P[1][0])/2;pcby=(P[0][1]+P[1][1])/2
    PX=lambda x:pcx+(x-pcbx);PY=lambda y:pcy-(y-pcby)
    s.rect(PX(P[0][0]),PY(P[1][1]),span(P,0),span(P,1),INK,.4,GREEN,r=2)
    s.line(PX(P[0][0]),PY(ys[-1]+9),PX(P[1][0]),PY(ys[-1]+9),GRID,.25,[2,1]);s.text(pcx,PY(P[1][1])+9,'PORT END (+Y)',8,MUTED,anchor='middle')
    holes=sorted(pi['hole_centers_xy'],key=lambda h:(h[1],h[0]))
    for i,(x,y)in enumerate(holes,1):
        s.circle(PX(x),PY(y),pi['boss_diameter']/2,INK,.25,WHITE);s.circle(PX(x),PY(y),pi['hole_diameter']/2,DIM,.3,WHITE)
        s.line(PX(x)-5,PY(y),PX(x)+5,PY(y),DIM,.15,[1,.5]);s.line(PX(x),PY(y)-5,PX(x),PY(y)+5,DIM,.15,[1,.5])
        s.text(PX(x)+(5 if x==xs[0]else-5),PY(y)+1,'H'+str(i),8,MUTED,anchor='start'if x==xs[0]else'end')
    s.text(pcx,132,'PCB',10,INK,'F4Sans-Bold',anchor='middle');s.text(pcx,138,f'{num(span(P,1))} along +Y',8,MUTED,anchor='middle')
    s.dimh(PX(P[0][0]),PX(P[1][0]),PY(P[1][1]),65,num(span(P,0))+' PCB X')
    s.dimv(PY(P[1][1]),PY(P[0][1]),PX(P[1][0]),387,num(span(P,1))+' PCB Y')
    s.dimh(PX(xs[0]),PX(xs[-1]),PY(ys[0]),182,num(xs[-1]-xs[0])+' hole pitch X')
    s.dimv(PY(ys[-1]),PY(ys[0]),PX(xs[-1]),352,num(ys[-1]-ys[0])+' hole pitch Y')
    s.leader([(PX(xs[-1]),PY(ys[-1])),(342,97),(367,97)],f'4 x DIA {num(pi["hole_diameter"])} bores',342,93,8)
    s.text(342,103,f'DIA {num(pi["boss_diameter"])} bosses; {num(pi["boss_height"])} high',8,DIM)
    s.circle(PX(px0),PY(py0),1,ORANGE,.3,ORANGE)
    s.leader([(PX(px0),PY(py0)),(255,177),(231,177)],f'PCB origin ({num(px0)}, {num(py0)})',230,190,8,color=ORANGE)
    s.text(229,201,'MOUNTING-HOLE CENTRES',9,INK,'F4Sans-Bold')
    s.text(229,207,'Hole',8,MUTED);s.text(250,207,'World X',8,MUTED);s.text(278,207,'World Y',8,MUTED);s.text(313,207,'PCB-edge offsets (X, Y)',8,MUTED)
    s.line(229,210,404,210,GRID,.3)
    for i,(x,y)in enumerate(holes,1):
        row=211+i*7;s.text(229,row,'H'+str(i),9,INK,'F4Sans-Bold');s.text(250,row,num(x),9);s.text(278,row,num(y),9);s.text(313,row,f'{num(x-px0)}, {num(y-py0)}',9)
    s.line(229,241,404,241,GRID,.25)
    s.multiline(229,248,[f'Seat top Z{num(P[0][2])}; PCB thickness {num(span(P,2))}. Four M2.5 fasteners.',f'Pitch is {num(d["Pi_hole_pitch"][0])} along Y x {num(d["Pi_hole_pitch"][1])} along X. Boss bores are printed clearances.', 'Verify the actual board revision, screw length, plugs and cooler before assembly.', 'Axes: +X forward, +Y left, +Z up. Board origin is its lower-X/lower-Y corner.'],8.2,6)
    s.footer();s.finish(pdfdir/'FIELD4_sleek_interfaces.svg');c.save()
    # Render the actual final PDF, not an approximate duplicate.
    subprocess.run(['pdftoppm','-png','-r','180',str(pdfpath),str(pngdir/'FIELD4_sleek_dimensions')],check=True)
    for i,name in [(1,'FIELD4_sleek_envelope_dimensions.png'),(2,'FIELD4_sleek_interface_dimensions.png')]:
        (pngdir/f'FIELD4_sleek_dimensions-{i}.png').rename(pngdir/name)
    report={'design':m['design'],'revision':m['revision'],'units':'millimetres','source_manifest_sha256':hashlib.sha256(raw).hexdigest(),'publication':'final'if a.final else'draft pending geometry confirmation','checks':'Dimension values cross-checked against dimensional manifest, body bounds, component reference bounds, and mounting coordinates.','nominal_assembled_envelope_mm':[d['overall_tire_length'],d['overall_tire_width'],lidhead],'roof_z':d['roof_z'],'battery_reserve_mm':d['battery_pocket'],'battery_reserved_top_to_shelf_mm':gap,'pi_hole_centers_xy':holes,'warnings':['Purchased-part geometry is nominal.','Battery pack and connector fit must be measured.','No physical print, fit, strength or safety certification.'],'outputs':[p.name for p in [pdfpath,pdfdir/'FIELD4_sleek_envelope.svg',pdfdir/'FIELD4_sleek_interfaces.svg',pngdir/'FIELD4_sleek_envelope_dimensions.png',pngdir/'FIELD4_sleek_interface_dimensions.png']]}
    (pdfdir/'FIELD4_dimensioned_views_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
