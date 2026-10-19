from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor, Color, white
from reportlab.platypus import Paragraph, Table, TableStyle
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from pathlib import Path
import json
# Rebuild from the redistributable PNGs beside this script.
# No Blender installation or private project files are needed to build the PDF.
import argparse
import os
parser = argparse.ArgumentParser(description="Build the FIELD4 public beginner manual.")
parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent)
parser.add_argument("--font-dir", type=Path, help="Directory containing DejaVuSans.ttf and DejaVuSans-Bold.ttf")
args = parser.parse_args()
R = Path(__file__).resolve().parent
O = args.output_dir.resolve()
O.mkdir(parents=True, exist_ok=True)
font_dirs = [p for p in [args.font_dir, Path(os.environ["FIELD4_FONT_DIR"]) if os.environ.get("FIELD4_FONT_DIR") else None, R / "fonts", Path("/usr/share/fonts/truetype/dejavu")] if p]
font_dir = next((p for p in font_dirs if (p / "DejaVuSans.ttf").is_file() and (p / "DejaVuSans-Bold.ttf").is_file()), None)
if font_dir is None:
    parser.error("DejaVu Sans fonts not found. Install DejaVu Sans or pass --font-dir PATH (regular and bold TTF files).")
pdfmetrics.registerFont(TTFont('D', str(font_dir / 'DejaVuSans.ttf')))
pdfmetrics.registerFont(TTFont('DB', str(font_dir / 'DejaVuSans-Bold.ttf')))
C=canvas.Canvas(str(O/'FIELD4_Beginner_Visual_Build_Manual.pdf'),pagesize=(595,842));C.setTitle('FIELD4 | Beginner visual build manual');C.setAuthor('FIELD4 project')
ink=HexColor('#172A35');gray=HexColor('#516571');orange=HexColor('#D35E29');teal=HexColor('#167D86');paper=HexColor('#F6F7F5');line=HexColor('#D9E1E3');page=0
style=ParagraphStyle('body',fontName='D',fontSize=10.5,leading=15,textColor=ink,spaceAfter=7)
def p(txt,x,y,w=507,size=10.5,color=ink,bold=False):
 st=ParagraphStyle('a',parent=style,fontName='DB' if bold else 'D',fontSize=size,leading=size*1.4,textColor=color)
 q=Paragraph(txt,st);_,h=q.wrap(w,800);q.drawOn(C,x,y-h);return y-h

def start(k,title,sub=''):
 global page;page+=1;C.setFillColor(paper);C.rect(0,0,595,842,fill=1,stroke=0)
 C.setFillColor(orange);C.rect(0,822,595,20,fill=1,stroke=0)
 p('FIELD4  /  BUILD MANUAL',44,798,size=9,bold=True,color=teal)
 p(k.upper(),44,772,size=9,bold=True,color=orange)
 y=p(title,44,747,size=25,bold=True)
 if sub:p(sub,44,y-12,size=10.5,color=gray)
 C.setStrokeColor(line);C.line(44,39,551,39);p('PROTOTYPE  •  Power off for every step',44,29,size=8,color=gray);p(f'{page:02d}',523,29,w=28,size=8,color=gray)

def end():C.showPage()
def pic(n,x=34,y=300,w=527,h=340):
 C.drawImage(str(R/'renders'/f'{n}.png'),x,y,w,h,mask='auto',preserveAspectRatio=True,anchor='c')
 if n in {'cover', 'sensor', 'shelf', 'electronics'}:
  p('SENSOR: SCHEMATIC PROXY / NOT FOR FIT',44,676,w=507,size=7.5,color=teal,bold=True)
def markers(n,x=34,y=300,w=527,h=340):
 data=json.load(open(R/'annotations.json'))[n]
 scale=min(w/1400,h/900);iw=1400*scale;ih=900*scale;ox=x+(w-iw)/2;oy=y+(h-ih)/2
 for lab,(u,v) in data:
  xx=ox+u*iw;yy=oy+v*ih
  C.setFillColor(teal);C.setStrokeColor(white);C.setLineWidth(1);C.circle(xx,yy,6,fill=1,stroke=1)
  C.setFillColor(white);C.setFont('DB',6.5);C.drawCentredString(xx,yy-2.3,lab if len(lab)<4 else '')
  if len(lab)>3:p(lab,xx-15,yy-13,w=55,size=8,bold=True,color=teal)

def box(title,txt,y=130,color=teal):
 C.setFillColor(white);C.roundRect(44,y,507,82,8,fill=1,stroke=0);C.setFillColor(color);C.rect(44,y,4,82,fill=1,stroke=0)
 p(title,59,y+67,w=475,size=10,bold=True,color=color);p(txt,59,y+47,w=475,size=10)
def bullets(items,y=290):
 for i,t in enumerate(items,1):y=p(f'<b>{i}.</b>  {t}',44,y)-10
 return y

def down_arrow(x,y,label):
 C.setStrokeColor(teal);C.setFillColor(teal);C.setLineWidth(2.5);C.line(x,y,x,y-42)
 q=C.beginPath();q.moveTo(x-5,y-35);q.lineTo(x,y-44);q.lineTo(x+5,y-35);C.drawPath(q,fill=1,stroke=0)
 p(label,x-47,y+24,w=94,size=8,color=teal,bold=True)

def stack(x,y):
 p('ONE SHELF JOINT',x-15,y+26,w=130,size=8,bold=True,color=teal)
 C.setStrokeColor(gray);C.setLineWidth(5);C.line(x+10,y-7,x+10,y-133)
 for yy,w,h,col in [(y,28,15,gray),(y-18,39,4,orange),(y-29,65,60,HexColor('#AAB9C0')),(y-95,39,4,orange),(y-105,34,6,teal),(y-117,31,13,gray)]:
  C.setFillColor(col);C.rect(x+10-w/2,yy-h,w,h,fill=1,stroke=0)
 for dy,lab in [(7,'Head'),(23,'Flat'),(58,'Printed stack'),(100,'Flat'),(113,'Split'),(132,'Plain nut')]:p(lab,x+48,y-dy+5,w=65,size=7.5)
 p('SCHEMATIC / NOT TO SCALE',x-20,y-153,w=150,size=6.5,color=gray)

def table(rows,widths,y):
 ts=ParagraphStyle('cell',parent=style,fontSize=9,leading=12)
 t=Table([[Paragraph(str(v),ts) for v in row] for row in rows],colWidths=widths)
 t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),HexColor('#E4ECEE')),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),8),('RIGHTPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),9),('BOTTOMPADDING',(0,0),(-1,-1),9),('LINEBELOW',(0,0),(-1,-1),.5,line)]));w,h=t.wrap(507,700);t.drawOn(C,44,y-h);return y-h

start('Your first build','A robot, one clear step at a time.','FIELD4 • Recessed-LiDAR prototype • Public edition with asset holds')
pic('cover',20,330,555,330)
p('A beginner-friendly mechanical build guide',44,301,size=19,bold=True)
p('Public CAD views show an open chassis and a schematic sensor. The current C02 lid and T10 aperture gauge are withheld pending reference-model redistribution clarification.',44,263,size=11)
box('READ THIS FIRST','This public package is incomplete for the intended C02 + C04 build. Do not infer lid or aperture geometry from the pictures, or use a substitute lid. Keep all power disconnected; this prototype is uncommissioned.',120,color=orange)
p('Build facts retained: B01 tub + B05 pillars + B09 blank + matched C02/C04. C02 and T10 files are withheld; B08 remains excluded.',44,93,size=9,color=gray);end()

start('01 / Plan your session','Check what is ready','Inspect the actual parts on your bench before opening a fastener bag.')
table([['Component','Check before use','Build gate'],['4 Pololu DRV8874 drivers','Confirm exact board revision and condition','No wiring or mounting approval is established by this guide.'],['4 goBILDA 96 mm Hogback wheels + 4 Sonic 6 mm hubs','Inspect wheels, hub bores and clamp access','Verify shaft engagement and clearance with the actual motors.'],['Mechanical fasteners','Count and measure against page 3','Do not infer screw length or washer thickness from appearance.'],['4 DFRobot FIT0185 motors','Check the actual gearbox face, shaft and lead exit','Pass the motor coupon and one-corner fit checks first.'],['Printed parts + A1M8 LiDAR','C02 lid and T10 gauge are withheld','The public package cannot complete the intended lid/aperture fit gates.'],['Battery, power protection and wiring','Electrical design is uncommissioned','Do not connect any power source.']], [165,162,180],650)
box('TODAY’S STOPPING POINT','Prepare only the available power-off mechanical checks. C02 lid and T10 gauge are withheld; stop at any dependent step. Keep the shelf removable and do not improvise a replacement lid or aperture.',105)
end()

start('02 / Sort the hardware','Make five small parts trays','Required quantities for these mechanical joints only; other assemblies need separate checks.')
table([['Tray / job','Required for this build','Reference SKU'],['LiDAR final mount','4 M2.5×16 screws + 4 M2.5 thin flat washers','MSP2.5X16SS<br/>WF2.5X5SS'],['Shelf to tub','4 M4×100 SOCKET-head screws + 8 M4 flats + 4 M4 split washers + 4 M4 plain nuts','BSH4X100<br/>WF4X8SS<br/>WL4SS; NR4SS'],['Pillars + lid','8 M3×12 button screws + 8 M3 plain nuts; no flat washers in these stacks','SB3X12SS<br/>NR3SS'],['Switch BLANK','4 M3×12 button screws + 8 M3 thin flats + 4 M3 plain nuts','SB3X12SS; NR3SS<br/>WF3X6SS'],['Shared M3 totals','12 M3×12 screws + 12 M3 plain nuts across the two jobs above','Verify actual sizes<br/>before assembly']], [122,240,145],650)
p('The three specified thin flat-washer sizes are 0.5 mm thick. Flat washer = smooth ring. Split washer = ring with a split. Plain nut = no nylon locking ring. Do not substitute taller locking nuts in printed pockets. SKU references identify BelMetric parts; verify specifications before use.',44,245,size=10)
box('ADDITIONAL HARDWARE REQUIRED','T09 needs separate M2.5×6 screws; they are not in the list above. Motor, wheel, electronics, controller-hanger and camera fasteners also require separate verification.',105,color=orange);end()

start('03 / Learn the parts','This is how the body fits together','Exploded public parts view. The intended C02 lid is withheld and not shown.')
pic('parts',35,300,525,350)
p('SHOWN: four B05 pillars + C04 shelf  →  B01 tub. C02 lid is withheld.',44,305,size=10,bold=True)
bullets(['The intended C02 lid is removable; its file is withheld. The LiDAR and shelf are carried by the lower body rather than the lid.','FL / FR / RL / RR mean front-left / front-right / rear-left / rear-right, as the robot faces forward. Front is the camera end; the LiDAR sits toward the rear.','C02 and C04 remain the intended matched pair, but C02 is not distributed here. Do not substitute older B02/B04 parts or invent a lid from this illustration. Leave B08 out.'],275)
box('BEGINNER TIP','Lay out labeled parts before fastening them. A dry fit means putting parts together gently, without power and without fully tightening screws.',80);end()

start('04 / Prepare','Get the bench ready','Keep small screws in trays. Put electronics on a clean, nonconductive surface.')
pic('lower',35,340,525,300)
markers('lower',35,340,525,300)
bullets(['Gather a ruler or caliper, the correct-fit hex keys and screwdriver for your screws, 5.5 mm (M3 nut) and 7 mm (M4 nut) spanners, deburring tool, and eye protection when removing supports. Do not use a power driver.','Remove print supports and loose strings. Inspect the B01 tub, shelf seats, rim and screw holes. Stop for cracks, missing layers or visible warping.','Use T02 for fastener clearance and T05 for captive-nut fit first. Screws should start by hand. If a hole or nut pocket is too tight, stop and correct the print; do not force metal into plastic.'],335)
box('PRINTING CHECK','Import supplied STL files in millimetres at 100%. C04 shelf: 183 × 212 × 11 mm. Intended C02 lid: 196 × 236 × 43 mm, but its file is withheld. Check print orientation, supports and the actual slicer profile.',80);end()

start('05 / Test before large prints','Check the LiDAR fit first','The public T09 gauge checks the feet. The T10 aperture gauge is withheld.')
pic('coupons',35,365,525,265)
p('T09 four-foot gauge only. T10 aperture profile and file are withheld.',44,366,size=10,bold=True)
bullets(['T09: rest the four fixed sensor feet on the frame. All four holes must line up without bending the feet. For a screw-fastened check, use M2.5×6 with the specified 0.5 mm washers.','T10 is withheld pending reference-model redistribution clarification. Its intended check is vertical clearance around the rotating head and belt/pulley extension. Do not improvise that profile or force a gauge past optics or the belt.','Check the actual A1M8 revision. A matching product name alone does not prove that its feet and body match this model.'],335)
box('STOP: DO NOT USE THE LONG SCREWS','M2.5×16 is for the final 11 mm shelf/seat stack only. Through the 3 mm T09 coupon it can damage the sensor. If the short screws are missing, do only a gentle no-screw alignment check; the fastened check is still pending.',70,color=orange);end()

start('06 / Build the shelf subassembly','Fit the four lid pillars','Parts: C04 shelf + B05 FL, FR, RL, RR • Hardware: 4 M3×12 + 8 plain M3 nuts')
pic('pillars',35,340,525,300)
markers('pillars',35,340,525,300)
p('Labels identify the four matching pillar positions.',44,350,size=8,color=gray)
bullets(['Place one ordinary M3 nut into the pocket at each end of every B05 pillar: eight nuts in total. Each nut must sit fully in its pocket and line up with the screw hole.','Put the labeled pillars in the matching shelf corners. Start four M3×12 screws upward from below the shelf into the bottom pillar nuts. No washers are specified for this stack.','Tighten gently with a hand tool, just until seated. Keep the upper nuts free for the lid screws. Plastic should not bow or whiten.'],335)
box('CHECK BEFORE MOVING ON','All four pillars stand square. Nuts do not spin. Every screw starts easily by hand. If a nut will not seat, fix the printed pocket rather than substituting a taller nyloc nut.',80);end()

start('07 / Close the unused opening','Fit the side switch blank','Parts: B09 BLANK • Hardware: 4 M3×12 + 8 thin M3 flat washers + 4 plain M3 nuts')
pic('blank',55,365,485,250)
bullets(['Find the matching side-panel opening and hold B09 against it. The panel is a blank adapter, not an installed electrical switch.','Use four M3×12 screws. Fit this before installing the shelf, while you can reach both sides. Each joint uses a flat washer under the head and another under the plain nut. Start all four before tightening.','Check that the panel sits flat and no screw tail touches a future wire route. Keep the blank unmodified until a real switch and its cutout/panel requirements are confirmed.'],335)
box('NO SWITCH CUTOUT YET','Do not drill a guessed hole from a model or generic part list. The specified hardware fastens the blank only; it does not establish switch fit or make the power system ready.',80,color=orange);end()

start('08 / Mount the LiDAR','Sensor above. Screws below.','Only after the coupon checks pass • Hardware: 4 M2.5×16 + 4 specified thin flat washers')
pic('sensor',35,340,525,305)
markers('sensor',35,340,525,305)
down_arrow(190,580,'LOWER SENSOR')
p('1-4: shelf-seat positions; the rear seat is hidden by the schematic sensor.',44,350,size=8,color=gray)
bullets(['Set the LiDAR’s four fixed feet onto the four C04 seats. In the model, its motor/belt extension points toward the front (camera end). The rotor is toward the rear.','Support the shelf safely without resting the sensor on its optics. From below, put a thin washer on each M2.5×16 screw, then start each screw into a sensor foot by hand.','Measure the real stack before tightening. Nominally: 4 mm shelf + 7 mm boss + 0.5 mm washer leaves 4.5 mm engagement into a 6 mm-deep thread. Hand-snug only; never bottom a screw.'],335)
box('CHECK BEFORE MOVING ON','All four feet rest flat. Screws do not reach moving parts or the PCB. Route the sensor cable below the roof and away from the rotor/belt, with slack for service. Never hang the shelf by a cable.',65);end()

start('09 / Seat the shelf','Lower the shelf onto the tub','Hardware: 4 M4×100 socket heads + 8 thin flats + 4 split washers + 4 plain M4 nuts')
pic('shelf',22,355,370,290)
markers('shelf',22,355,370,290)
stack(422,575)
p('1-4: long-bolt seats • Enlarged gap shows lowering direction.',44,358,size=8,color=gray)
bullets(['First check the space below the shelf. Leave out the B08 buck platform and any unapproved battery/power hardware. Lower the shelf straight onto its four tub seats.','At each fixing: socket-head screw → upper flat washer → printed shelf/seat stack → lower flat washer → split washer → plain M4 nut. The screws point downward.','Start all four joints loosely, then tighten gently and evenly. Measure the actual split-washer stack; do not assume it fully flattens. Confirm the nut is on the fully threaded section and at least two full threads protrude, and that nuts/tails do not compromise ground clearance.'],345)
box('IMPORTANT CLEARANCE CHECK','Tighten these bolts before connecting Pi plugs. A long-bolt head overlaps a modeled Pi plug/cable space. Check your actual plugs, cables and tool access; do not assume a rendered connector proves fit.',65,color=orange);end()

start('10 / Lid step on hold','C02 is withheld','Retained assembly facts only. The public files do not complete this step.')
C.setFillColor(white);C.roundRect(44,405,507,220,10,fill=1,stroke=0)
p('C02 LID + T10 GAUGE',68,592,w=459,size=19,bold=True,color=teal)
p('Files and affected illustrations withheld',68,551,w=459,size=14,bold=True)
p('Reference-model redistribution needs clarification. No replacement aperture or printable lid geometry is supplied. Do not derive a fit design from the sensor proxy or this manual.',68,514,w=459,size=11)
p('The checks below preserve the intended build requirements; they are not permission to bypass the missing parts or fit gates.',68,447,w=459,size=10,color=gray)
bullets(['If an appropriately cleared C02 reference becomes available, keep power disconnected and cables clear; lower it vertically over the sensor. Do not tilt it or push against the rotor, belt extension or camera area.','Confirm the lid seats on its rim without force. The sensor is carried by the shelf, not by the roof. The modeled lower edge of the rotating head is only 1.5 mm above the roof: check the actual clearance.','If it fits, start four M3×12 lid screws from above into the upper pillar nuts. No washers are specified. Hand-snug only. Remove and replace the lid once to prove a clear vertical service path.'],335)
box('STOP IF IT TOUCHES','Any rubbing, pinched cable or forced fit means stop and correct the fit. Do not use the lid screws to pull a warped or obstructed lid down. Never lift the robot by this cover.',65,color=orange);end()

start('11 / Electronics fit gate','Keep this area provisional','The model places boards; it does not prove wiring, electrical design or actual board mounting.')
pic('electronics',35,350,525,300)
bullets(['The Pi and Pico shelf bosses are integral. Do not stack extra standoffs on top without rechecking the design. Check actual board undersides, insulating washers and connector clearances.','Two C16 hangers are conditional. They assume a 90 × 70 mm carrier with 80 × 60 mm hole pitch, not a ready-made mounting pattern for four loose DRV8874 boards.','C16 hardware, camera fasteners and IMU mounting are outside the page 3 list. Check the actual camera/carrier fit and cable route. Leave these installations pending if unverified.'],335)
box('DO NOT POWER IT','Battery, fuses/protection, E-stop, driver current limits, encoder voltage compatibility, safe boot and loss-of-command behavior are unresolved. No wiring diagram or powered test is approved by this manual.',65,color=orange);end()

start('12 / Conditional: motor installation','Test one motor and bracket first','Conditional sequence • Verify the suggested hardware and actual component fit before assembly.')
pic('motor',60,350,475,285)
bullets(['Start with the T01 motor-face coupon. Check the six M3 positions, gearbox boss, offset shaft and relief for the factory screw heads. The relief side faces the gearbox. Stop if the face does not sit flat.','For each B10 mount, the design calls for six M3×8 with approximately 0.5 mm washers through a 5 mm face: nominal 2.5 mm insertion, versus the stated 3 mm maximum. Measure the actual motor and stack; never probe depth with a long screw.','Bracket-to-tub fixings were specified as M4×20 upward from below, washers and locking nuts above the feet. Verify the exact required hardware and real tool clearance before proceeding.'],335)
box('WAIT FOR THE REAL MOTOR','The rendered motor is simplified. Its encoder, connector and lead exit may differ. Confirm rear clearance, screw-head/tire clearance and lead routing before repeating the assembly at all four corners.',65,color=orange);end()

start('13 / Later: wheels and hubs','Clamp the hub before the wheel','Check final shaft fit using the actual wheels, hubs and motors.')
pic('wheel',60,345,475,300)
bullets(['Fit a Sonic 6 mm round-bore hub onto the 6 mm D shaft. The model targets its back face 9.5 mm outward from the gearbox face. Verify actual shaft engagement and use the hub maker’s tightening instructions.','Keep the hub pinch bolts reachable and clamp the hub before fitting the wheel. The wheel’s 8 mm inset side faces the motor. Do not guess torque or add improvised shims.','Suggested design hardware: four M4×10 per wheel with approximately 0.8 mm washers through the 4 mm web, giving nominal 5.2 mm hub engagement. This hardware is outside the page 3 list; verify actual screws do not bottom or foul.'],335)
box('POWER-OFF SPIN CHECK','Rotate each wheel by hand through a full turn, with a light side load. Check the tire against every screw head, washer, bracket and shell edge. Any rubbing or flex-related contact is a stop.',65);end()

start('14 / Final mechanical checkpoint','Stop here, then review','A tidy dry build is progress. It is not yet a commissioned robot.')
table([['Check','Pass / hold'],['Correct C02 + C04 pair; B08 absent; no old B02/B04 mixed in','________'],['No cracks, warped seating faces or forced fasteners','________'],['LiDAR coupon checks passed; real screw projection measured','________'],['Four feet seated; rotor/belt/optics clear; cables restrained safely','________'],['Shelf stack verified; plain nuts engaged; ground/tool/plug clearance checked','________'],['Lid lowers and lifts vertically without touching anything','________'],['Motor/wheel steps deferred until actual motor + hardware checks pass','________'],['Battery and all power sources remain disconnected','________']], [425,82],650)
p('If something fails: stop, photograph the part and screw stack, and record the part ID and measured dimension. Do not force the fit, drill a sensor, bottom a gearbox screw or change electrical settings to compensate.',44,268,size=11)
box('NEXT BUILD SESSION','Resolve the withheld C02/T10 references and missing hardware/fit checks before completing the intended build. A reviewed power/wiring design is required before powered tests. The Windows simulator is not ready-to-run Pi firmware.',105);end()

start('Reference / What this guide is based on','Model and evidence notes','Use these notes when a physical part differs from the picture.')
y=650
for title,txt in [
('Selected geometry','The intended build pairs C02 recessed A1M8 lid with C04 rearranged shelf, retaining RevB B01, B05 and B09. C02 and T10 aperture-gauge files and affected illustrations are withheld pending reference-model redistribution clarification. B02/B04 remain superseded; B08 is excluded.'),
('Public illustrations','Public illustrations omit the withheld C02 lid and T10 gauge. The sensor is an original simple geometric proxy, not manufacturer CAD, an exact outline, or evidence of fit. No generic placeholder is a printable replacement or a redesigned engineering part.'),
('Fit and fastener limits','Dimensions and screw stacks are nominal design values. Verify all actual component revisions, printed dimensions, engagement depths, washer thicknesses, tool access and clearances. Pass the coupons before larger prints. No physical fit, strength, thermal, vibration or wiring tests are certified by this guide.'),
('Mechanical scope only','The guide covers a power-off mechanical prototype. Electronics mounting and all power-system decisions remain conditional or unresolved. Do not use a picture, simulator or hardware list as electrical commissioning approval.'),
('Use the makers’ instructions','Confirm current component instructions and actual revisions before the conditional drivetrain steps. Manufacturer reference sites: slamtec.com/en/support • dfrobot.com/product-633.html • gobilda.com • pololu.com. These references do not endorse unverified substitutions.')]:
 y=p(title,44,y,size=12,bold=True,color=teal)-6;y=p(txt,44,y,size=10)-19
p('Public beginner mechanical guide v1 • Dimensions are nominal millimetres unless stated otherwise.',44,77,size=8.5,color=gray)
end();C.save()
print(O/'FIELD4_Beginner_Visual_Build_Manual.pdf')
