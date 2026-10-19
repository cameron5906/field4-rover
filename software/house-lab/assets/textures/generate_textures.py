from PIL import Image
import math,random
from pathlib import Path
p=Path(__file__).resolve().parent; n=512
for kind in ['oak','plaster','linen','tile','stone','walnut']:
 r=random.Random(412); im=Image.new('RGB',(n,n)); pix=im.load()
 for y in range(n):
  for x in range(n):
   noise=r.uniform(-1,1)
   if kind in ['oak','walnut']:
    board=y//64; offset=(board%3)*173; seam=y%64<2 or (x+offset)%512<2
    grain=math.sin(x*.12+5*math.sin(y*.035)+math.sin(x*.021))*2+math.sin(y*1.8+x*.024)*2
    tone=[-9,4,0,8,-3,3,-6,5][board]+grain+noise*3
    base=(170,137,97) if kind=='oak' else (111,78,50)
    col=tuple(max(0,int(v+tone-(28 if seam else 0))) for v in base)
   elif kind=='tile':
    seam=x%128<3 or y%128<3; tint=((x//128*3+y//128)%5-2)*2
    col=tuple(int(v+noise*2+tint) for v in ((94,107,103) if seam else (153,165,160)))
   elif kind=='linen':
    v=noise*7+(3 if x%4<2 else -3)+(2 if y%4<2 else -2); col=tuple(int(c+v) for c in (205,201,184))
   elif kind=='plaster':
    v=noise*3+math.sin(x*.06)*math.sin(y*.08)*1.8; col=tuple(int(c+v) for c in (206,201,188))
   else:
    v=noise*3+math.sin(x*.05+math.sin(y*.06))*3; col=tuple(int(c+v) for c in (166,165,153))
   pix[x,y]=col
 im.save(p/(kind+'.png'))
