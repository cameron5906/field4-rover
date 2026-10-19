"""Pixel-only raised-hands classifier. No simulator actor/button input."""
import math

def classify_pose(points):
    ids=[11,12,13,14,15,16,23,24]
    if len(points)<25:return {"gesture":"unknown"}
    pts=[points[i] for i in ids]
    if any(not all(math.isfinite(float(p.get(k,float('nan')))) for k in ['x','y','visibility','presence']) for p in pts):return {"gesture":"unknown"}
    confidence=min(min(p['visibility'],p['presence']) for p in pts)
    shoulders=[points[11],points[12]];hips=[points[23],points[24]]
    shoulder_y=sum(p['y'] for p in shoulders)*.5;hip_y=sum(p['y'] for p in hips)*.5
    scale=hip_y-shoulder_y
    center=[sum(p['x'] for p in shoulders+hips)*.25,(shoulder_y+hip_y)*.5]
    result={"gesture":"unknown","center":center,"scale":scale,"confidence":confidence}
    if confidence<.55 or scale<.045:return result
    if any(p['x']<.015 or p['x']>.985 or p['y']<.015 or p['y']>.985 for p in pts):return result
    wrists=[points[15],points[16]]
    raised=all(w['y']<s['y']-.20*scale for w,s in zip(wrists,shoulders))
    lowered=all(w['y']>s['y']+.22*scale for w,s in zip(wrists,shoulders))
    result['gesture']='raised' if raised else ('lowered' if lowered else 'unknown')
    return result
