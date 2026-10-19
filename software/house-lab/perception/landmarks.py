"""Conservative camera-bearing landmark hypotheses, using measured sensor data only.

Map coordinates: x,z; pose=(x,z,yaw), positive yaw turns forward (-z) left.
No simulator object IDs, meshes, true poses, depth buffers or floor-plan inputs.
Lidar is a corroborating surface hint, NEVER an automatic object depth source.
Uncertainty values are heuristic bounds, not calibrated statistical guarantees.
"""
from __future__ import annotations
import math
import statistics
from dataclasses import dataclass, field

STATIC_LABELS = {"bed", "couch", "chair", "dining table", "refrigerator", "tv", "toilet", "oven", "sink", "microwave", "potted plant"}
DYNAMIC_LABELS = {"person", "bicycle", "car", "motorcycle", "bus", "truck", "bird", "cat", "dog", "horse", "sheep", "cow"}
MAX_RANGE = 6.0

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

def rotate(v, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*v[0]+s*v[1], -s*v[0]+c*v[1])

def add(a,b): return (a[0]+b[0],a[1]+b[1])
def sub(a,b): return (a[0]-b[0],a[1]-b[1])
def dot(a,b): return a[0]*b[0]+a[1]*b[1]
def norm(a): return math.hypot(*a)
def finite(values): return all(isinstance(v,(int,float)) and math.isfinite(v) for v in values)

def camera_origin(frame):
    p=frame["pose"]
    m=frame.get("camera_mount",[0,.133,-.1078])
    return add(p[:2],rotate((m[0],m[2]),p[2]))

def intrinsics(frame):
    w,h=frame["width"],frame["height"]
    f=.5*w/math.tan(math.radians(frame.get("hfov_deg",102))*.5)
    return f,w*.5,h*.5

def observation(frame,detection):
    f,cx,cy=intrinsics(frame)
    x,y,w,h=detection["bbox"]
    # Horizontal centre is a semantic object anchor, not the nearest surface.
    angle=wrap(frame["pose"][2]-math.atan2(x+w*.5-cx,f))
    return {"origin":camera_origin(frame),"direction":(-math.sin(angle),-math.cos(angle)),"angle":angle,
            "t":frame["timestamp_s"],"frame_id":frame["frame_id"],"box":detection["bbox"],
            "angular_width":abs(math.atan2(x+w-cx,f)-math.atan2(x-cx,f)),
            "score":detection["score"],"surface":lidar_surface_hint(frame,detection)}

def project_scan(frame, include_free_limit=False):
    """Project scan-time endpoints through capture-time estimated camera pose.

    The deployment adapter must supply gravity-aligned estimates. Uncompensated
    pitch/roll or time skew cause rejection rather than false pixel precision.
    """
    scan=frame.get("scan",{})
    if not isinstance(scan, dict) or not scan: return []
    timestamps = [frame.get("timestamp_s"), scan.get("timestamp_s")]
    if any(isinstance(t, bool) for t in timestamps) or not finite(timestamps) or min(timestamps) < 0: return []
    if abs(timestamps[0]-timestamps[1])>.12: return []
    tilt = [frame.get("roll_rad",0), frame.get("pitch_rad",0), scan.get("roll_rad",0), scan.get("pitch_rad",0)]
    if any(isinstance(t, bool) for t in tilt) or not finite(tilt) or max(abs(t) for t in tilt)>.025: return []
    pose=frame["pose"]; sp=scan.get("pose",[])
    if len(sp)!=3 or not finite(sp): return []
    ranges=scan.get("ranges_m",[])
    if not isinstance(ranges, (list, tuple)) or not 90 <= len(ranges) <= 720: return []
    # Historical captures omit sampling metadata. When supplied, it must agree
    # with the actual uniform full-circle scan; never reinterpret mismatched bins.
    if "ray_count" in scan and (isinstance(scan["ray_count"], bool) or scan["ray_count"] != len(ranges)): return []
    if "profile" in scan and scan["profile"] != f"uniform_instantaneous_{len(ranges)}": return []
    for key, expected in (("angle_increment_rad", 2*math.pi/len(ranges)), ("field_of_view_rad", 2*math.pi)):
        if key in scan:
            value = scan[key]
            if isinstance(value, bool) or not finite([value]) or not math.isclose(value, expected, rel_tol=1e-6, abs_tol=1e-8): return []
    cam=camera_origin(frame); mount=frame.get("camera_mount",[0,.133,-.1078])
    lidar=scan.get("mount",[0,.164,.062]); f,cx,cy=intrinsics(frame)
    out=[]
    for i,r in enumerate(ranges):
        if isinstance(r, bool) or not finite([r]) or not .15<=r<=MAX_RANGE: continue
        if r>=MAX_RANGE-.01 and not include_free_limit: continue
        a=i*2*math.pi/len(ranges)
        body=(lidar[0]-r*math.sin(a),lidar[2]-r*math.cos(a))
        point=add(sp[:2],rotate(body,sp[2]))
        local=rotate(sub(point,cam),-pose[2]); depth=-local[1]
        if depth<=.10: continue
        u=cx+f*local[0]/depth; v=cy-f*(lidar[1]-mount[1])/depth
        if 0<=u<frame["width"] and 0<=v<frame["height"]:
            out.append({"i":i,"u":u,"v":v,"point":point,"depth":depth,"r":r,"free_limit":r>=MAX_RANGE-.01})
    return out

def lidar_surface_hint(frame,detection):
    """Only isolated foreground clusters inside full 2D box yield a hint.

    A horizontal wall spanning the box is rejected. Even a good cluster could
    be an unrelated foreground occluder; multi-view bearings must corroborate.
    """
    x,y,w,h=detection["bbox"]
    rays=project_scan(frame)
    eligible=[p for p in rays if x+.12*w<=p["u"]<=x+.88*w and y+2<=p["v"]<=y+h-2]
    if len(eligible)<3: return None
    eligible.sort(key=lambda p:p["u"])
    clusters=[]
    for p in eligible:
        if not clusters or norm(sub(p["point"],clusters[-1][-1]["point"]))>.22:
            clusters.append([p])
        else: clusters[-1].append(p)
    clusters=[c for c in clusters if len(c)>=3]
    if not clusters: return None
    clusters.sort(key=lambda c:statistics.median(p["depth"] for p in c))
    c=clusters[0]; depth=statistics.median(p["depth"] for p in c)
    if max(p["depth"] for p in c)-min(p["depth"] for p in c)>.35: return None
    # Need measured farther returns immediately outside both bbox sides.
    # Missing neighbours/max-range are unknown, not background evidence.
    left=[p for p in rays if x-.3*w<=p["u"]<x]
    right=[p for p in rays if x+w<p["u"]<=x+1.3*w]
    if not left or not right: return None
    if min(statistics.median(p["depth"] for p in side) for side in [left,right])<depth+.30: return None
    return {"point":[statistics.median(p["point"][a] for p in c) for a in (0,1)],
            "depth_m":depth,"ray_count":len(c),"status":"unassigned_foreground_surface"}

def triangulate(obs):
    if len(obs)<4: return None
    baseline=max(norm(sub(a["origin"],b["origin"])) for a in obs for b in obs)
    parallax=max(abs(wrap(a["angle"]-b["angle"])) for a in obs for b in obs)
    if baseline<.35 or parallax<.09 or obs[-1]["t"]-obs[0]["t"]<2: return None
    # Least-squares perpendicular distances to rays, with angular uncertainty.
    a=b=c=d=e=0.0
    for o in obs:
        nx,nz=-o["direction"][1],o["direction"][0]
        q=nx*o["origin"][0]+nz*o["origin"][1]
        a+=nx*nx; b+=nx*nz; c+=nz*nz; d+=nx*q; e+=nz*q
    det=a*c-b*b
    if det<.003*(a+c)**2: return None
    point=((d*c-b*e)/det,(a*e-b*d)/det)
    distances=[dot(sub(point,o["origin"]),o["direction"]) for o in obs]
    if min(distances)<.4 or max(distances)>MAX_RANGE: return None
    angular=[abs(wrap(math.atan2(-(point[0]-o["origin"][0]),-(point[1]-o["origin"][1]))-o["angle"])) for o in obs]
    residuals=[abs((point[0]-o["origin"][0])*o["direction"][1]-(point[1]-o["origin"][1])*o["direction"][0]) for o in obs]
    if max(angular)>.05 or max(residuals)>.22: return None
    eigen_min=((a+c)-math.sqrt((a-c)**2+4*b*b))*.5
    # 1.5 degree image-centre + estimated pose drift + object's extended size.
    noise=.045+.027*statistics.mean(distances)
    radius=max(.18,2*noise/math.sqrt(max(.001,eigen_min)))
    extent=statistics.median(2*r*math.tan(o["angular_width"]*.5) for r,o in zip(distances,obs))
    if radius>.8: return None
    hints=[o["surface"] for o in obs if o["surface"]]
    corroborated=sum(norm(sub(h["point"],point))<max(.35,extent*.65)+radius for h in hints)
    # Unsupported surface depth NEVER substitutes for the visual intersection.
    return {"point":point,"uncertainty_m":radius,"object_extent_m":extent,"baseline_m":baseline,
            "parallax_deg":math.degrees(parallax),"residual_m":max(residuals),"surface_support":corroborated,
            "source":"multiview_bearings"+("+lidar_surface" if corroborated>=2 else "")}

@dataclass
class Track:
    id:int
    label:str
    observations:list=field(default_factory=list)
    appearance:list=field(default_factory=list)
    estimate:dict|None=None
    stable_estimates:int=0
    status:str="bearing_only"
    dynamic_until:float=0
    co_seen:set=field(default_factory=set)
    last_seen:float=0
    first_seen:float=0

class LandmarkMap:
    def __init__(self): self.reset()
    def reset(self):
        self.tracks=[]; self.next_id=1; self.last_frame=-1; self.epoch=None; self.last_timestamp=-1
    def _cost(self,t,o,appearance):
        if t.label in DYNAMIC_LABELS or o["t"]-t.last_seen>4: return None
        if t.estimate:
            p=t.estimate["point"]; delta=sub(p,o["origin"])
            expected=math.atan2(-delta[0],-delta[1]); angular=abs(wrap(expected-o["angle"]))
            gate=min(.18,.07+t.estimate["uncertainty_m"]/max(.5,norm(delta)))
        else:
            last=t.observations[-1]
            angular=abs(wrap(last["angle"]-o["angle"]))
            gate=.11+min(.12,norm(sub(last["origin"],o["origin"]))*1.2)
        if angular>gate:return None
        appearance_distance=sum(abs(a-b) for a,b in zip(appearance,t.appearance))*.5 if appearance and t.appearance else 0
        if appearance_distance>.38:return None
        return angular+appearance_distance*.12
    def update(self,frame,detections):
        # Epoch resets are explicit; delayed/inverted frames cannot mutate memory.
        if not isinstance(frame,dict) or not finite([frame.get("timestamp_s"),frame.get("width"),frame.get("height"),frame.get("hfov_deg",102)]) or not isinstance(frame.get("frame_id"),int):
            return {"error":"invalid_frame","landmarks":[],"detections":[]}
        if not isinstance(frame.get("pose"),list) or len(frame["pose"])!=3 or not finite(frame["pose"]) or not 20<frame.get("hfov_deg",102)<150 or min(frame["width"],frame["height"])<16:
            return {"error":"invalid_geometry","landmarks":[],"detections":[]}
        if not finite(frame.get("camera_mount",[0,.133,-.1078])) or not finite([frame.get("roll_rad",0),frame.get("pitch_rad",0)]):
            return {"error":"invalid_extrinsics","landmarks":[],"detections":[]}
        epoch=frame.get("epoch",0)
        if self.epoch is None:self.epoch=epoch
        if epoch!=self.epoch:return {"error":"wrong_epoch","landmarks":self.export(frame["timestamp_s"]),"detections":[]}
        now=frame["timestamp_s"]
        if frame["frame_id"]<=self.last_frame or now<self.last_timestamp:
            return {"error":"out_of_order","landmarks":self.export(now),"detections":[]}
        self.last_frame=frame["frame_id"];self.last_timestamp=now
        self.tracks=[t for t in self.tracks if now-t.last_seen<(60 if t.status=="confirmed" else 12)]
        used=set(); output=[]; seen=[]
        for det in sorted(detections,key=lambda d:d["score"],reverse=True):
            if not isinstance(det,dict) or not finite([det.get("score")]) or not .4<=det["score"]<=1 or not isinstance(det.get("label"),str):continue
            box=det.get("bbox",[])
            if len(box)!=4 or not finite(box) or min(box[2:])<8:continue
            det=dict(det);label=det["label"]
            if box[0]<0 or box[1]<0 or box[0]+box[2]>frame["width"] or box[1]+box[3]>frame["height"] or abs(frame.get("roll_rad",0))>.025 or abs(frame.get("pitch_rad",0))>.025:
                det.update({"status":"bearing_unusable","track_id":None});output.append(det);continue
            if label in DYNAMIC_LABELS or label not in STATIC_LABELS:
                det.update({"status":"dynamic" if label in DYNAMIC_LABELS else "detection_only","track_id":None});output.append(det);continue
            o=observation(frame,det); appearance=det.get("appearance",[])
            choices=[]
            for t in self.tracks:
                if t.label!=label or t.id in used:continue
                cost=self._cost(t,o,appearance)
                if cost is not None:choices.append((cost,t))
            choices.sort(key=lambda x:x[0])
            # Ambiguous same-class neighbours are not forcibly merged.
            t=choices[0][1] if choices and (len(choices)==1 or choices[1][0]-choices[0][0]>.025) else None
            if t is None:
                t=Track(self.next_id,label,appearance=appearance,first_seen=now);self.next_id+=1;self.tracks.append(t)
            used.add(t.id);seen.append(t)
            if t.estimate:
                pred=sub(t.estimate["point"],o["origin"])
                error=abs(wrap(math.atan2(-pred[0],-pred[1])-o["angle"]))
                if error>.10:
                    t.status="moving_or_inconsistent";t.dynamic_until=now+15;t.estimate=None;t.observations=[];t.stable_estimates=0
            t.last_seen=now
            sampled=not t.observations or now-t.observations[-1]["t"]>=.18
            if sampled:
                t.observations.append(o);t.observations=t.observations[-28:]
            estimate=triangulate(t.observations) if sampled else None
            if estimate:
                if t.estimate and norm(sub(t.estimate["point"],estimate["point"]))>max(.35,t.estimate["uncertainty_m"]):
                    t.status="moving_or_inconsistent";t.dynamic_until=now+15;t.stable_estimates=0;t.observations=t.observations[-2:];t.estimate=None
                else:
                    t.stable_estimates+=1;t.estimate=estimate
                    t.status="confirmed" if t.stable_estimates>=3 and now>=t.dynamic_until else "tentative"
            elif sampled and t.estimate and len(t.observations)>=6:
                # Contradictory new evidence withdraws static classification.
                t.status="moving_or_inconsistent";t.dynamic_until=now+15;t.estimate=None;t.stable_estimates=0;t.observations=t.observations[-3:]
            det.update({"status":t.status,"track_id":t.id,"range_hint":o["surface"] is not None});output.append(det)
        for a in seen:
            for b in seen:
                if a.id!=b.id:a.co_seen.add(b.id)
        self._merge_duplicates()
        return {"landmarks":self.export(now),"detections":output,"bearing_only_count":sum(t.estimate is None for t in self.tracks)}
    def _merge_duplicates(self):
        remove=set()
        for i,a in enumerate(self.tracks):
            if not a.estimate or a.id in remove:continue
            for b in self.tracks[i+1:]:
                if not b.estimate or b.id in remove or a.label!=b.label or b.id in a.co_seen or a.id in b.co_seen:continue
                distance=norm(sub(a.estimate["point"],b.estimate["point"]))
                app=sum(abs(x-y) for x,y in zip(a.appearance,b.appearance))*.5 if a.appearance and b.appearance else 1
                if distance<.3 and app<.15:
                    a.last_seen=max(a.last_seen,b.last_seen);a.co_seen|=b.co_seen;remove.add(b.id)
        self.tracks=[t for t in self.tracks if t.id not in remove]
    def export(self,now):
        out=[]
        for t in self.tracks:
            if not t.estimate or t.status not in ("confirmed","tentative"):continue
            e=t.estimate
            out.append({"id":t.id,"label":t.label,"x":e["point"][0],"z":e["point"][1],
                        "status":t.status if now-t.last_seen<5 else "stale","uncertainty_m":e["uncertainty_m"]+.008*max(0,now-t.last_seen),
                        "last_seen_s":t.last_seen,"object_extent_m":e["object_extent_m"],"epoch":self.epoch,"age_s":max(0,now-t.last_seen),"observations":len(t.observations),"source":e["source"],"source_frame_id":t.observations[-1]["frame_id"],"surface_support":e["surface_support"],
                        "baseline_m":e["baseline_m"],"parallax_deg":e["parallax_deg"],"residual_m":e["residual_m"]})
        return out
