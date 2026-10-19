#!/usr/bin/env python3
"""Offline CPU vision worker. Single request/result file pair, bounded backpressure.
No sockets, API keys, network, simulator labels, depth buffer or truth poses.
"""
from __future__ import annotations
import argparse,faulthandler,json,math,os,sys,time,traceback
from pathlib import Path

PATCH_VERSION='v07.3-person-tracking'
_diagnostic_stream=None

def retry_file_operation(operation,timeout_s=1.0):
    """Windows readers can briefly deny rename/delete; keep retries bounded.

    Retrying runs only in the isolated worker, never in the Godot control loop.
    Other I/O failures must remain visible rather than being silently ignored.
    """
    deadline=time.monotonic()+timeout_s
    while True:
        try:return operation()
        except PermissionError:
            if time.monotonic()>=deadline:raise
            time.sleep(.01)

def write_json(path,data):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data,allow_nan=False),encoding='utf-8')
    retry_file_operation(lambda:os.replace(temporary,path))

def consume_request(path):
    """Retire the mailbox after closing its handle, before inference starts.

    The producer allows exactly one in-flight frame and cannot publish the next
    request until it receives this one's result. Leaving the old request in place
    made Windows DeleteFile/MoveFile race our polling reader on every frame.
    """
    frame=json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(frame,dict) or 'frame_id' not in frame:
        raise ValueError('Vision request lacks frame identity')
    try:retry_file_operation(path.unlink)
    except OSError as exc:
        # Do not publish a result or retry this deletion after completion: the
        # producer would then be allowed to create a different request here.
        raise RuntimeError('Cannot retire vision request: '+str(exc)) from exc
    return frame

def trace_event(stage,**details):
    if _diagnostic_stream is None:return
    # Keep one local text-only log. Never log images, keys, scan or pose payloads.
    try:
        if _diagnostic_stream.tell()>1024*1024:
            _diagnostic_stream.seek(0);_diagnostic_stream.truncate()
        print(json.dumps({'time':time.time(),'stage':stage,**details}),file=_diagnostic_stream,flush=True)
    except OSError:pass

def _configure_diagnostics():
    global _diagnostic_stream
    if __name__!='__main__' or '--ipc' not in sys.argv:return
    try:
        ipc=Path(sys.argv[sys.argv.index('--ipc')+1])
        _diagnostic_stream=(ipc.parent/'perception_worker_latest.log').open('w',encoding='utf-8',buffering=1)
        sys.stdout=sys.stderr=_diagnostic_stream
        # pythonw has no console. Capture native stderr where the CRT allows it.
        for descriptor in (1,2):
            try:os.dup2(_diagnostic_stream.fileno(),descriptor)
            except OSError:pass
        faulthandler.enable(file=_diagnostic_stream,all_threads=True)
        original_hook=sys.excepthook
        def report_unhandled(exc_type,exc,tb):
            trace_event('unhandled_exception',error=exc_type.__name__+': '+str(exc))
            original_hook(exc_type,exc,tb)
            try:write_json(ipc/'ready.json',{'ok':False,'error':exc_type.__name__+': '+str(exc)})
            except OSError:pass
        sys.excepthook=report_unhandled
        trace_event('boot',patch=PATCH_VERSION,platform=sys.platform,pid=os.getpid(),python=sys.version)
    except (OSError,IndexError):
        # Logging is diagnostic only and must never prevent normal inference.
        pass

_configure_diagnostics()
# Set before numeric/MediaPipe imports. These only control supporting libraries;
# MediaPipe internal TFLite thread count is not exposed in this Tasks API.
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ.setdefault(key,'1')
from compute_profile import load_profile,configure_worker_limits,Scheduler
_profile_name='pi4_budget'
if __name__=='__main__' and '--profile' in sys.argv:_profile_name=sys.argv[sys.argv.index('--profile')+1]
_profile=load_profile(_profile_name)
_limits=configure_worker_limits(_profile) if __name__=='__main__' else {}
import tempfile
os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'field4-mpl-cache'))
os.environ.setdefault('XDG_CACHE_HOME',str(Path(tempfile.gettempdir())/'field4-vision-cache'))
import numpy as np
from PIL import Image as PillowImage
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from landmarks import LandmarkMap
from gesture import classify_pose
from person_evidence import make_person_evidence
from person_tracking import PersonTracker
from person_detector_bridge import PersonDetectorBridge
trace_event('imports_ready',mediapipe=mp.__version__,profile=_profile.name,limits=_limits)

ROOT=Path(__file__).resolve().parent

def load_frame_image(path):
    # Open through Python's Unicode-aware filesystem layer. MediaPipe's native
    # filename API passes UTF-8 to stb's file loader, whose Windows filename
    # support depends on its build flags/code page. Godot's own app-data path
    # contains a middle dot, so passing decoded pixels avoids that dependency.
    # Pillow is already a pinned, bundled dependency; no new install is needed.
    with PillowImage.open(path) as source:
        pixels=np.ascontiguousarray(np.asarray(source.convert('RGB'),dtype=np.uint8))
    return mp.Image(image_format=mp.ImageFormat.SRGB,data=pixels)

def appearance(rgb,box):
    h,w=rgb.shape[:2];x,y,bw,bh=box
    patch=rgb[max(0,y):min(h,y+bh),max(0,x):min(w,x+bw)]
    if patch.size==0:return []
    # Coarse RGB histogram assists association only. It never sets a class.
    values=[]
    for ch in range(3):
        hist=np.histogram(patch[:,:,ch],bins=8,range=(0,256))[0].astype(float)
        values.extend((hist/max(1,hist.sum())/3).tolist())
    return values

class Perception:
    def __init__(self):
        base=lambda p:python.BaseOptions(model_asset_path=str(ROOT/'models'/p),delegate=python.BaseOptions.Delegate.CPU)
        trace_event('loading_object_model')
        self.detector=vision.ObjectDetector.create_from_options(vision.ObjectDetectorOptions(base_options=base('efficientdet_lite0.tflite'),running_mode=vision.RunningMode.IMAGE,max_results=12,score_threshold=.4))
        trace_event('loading_pose_model')
        self.pose=vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(base_options=base('pose_landmarker_lite.task'),running_mode=vision.RunningMode.IMAGE,num_poses=3,min_pose_detection_confidence=.45,min_pose_presence_confidence=.5,min_tracking_confidence=.5,output_segmentation_masks=False))
        self.fusion=LandmarkMap();self.person_tracker=PersonTracker();self.person_detector_bridge=PersonDetectorBridge();self.epoch=None
        self.last_input_frame=-1;self.last_input_time=-1.;self.last_input_capture=-1.
    def infer(self,image_path,frame,run_objects=True,run_pose=True):
        # Reject delayed/duplicate work before loading images, invoking models,
        # or resetting either map. Capture-time freshness at delivery is still
        # enforced by Godot; the two processes do not compare unrelated clocks.
        def reject(reason):
            source=frame if isinstance(frame,dict) else {}
            number=lambda key,default:source.get(key,default) if isinstance(source.get(key,default),(int,float)) and math.isfinite(source.get(key,default)) else default
            return {'ok':False,'error':reason,'frame_id':number('frame_id',-1),'epoch':number('epoch',-1),
                    'timestamp_s':number('timestamp_s',0),'capture_monotonic_ms':number('capture_monotonic_ms',0),
                    'poses':[],'person_observations':[],'detections':[],'landmarks':[],
                    'person_tracks':{'status':reason,'tracks':[]},'objects_ran':False,'pose_ran':False,
                    'timings_ms':{'object':0.,'pose':0.,'total':0.}}
        if not isinstance(frame,dict) or type(frame.get('frame_id')) is not int or type(frame.get('epoch',0)) is not int:
            return reject('invalid_frame')
        incoming_epoch=frame.get('epoch',0)
        if min(frame['frame_id'],incoming_epoch)<0 or any(isinstance(frame.get(key),bool) or not isinstance(frame.get(key),(int,float)) or not math.isfinite(frame[key]) or frame[key]<0 for key in ('timestamp_s','capture_monotonic_ms')):
            return reject('invalid_frame')
        if self.epoch is not None and incoming_epoch<self.epoch:
            return reject('wrong_epoch')
        if self.epoch==incoming_epoch and (frame['frame_id']<=getattr(self,'last_input_frame',-1) or frame['timestamp_s']<=getattr(self,'last_input_time',-1.) or frame['capture_monotonic_ms']<=getattr(self,'last_input_capture',-1.)):
            return reject('out_of_order')
        if self.epoch!=incoming_epoch:
            self.fusion.reset();self.person_detector_bridge=PersonDetectorBridge();self.epoch=incoming_epoch
        self.last_input_frame=frame['frame_id'];self.last_input_time=frame['timestamp_s'];self.last_input_capture=frame['capture_monotonic_ms']
        trace_event('loading_frame',frame_id=frame.get('frame_id'))
        start=time.monotonic();image=load_frame_image(image_path);rgb=image.numpy_view()
        if rgb.shape[:2]!=(frame['height'],frame['width']):raise ValueError('Frame resolution mismatch')
        trace_event('object_inference',frame_id=frame.get('frame_id'))
        a=time.monotonic();objects=self.detector.detect(image) if run_objects else None;b=time.monotonic()
        trace_event('pose_inference',frame_id=frame.get('frame_id'))
        pose_start=time.monotonic()
        poses=self.pose.detect(image) if run_pose else None;c=time.monotonic()
        trace_event('fusion',frame_id=frame.get('frame_id'))
        detections=[]
        for d in objects.detections if objects else []:
            if not d.categories:continue
            category=d.categories[0];box=d.bounding_box
            bbox=[box.origin_x,box.origin_y,box.width,box.height]
            detections.append({'label':category.category_name,'score':category.score,'bbox':bbox,'appearance':appearance(rgb,bbox)})
        fused=self.fusion.update(frame,detections)
        pose_results=[]
        for landmarks in poses.pose_landmarks if poses else []:
            points=[{'x':p.x,'y':p.y,'visibility':p.visibility,'presence':p.presence} for p in landmarks]
            result=classify_pose(points);result['points']=points
            result['observation_source']='pose_landmarks'
            result['follow_evidence']=make_person_evidence(points,frame,rgb,appearance)
            pose_results.append(result)
        person_observations=pose_results[:]
        if run_pose and not pose_results:
            person_observations=self.person_detector_bridge.continuations(frame,detections,self.person_tracker,rgb,appearance)
        person_tracks=self.person_tracker.update(frame,person_observations,pose_ran=run_pose)
        self.person_detector_bridge.remember(frame,pose_results,detections,self.person_tracker)
        for d in fused.get('detections',[]):d.pop('appearance',None)
        return {**fused,'poses':pose_results,'person_observations':person_observations,'person_tracks':person_tracks,'frame_id':frame['frame_id'],'epoch':frame.get('epoch',0),
                'timestamp_s':frame['timestamp_s'],'capture_monotonic_ms':frame.get('capture_monotonic_ms',0),
                'objects_ran':run_objects,'pose_ran':run_pose,'model':'MediaPipe EfficientDet-Lite0 INT8 + Pose Lite CPU',
                'timings_ms':{'object':(b-a)*1000,'pose':(c-pose_start)*1000,'total':(time.monotonic()-start)*1000},'ok':True}
    def close(self):self.detector.close();self.pose.close()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--ipc');parser.add_argument('--image');parser.add_argument('--frame');parser.add_argument('--output');parser.add_argument('--profile',default='pi4_budget');args=parser.parse_args()
    ipc=Path(args.ipc) if args.ipc else None
    engine=None
    try:
        engine=Perception()
        if args.image:
            frame=json.loads(Path(args.frame).read_text(encoding='utf-8'));result=engine.infer(args.image,frame)
            if args.output:write_json(Path(args.output),result)
            else:print(json.dumps(result))
            return
        if ipc is None:raise ValueError('--ipc or --image required')
        write_json(ipc/'ready.json',{'ok':True,'runtime':mp.__version__,'profile':_profile.as_dict(),'limits':_limits})
        trace_event('ready')
        scheduler=Scheduler(_profile)
        last_id=(-1,-1);last_contact=time.monotonic()
        while not (ipc/'stop').exists():
            if time.monotonic()-last_contact>90:break
            request=ipc/'request.json'
            if not request.exists():
                if time.monotonic()-last_contact>90:break
                time.sleep(.01);continue
            # Read only when scheduling can accept the frame. A consumed request
            # must never be discarded just because the cadence is not ready yet.
            if not scheduler.can_start(time.monotonic()):time.sleep(.01);continue
            try:frame=consume_request(request)
            except (OSError,ValueError):time.sleep(.01);continue
            identity=(frame.get('epoch',0),frame['frame_id'])
            if identity==last_id:time.sleep(.01);continue
            last_id=identity;last_contact=time.monotonic()
            trace_event('request_consumed',frame_id=frame['frame_id'],epoch=frame.get('epoch',0))
            job=scheduler.start(frame_id=frame['frame_id'])
            try:
                if _diagnostic_stream is not None:faulthandler.dump_traceback_later(10,file=_diagnostic_stream)
                result=engine.infer(ipc/'frame.png',frame,run_objects=job['run_objects'],run_pose=job['run_pose'])
                tm=result['timings_ms']
                result['compute']=scheduler.finish({'inference_ms':tm['object']+tm['pose'],'object_inference_ms':tm['object'],'pose_inference_ms':tm['pose'],'object_ran':job['run_objects'],'pose_ran':job['run_pose']})
            except Exception as exc:
                trace_event('inference_error',error=type(exc).__name__+': '+str(exc))
                traceback.print_exc()
                if scheduler.active:scheduler.finish({'inference_ms':0,'object_ran':False,'pose_ran':False})
                result={'ok':False,'frame_id':frame.get('frame_id'), 'epoch':frame.get('epoch',0),'error':type(exc).__name__+': '+str(exc)}
            finally:
                if _diagnostic_stream is not None:faulthandler.cancel_dump_traceback_later()
            write_json(ipc/'result.json',result)
            trace_event('result_published',frame_id=frame['frame_id'],ok=result['ok'])
    except Exception as exc:
        trace_event('worker_error',error=type(exc).__name__+': '+str(exc))
        traceback.print_exc()
        if ipc:write_json(ipc/'ready.json',{'ok':False,'error':type(exc).__name__+': '+str(exc)})
        else:raise
    finally:
        if engine:engine.close()
        trace_event('stopped')
if __name__=='__main__':main()
