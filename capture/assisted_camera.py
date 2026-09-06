"""Continuous camera acquisition, ChArUco quality, and lossless raw capture."""
from __future__ import annotations
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from board_config import ROBOT_BOARD
from shah_capture_client import BoardDetector, Camera


def finite_quality(quality):
    """Keep failed camera measurements as null; never sanitize robot commands."""
    invalid=[]
    def clean(value,path):
        if isinstance(value,dict):
            return {k:clean(v,path+'.'+str(k)) for k,v in value.items()}
        if isinstance(value,(list,tuple,np.ndarray)):
            return [clean(v,path+'[%d]' % i) for i,v in enumerate(value)]
        if isinstance(value,(float,np.floating)) and not np.isfinite(value):
            invalid.append(path)
            return None
        return value.item() if isinstance(value,np.generic) else value
    result=clean(quality,'quality')
    if invalid:
        result['nonfinite_fields']=invalid
        result['gate_reason']=';'.join(filter(None,[result.get('gate_reason'),'nonfinite_detection']))
        result['score']=0.
        charuco=result['charuco']
        charuco['ok']=False
        for key in ('rvec','tvec','T_cam_board_4x4','reproj_error_px'):
            charuco[key]=None
        charuco['corners_px']=[p for p in charuco.get('corners_px',[]) if all(v is not None for v in p)]
    return result


def analyze(frame, camera, detector, args):
    try:
        result=detector.detect(frame['color'],camera.K,camera.D)
        corners, ids=result.pop('_draw')
        xy=None if corners is None else np.asarray(corners).reshape(-1,2)
        result['corner_ids']=[] if ids is None else ids.ravel().tolist()
        result['corners_px']=[] if xy is None else xy.tolist()
        reasons=[]
        if not result['ok']:
            reasons.append('pose_failed')
        if result.get('failure_reason'):
            reasons.append(result['failure_reason'])
        if result['n_corners']<args.min_corners:
            reasons.append('too_few_corners')
        if ids is not None and len(ids)>0 and detector.grid.checkCharucoCornersCollinear(ids):
            reasons.append('collinear_corners')
        error=result.get('reproj_error_px')
        if error is not None and (not np.isfinite(error) or error>args.max_reproj_px):
            reasons.append('reprojection_error')
        T=result.get('T_cam_board_4x4')
        if T is not None and (not np.isfinite(T).all() or T[2][3]<=0):
            reasons.append('invalid_pose')
        h,w=frame['color'].shape[:2]
        margin=0. if xy is None else float(min(xy[:,0].min(),xy[:,1].min(),
                                                   w-1-xy[:,0].max(),h-1-xy[:,1].max()))
        gray=cv2.cvtColor(frame['color'],cv2.COLOR_BGR2GRAY)
        if xy is not None:
            x0,y0=np.maximum(0,np.floor(xy.min(axis=0)).astype(int))
            x1,y1=np.minimum([w,h],np.ceil(xy.max(axis=0)).astype(int)+1)
            roi=gray[y0:y1,x0:x1]
        else:
            roi=gray
        sharpness=float(cv2.Laplacian(roi,cv2.CV_64F).var()) if roi.size else 0.
        if margin<args.min_margin_px:
            reasons.append('near_image_edge')
        if sharpness<args.min_sharpness:
            reasons.append('blur')
        score=0.
        if result['ok'] and error is not None and np.isfinite(error) and 'invalid_pose' not in reasons:
            score=(result['n_corners']/ROBOT_BOARD.corner_count)*min(1.,max(0.,margin)/40.)/(1.+error)
        return finite_quality(dict(charuco=result, gate_reason=';'.join(dict.fromkeys(reasons)) or None,
                    detected_margin_px=margin, sharpness=sharpness, score=float(score)))
    except Exception as error:
        return dict(charuco={'ok':False,'n_corners':0,'T_cam_board_4x4':None,
                             'reproj_error_px':None,'corners_px':[],'corner_ids':[]},
                    gate_reason='detector_exception: '+repr(error),score=0.,
                    detected_margin_px=0.,sharpness=0.)


class SyntheticCamera:
    def __init__(self,name,width,height,order):
        self.name=name
        self.index=int(''.join(x for x in name if x.isdigit()))
        self.serial='SIMULATION_'+name
        self.K=np.array([[900.,0,width/2],[0,900.,height/2],[0,0,1.]])
        self.D=np.zeros(5)
        self.photometry={'locked':True,'source':'simulation'}
        self.width,self.height,self.order=width,height,order
        self.counter=0
        board=BoardDetector(ROBOT_BOARD,0).grid.generateImage((700,500))
        board=cv2.cvtColor(board,cv2.COLOR_GRAY2BGR)
        source=np.float32([[0,0],[699,0],[699,499],[0,499]])
        points=np.float32([[0,0,0],[.119,0,0],[.119,.085,0],[0,.085,0]])
        dest,_=cv2.projectPoints(points,np.array([.1+order*.05,-.15+order*.25,.03]),
                                 np.array([-.06+order*.01,-.04,.3]),self.K,self.D)
        dest=dest.reshape(4,2).astype(np.float32)
        self.image=cv2.warpPerspective(board,cv2.getPerspectiveTransform(source,dest),
                                       (width,height),borderValue=(230,230,230))

    def grab(self,timeout_ms=1000):
        time.sleep(1./15)
        self.counter+=1
        return dict(color=self.image.copy(),depth=None,ts_ms=self.counter*1000./15,
                    host_monotonic_ts_ms=time.monotonic()*1000.,device_ts_ms=None)

    def stop(self):
        pass


class Worker:
    def __init__(self,camera,args):
        self.camera,self.args=camera,args
        self.detector=BoardDetector(ROBOT_BOARD,0,args.detection_scale)
        self.lock=threading.Lock()
        self.done=threading.Event()
        self.latest=None
        self.error=None
        self.sequence=0
        self.thread=threading.Thread(target=self.run,daemon=True)

    def run(self):
        while not self.done.is_set():
            try:
                frame=self.camera.grab(timeout_ms=1000)
                if frame is None:
                    raise RuntimeError('No color frame')
                frame['color']=frame['color'].copy()
                quality=analyze(frame,self.camera,self.detector,self.args)
                with self.lock:
                    self.sequence+=1
                    self.latest=dict(frame=frame,quality=quality,sequence=self.sequence)
                    self.error=None
            except Exception as error:
                with self.lock:
                    self.error=repr(error)
                self.done.wait(0.1)

    def snapshot(self):
        with self.lock:
            if self.error:
                return None
            return self.latest


class CameraRig:
    def __init__(self,args):
        self.args=args
        self.cameras=[]
        self.workers=[]
        try:
            if args.simulate_cameras:
                self.cameras=[SyntheticCamera(n,args.width,args.height,i) for i,n in enumerate(args.cameras)]
            else:
                for order,name in enumerate(args.cameras):
                    with np.load(Path(args.intrinsics_dir)/(name+'.npz')) as d:
                        if (int(d['color_w']),int(d['color_h']))!=(args.width,args.height):
                            raise ValueError(name+': intrinsics resolution mismatch')
                        camera=Camera(name,int(''.join(c for c in name if c.isdigit())),
                                      str(d['serial']),np.array(d['color_K'],float),
                                      np.array(d['color_D'],float),args.width,args.height,
                                      args.fps,args.save_depth)
                    self.cameras.append(camera)
                    camera.start(stagger_s=args.startup_stagger_s if order else 0.)
            for camera in self.cameras:
                w=Worker(camera,args)
                self.workers.append(w)
                w.thread.start()
        except BaseException:
            self.close()
            raise

    def fresh(self,after,timeout=5.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            items={w.camera.name:w.snapshot() for w in self.workers}
            if all(x and x['frame']['host_monotonic_ts_ms']/1000.>after for x in items.values()):
                return items
            time.sleep(.02)
        return {w.camera.name:(s if (s:=w.snapshot()) and
                    s['frame']['host_monotonic_ts_ms']/1000.>after else None) for w in self.workers}

    def preview(self):
        for w in self.workers:
            s=w.snapshot()
            if s is None:
                view=np.zeros((360,640,3),np.uint8)
                text=w.camera.name+': '+(w.error or 'waiting for frame')
            else:
                view=s['frame']['color'].copy()
                q=s['quality']
                for x,y in q['charuco'].get('corners_px',[]):
                    cv2.circle(view,(round(x),round(y)),3,(0,255,0),-1)
                view=cv2.resize(view,(640,360))
                age=time.monotonic()-s['frame']['host_monotonic_ts_ms']/1000.
                text='%s corners=%d score=%.2f age=%.2fs' % (
                    w.camera.name,q['charuco']['n_corners'],q['score'],age)
                cv2.putText(view,q['gate_reason'] or 'PASS',(8,48),cv2.FONT_HERSHEY_SIMPLEX,
                            .45,(0,80,255) if q['gate_reason'] else (0,180,0),1)
            cv2.putText(view,text[:85],(8,23),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,100,0),1)
            cv2.imshow('ZEUS '+w.camera.name,view)
        return cv2.waitKey(10)&255

    def close(self):
        for w in self.workers:
            w.done.set()
        for w in self.workers:
            w.thread.join(2.)
        for c in self.cameras:
            c.stop()
