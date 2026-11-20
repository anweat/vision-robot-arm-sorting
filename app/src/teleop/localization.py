"""Read-only map-based localization. No motion imports or writes."""
import json
import sys
import threading
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
for sub in ('spatial','vision'):
    sys.path.insert(0,str(ROOT/'src'/sub))


_OFFICIAL=None
STICKER_MIN_CONF=.5


def official_detector():
    """The official 3-class garbage model on the NPU, created once; None where there is no Ascend runtime
    (the Mac simulation) or the model is missing, and then nothing else changes."""
    global _OFFICIAL
    if _OFFICIAL is None:
        try:
            import official_garbage
            _OFFICIAL=official_garbage.OfficialGarbage()
        except Exception as exc:   # ImportError, missing libascendcl, missing model
            _OFFICIAL=False
            print('official garbage model unavailable:',exc,flush=True)
    return _OFFICIAL or None


def attach_stickers(targets,dets,min_conf=STICKER_MIN_CONF):
    """Give each located block the best official detection covering at least 30 % of its box. Detections
    elsewhere (background, map markers) are dropped: they are the model's false positives."""
    for t in targets:
        x,y,w,h=t['bbox'];best=None
        for d in dets:
            x0,y0,x1,y1=d['bbox']
            inter=max(0,min(x+w,x1)-max(x,x0))*max(0,min(y+h,y1)-max(y,y0))
            if w*h and inter/(w*h)>=.3 and d['conf']>=min_conf and (best is None or d['conf']>best['conf']):
                best=d
        t['sticker']=None if best is None else dict(label=best['label'],category=best['category'],conf=best['conf'],bbox=best['bbox'])
    return targets


def locate_board(jpeg, servos):
    import cv2
    import numpy as np
    from map_registration import TableMap,detect_markers,load_intrinsics,solve_camera_pose
    from camera_model import ArmCamera
    import block_detector as bd
    import map_difference as md
    image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    if image is None or image.shape[:2]!=(480,640):
        raise ValueError('板载内参只适用于 640×480 画面')
    table=TableMap();k,dist=load_intrinsics()
    markers=detect_markers(image,table.layout)
    pose=solve_camera_pose(markers,table,k,dist,ArmCamera().pose_in_map(servos,table).centre)
    if pose is None:
        raise ValueError('没有可用地图码，停止输出方块坐标')
    if pose.reprojection_px>3:
        raise ValueError('地图重投影误差超过 3 px，不输出位置')
    blobs=md.detect_objects(image,pose,k,dist,table.layout,markers)
    # Blocks a few cm apart merge with their shadows into one map-difference blob (2026-09-26: red + green
    # + shadow read as one 'red', rejected for size, so neither was located). Oversized blobs are split
    # into the coloured patches inside them.
    # coloured pieces must lie on the map: the dark table around it has warm, saturated hues (2026-09-26),
    # and a brightness cut high enough to drop it also dropped a dark green top face
    corners=pose.project(np.array([[-.1485,0,0],[.1485,0,0],[.1485,.21,0],[-.1485,.21,0]]),k,dist).astype(np.float32)
    colour=[c for c in bd.detect_blobs(image,val=90,exclude=list(markers.values()),per_hue=True)
            if c.label in ('red','green','blue','yellow') and not c.partial
            and cv2.pointPolygonTest(corners.reshape(-1,1,2),(float(c.centroid_px[0]),float(c.centroid_px[1])),False)>=0]
    split,pieces=[],[]
    for b in list(blobs):   # a copy: pieces are added below and must not be split again
        try:bd.locate(b,pose,k,dist,.03)
        except ValueError:continue
        if b.size_m>.07:
            x0,y0,w0,h0=b.bbox
            inside=[c for c in colour if x0<=c.centroid_px[0]<=x0+w0 and y0<=c.centroid_px[1]<=y0+h0 and c not in pieces]
            if inside:
                split.append(b);pieces.extend(inside)
    blobs=[b for b in blobs if b not in split]+pieces
    targets,rejected=[],[]
    def reject(b,reason):
        rejected.append(dict(label=b.label,bbox=[int(v) for v in b.bbox],reason=reason))
    for b in split:
        reject(b,'粘连，已按颜色拆分')
    for b in blobs:
        x0,y0,w0,h0=b.bbox
        if b.partial or x0<8 or y0<8 or x0+w0>632 or y0+h0>472:
            reject(b,'贴画面边缘');continue
        try:bd.locate(b,pose,k,dist,.03)
        except ValueError:
            reject(b,'无法投影');continue
        if b.map_xy is None or not .02 <= b.size_m <= .07:
            reject(b,f'尺寸 {b.size_m*100:.1f} cm');continue
        x,y=b.map_xy
        if not (-.1485<=x<=.1485 and 0<=y<=.21):
            reject(b,'地图外');continue
        yaw=None
        if b.method=='top-face':   # square top face: its outline's rotation on the map, folded into [-45, 45)
            outline=np.array([pose.pixel_to_plane(p,k,dist,z=.03)[:2] for p in b.contour.reshape(-1,2)],np.float32)
            yaw=round((cv2.minAreaRect(outline)[2]+45)%90-45,1)
        # Classical ROI refinement (src/vision/roi_pose.py): the cube silhouette, not the colour blob, so
        # sticker-up blocks get a yaw too, and the centre is not dragged by a partial colour patch. It is
        # allowed to fail: the blob result above is the fallback.
        method,roi=b.method,None
        try:
            import roi_pose
            x0,y0,w0,h0=b.bbox
            roi=roi_pose.solve_top_face(image,[x0,y0,x0+w0,y0+h0],pose,k,dist,yaw,prior_xy=(x,y))
        except Exception as exc:
            roi=dict(ok=False,reason=str(exc))
        if roi.get('ok'):
            rx,ry=roi['map_cm'][0]/100,roi['map_cm'][1]/100
            shift=float(np.hypot(rx-x,ry-y))
            if not (-0.1485<=rx<=0.1485 and 0<=ry<=0.21):
                roi=dict(roi,ok=False,reason='refined centre off the map')
            elif shift>0.008:   # the colour blob is a strong prior; a fit this far away is a wrong lock
                roi=dict(roi,ok=False,reason=f'refined centre {shift*100:.1f} cm from the colour blob')
            else:
                x,y,yaw=rx,ry,roi['yaw_deg']
                method='top-face' if b.method=='top-face' else 'edges'
        targets.append(dict(label=b.label,map_cm=[round(x*100,2),round(y*100,2)],yaw_deg=yaw,
                            base_cm=(table.map_to_base([x,y,.03])*100).round(2).tolist(),
                            bbox=list(b.bbox),method=method,height_cm=3,roi=roi,
                            precision='ROI silhouette fit; map registration and the 3 cm block assumption apply'
                                      if roi and roi.get('ok') else
                                      'rough; tentative intrinsics, assumed 3 cm block and map placement'))
    detector=official_detector();garbage=[]
    if detector is not None:
        try:
            garbage=detector.detect(image,conf=.3)
            attach_stickers(targets,garbage)
        except Exception as exc:
            garbage=[dict(error=str(exc))]
    return dict(source='board',markers=sorted(markers),reprojection_px=round(pose.reprojection_px,2),
                official=garbage,
                objects=targets,rejected=rejected,size=[640,480],coordinates='map cm; base coordinates use unverified placement offset')


def gripper_map_points(servos):
    """Servo-5 centre, mid-gripper and tip projected onto the map plane (design placement), metres."""
    import dofbot_kinematics as kin
    from map_registration import TableMap
    if not servos or len(servos)<4 or any(v is None for v in servos[:4]):
        return []
    fk=kin.forward(list(servos)[:4]);table=TableMap()
    a,b=fk['servo5'],fk['tip']
    return [table.base_to_map([a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,0])[:2] for t in (0,.5,1)]


BLOCK_TOP_M=.03


def phone_camera(map_to_image,size):
    """Phone focal length and pose from the map homography alone (square pixels, principal point at
    the image centre, no distortion). Returns (f, R, C) with X_cam = R @ (X_map - C), or None when the
    view is too frontal to tell focal length from distance."""
    import numpy as np
    w,h=size
    g=np.array([[1,0,-w/2],[0,1,-h/2],[0,0,1.]])@map_to_image
    g1,g2,g3=g[:,0],g[:,1],g[:,2]
    if abs(g1[2]*g2[2])<1e-12:
        return None
    f2=-(g1[0]*g2[0]+g1[1]*g2[1])/(g1[2]*g2[2])   # r1 . r2 = 0
    if not 300**2<f2<6000**2:
        return None
    f=float(np.sqrt(f2));ki=np.diag([1/f,1/f,1.])
    lam=1/np.linalg.norm(ki@g1)
    r1,r2,t=ki@g1*lam,ki@g2*lam,ki@g3*lam
    u,_,vt=np.linalg.svd(np.column_stack([r1,r2,np.cross(r1,r2)]));R=u@vt
    centre=-R.T@t
    if centre[2]<0:   # the sign of the homography scale: pick the camera above the table
        r1,r2,t=-r1,-r2,-t
        u,_,vt=np.linalg.svd(np.column_stack([r1,r2,np.cross(r1,r2)]));R=u@vt
        centre=-R.T@t
    if not .1<centre[2]<3:
        return None
    return f,R,centre


def locate_phone(jpeg,servos=None):
    import cv2
    import numpy as np
    from map_registration import TableMap,detect_markers
    import block_detector as bd
    image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise ValueError('无法解码手机画面')
    table=TableMap();markers=detect_markers(image,table.layout)
    if len(markers)<3:raise ValueError('手机画面至少需要 3 个完整地图码')
    src=np.vstack(list(markers.values())).astype(float)
    dst=np.vstack([table.layout[m] for m in markers]).astype(float)
    h,mask=cv2.findHomography(src,dst,cv2.RANSAC,.003)
    if h is None or mask.sum()<.75*len(src):raise ValueError('手机地图配准不稳定')
    def project(p):return cv2.perspectiveTransform(np.asarray(p,float).reshape(-1,1,2),h).reshape(-1,2)
    residual=np.linalg.norm(project(src)-dst,axis=1)
    rms=float(np.sqrt(np.mean(residual[mask.ravel().astype(bool)]**2))*1000)
    if rms>3:raise ValueError('手机地图拟合误差超过 3 mm')
    # The coloured patch is a block's top face, 3 cm above the table. Projecting it onto the table
    # itself pushed every block 2-3 cm away from the phone (2026-09-26, blocks on marker centres);
    # intersecting the pixel ray with z = 3 cm instead left 0.0-0.5 cm (mean 0.27 cm).
    cam=phone_camera(np.linalg.inv(h),(image.shape[1],image.shape[0]))
    def locate(uv):
        if cam is None:
            return project([uv])[0]
        f,R,c=cam
        d=R.T@np.array([(uv[0]-image.shape[1]/2)/f,(uv[1]-image.shape[0]/2)/f,1.])
        return (c+d*(BLOCK_TOP_M-c[2])/d[2])[:2]
    objects,rejected=[],[]
    # The phone has no pose for the arm, so colour on the gripper (its red servo housing) looks like a
    # block. Anything within 4 cm of where kinematics puts the gripper is reported separately.
    near=gripper_map_points(servos)
    for b in bd.detect_blobs(image,min_area=500,exclude=list(markers.values())):
        if b.partial:
            rejected.append(dict(label=b.label,bbox=list(b.bbox),reason='贴画面边缘',draw=False));continue
        q=locate(b.centroid_px);outline=project(b.contour.reshape(-1,2))
        area=abs(cv2.contourArea(outline.astype(np.float32)))
        on_map=bool(-.1485<=q[0]<=.1485 and 0<=q[1]<=.21)
        if not on_map:
            rejected.append(dict(label=b.label,bbox=list(b.bbox),reason='地图外',draw=False));continue
        if not .0002<=area<=.003:
            rejected.append(dict(label=b.label,bbox=list(b.bbox),reason=f'尺寸 {area*1e4:.0f} cm²',
                                 map_cm=(q*100).round(2).tolist()));continue
        if near and min(float(np.hypot(*(q-np.asarray(p)))) for p in near)<.04:
            rejected.append(dict(label=b.label,bbox=list(b.bbox),reason='夹爪附近（可能是夹爪或被夹物块）',
                                 map_cm=(q*100).round(2).tolist()));continue
        objects.append(dict(label=b.label,map_cm=(q*100).round(2).tolist(),bbox=list(b.bbox),
                            method='colour-top-face-3cm' if cam else 'colour-patch-table-projection',
                            height_corrected=cam is not None,
                            precision='~0.3 cm when the coloured face is on top (2026-09-26 check); '
                                      '~1.5 cm off if it is a side face' if cam else
                                      'uncorrected: 2-3 cm away from the phone for 3 cm blocks'))
    return dict(source='phone',markers=sorted(markers),map_fit_rms_mm=round(rms,2),objects=objects,rejected=rejected,
                size=[image.shape[1],image.shape[0]],
                phone_camera=None if cam is None else dict(focal_px=round(cam[0],1),centre_map_cm=(cam[2]*100).round(1).tolist()),
                coordinates='top face of 3 cm blocks projected with the phone pose from the map' if cam else
                            'table projection only; not grasp coordinates')


def annotate(image,result):
    """Accepted objects in yellow with their map position; rejected candidates in grey with the reason.
    Text scales with the image so 1920-wide phone frames stay readable after downscaling. Labels are
    ASCII (OpenCV's Hershey fonts have no CJK); the reasons are in the JSON."""
    import cv2
    scale=max(.75,image.shape[1]/1280);thick=max(1,round(scale*1.5))
    placed=[]
    def label(text,box,colour):
        x,y0,bw,bh=box
        (w,h),_=cv2.getTextSize(text,cv2.FONT_HERSHEY_SIMPLEX,scale*.6,thick)
        x=max(0,min(x,image.shape[1]-w-6))
        # above the box, else below it, else keep stepping down until it clears earlier labels
        for y in [y0-6,y0+bh+h+10]+[y0+bh+h+10+k*(h+12) for k in range(1,8)]:
            y=max(h+6,y);r=(x,y-h-6,x+w+6,y+4)
            if not any(r[0]<q[2] and q[0]<r[2] and r[1]<q[3] and q[1]<r[3] for q in placed):break
        placed.append(r)
        cv2.rectangle(image,r[:2],r[2:],(0,0,0),-1)
        cv2.putText(image,text,(x+3,y),cv2.FONT_HERSHEY_SIMPLEX,scale*.6,colour,thick)
    reasons={'贴画面边缘':'edge','无法投影':'no-proj','地图外':'off-map','地图外或尺寸不符':'off-map/size'}
    shown=[r for r in result.get('rejected',[]) if r.get('draw',True)]  # phone clutter off the map stays in JSON only
    for r in shown:
        x,y,w,h=r['bbox'];cv2.rectangle(image,(x,y),(x+w,y+h),(150,150,150),max(1,thick-1))
    for i,o in enumerate(result.get('objects',[])):
        x,y,w,h=o['bbox'];cv2.rectangle(image,(x,y),(x+w,y+h),(0,220,255),thick+1)
    # accepted labels are placed first so they keep the spot directly above their box
    for i,o in enumerate(result.get('objects',[])):
        st=o.get('sticker')
        tag=f" [{st['label']} {st['conf']:.2f}]" if st else ''
        label(f"{i+1} {o['label']} ({o['map_cm'][0]:.1f}, {o['map_cm'][1]:.1f}) cm{tag}",o['bbox'],(0,220,255))
    for r in shown:
        why='near gripper' if r['reason'].startswith('夹爪') else reasons.get(r['reason'],'size' if r['reason'].startswith('尺寸') else 'rejected')
        label(f"x {r['label']} {why}",r['bbox'],(170,170,170))
    return image


class Locator:
    def __init__(self,control,camera,phone,directory):
        self.control,self.camera,self.phone=control,camera,phone
        self.directory=Path(directory)
        self.lock=threading.Lock()
        self.result={'board':{'error':'定位初始化中'},'phone':{'error':'定位初始化中'}}
        self.images={}
        self.closed=threading.Event()

    def snapshot(self):
        with self.lock:
            result=json.loads(json.dumps(self.result))
        for value in result.values():
            value['age_s']=time.time()-value.get('t',0)
            if value['age_s']>4:
                value['error']='定位结果过期'
                value['objects']=[]
        return result

    def run(self):
        while not self.closed.is_set():
            state=self.control.status()
            for name,camera,fn in [('board',self.camera,locate_board),('phone',self.phone,locate_phone)]:
                if camera is None:continue
                try:
                    jpeg,meta=camera.snapshot()
                    if not jpeg or meta['kind']!='live' or meta['age_s'] is None or not 0<=meta['age_s']<2:
                        raise ValueError('画面不可用或过期')
                    if name=='board' and (state['mode']!='hardware' or state['busy'] or state['age_s'] is None or state['age_s']>2):
                        raise ValueError('需要静止的实机新鲜回读')
                    result=fn(jpeg,state['pose']) if name=='board' else fn(jpeg,state['pose'] if state['age_s'] is not None and state['age_s']<2 else None)
                    after=self.control.status()
                    # readbacks flicker by a degree at rest; only a real move invalidates the result
                    moved=any(abs(a-b)>1 for a,b in zip(after['pose'],state['pose']) if a is not None and b is not None)
                    if name=='board' and (after['busy'] or after['revision']!=state['revision'] or moved):
                        raise ValueError('计算期间姿态发生变化，结果已丢弃')
                    result.update(t=time.time(),frame=meta,servos=state['pose'] if name=='board' else None)
                    import cv2
                    import numpy as np
                    image=annotate(cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR),result)
                    ok,buf=cv2.imencode('.jpg',image)
                    with self.lock:
                        self.result[name]=result
                        self.images[name]=buf.tobytes() if ok else jpeg
                except Exception as e:
                    failed=dict(error=str(e),t=time.time(),objects=[])
                    if name=='board' and 'jpeg' in locals() and jpeg and official_detector() is not None:
                        try:   # a view without markers still gets the official detections (the scan uses them)
                            import cv2
                            import numpy as np
                            failed['official']=official_detector().detect(cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR),conf=.3)
                            failed['servos']=state['pose']
                        except Exception:
                            pass
                    with self.lock:
                        self.result[name]=failed
                        self.images.pop(name,None)
            self.closed.wait(1)

    def save(self):
        with self.lock:
            record=self.snapshot_unlocked()
            ident=str(time.time_ns())
            for name,jpeg in self.images.items():
                file=f'localization_{ident}_{name}.jpg'
                (self.directory/file).write_bytes(jpeg)
                record[name]['annotated_image']=file
            (self.directory/f'localization_{ident}.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
            return record

    def snapshot_unlocked(self):
        return json.loads(json.dumps(self.result))
