from pathlib import Path
import collections,json,sys
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from types import SimpleNamespace
from capture.shah_capture_client import BoardDetector
from assess_dataset import dist,err,avg,inv,validT
OUT=Path(__file__).parent
results={}; all_tables={};observations={};checked=0;bad_images=[]
for mp in sorted((ROOT/'datasets/260910').rglob('meta.json')):
 m=json.loads(mp.read_text()); board=SimpleNamespace(**m['board_config']); table_config=SimpleNamespace(**dict(m['table_board_config'],legacy_pattern=m['table_board_config'].get('legacy_pattern',False)))
 robot=BoardDetector(board,board.marker_id_start);table=BoardDetector(table_config,table_config.marker_id_start)
 cases=[];image_points={}
 for cap in m['captures']:
  for cam in m['cam_indices']:
   c=cap['cams'][str(cam)];eid=cap['event_id'];intr=np.load(ROOT/'intrinsics'/f'cam{cam}.npz')
   assert str(intr['serial'])==m['cam_serials'][str(cam)]
   K=intr['color_K'].astype(float);D=intr['color_D'].astype(float)
   f=mp.parent/c['rgb_path'];img=cv2.imread(str(f))
   depth=cv2.imread(str(mp.parent/c['depth_path']),cv2.IMREAD_UNCHANGED) if c.get('depth_path') else None
   if img is None or depth is None:
    bad_images.append(str(f));continue
   assert img.shape[:2]==(720,1280) and depth.shape==(720,1280)
   checked+=1
   gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
   det=robot.detect(img,K,D)
   corners,cids=det.pop('_draw')
   old=c['charuco'];oldpx=old.get('reproj_error_px')
   case={'event_id':eid,'cam':cam,'stored_corners':old.get('n_corners'),
         'redetected_corners':det['n_corners'],'stored_reproj':oldpx if oldpx is not None and np.isfinite(oldpx) else None,
         'redetected_reproj':det.get('reproj_error_px') if det.get('reproj_error_px') is not None and np.isfinite(det['reproj_error_px']) else None,
         'stored_gate_pass':cap['capture_gate']['pass']}
   if corners is not None:
    image_points[f'{eid}:{cam}']={'ids':cids.ravel().tolist(),'pixels':corners.reshape(-1,2).tolist()}
   if validT(old.get('T_cam_board_4x4')):
    T=np.array(old['T_cam_board_4x4']);obj=np.array([[0,0,0],[board.size_mm[0]/1000,0,0],[board.size_mm[0]/1000,board.size_mm[1]/1000,0],[0,board.size_mm[1]/1000,0]],np.float32)
    pix=cv2.projectPoints(obj,cv2.Rodrigues(T[:3,:3])[0],T[:3,3],K,D)[0].reshape(-1,2)
    case['predicted_board_clipped']=bool(np.any((pix[:,0]<0)|(pix[:,0]>=1280)|(pix[:,1]<0)|(pix[:,1]>=720)))
    case['board_tilt_deg']=float(np.degrees(np.arccos(np.clip(abs(T[2,2]),0,1))))
   tc,ti,_,_=table.detector.detectBoard(gray)
   if tc is not None:
    all_tables.setdefault((mp.parent.name,cam),[]).append({int(i):p.tolist() for i,p in zip(ti.ravel(),tc.reshape(-1,2))})
   cases.append(case)
 results[mp.parent.name]=cases;observations[mp.parent.name]=image_points
 print(mp.parent.name, 'RGB/depth read:',len(cases),flush=True)
# Fixed table-board pixel coordinates are a check for gross scene movement.
table_summary={}; centers={}
for (session,cam),views in all_tables.items():
 points=collections.defaultdict(list)
 for v in views:
  for i,p in v.items():points[i].append(p)
 center={i:np.median(p,axis=0) for i,p in points.items() if len(p)>=max(3,len(views)*0.5)}
 centers[(session,cam)]=center
 residuals=[np.linalg.norm(np.asarray(p)-center[i]) for v in views for i,p in v.items() if i in center]
 table_summary[f'{session}:cam{cam}']={'frames':len(views),'stable_corners':len(center),'within_session_corner_shift_px':dist(residuals)}
comparisons={}
keys=list(centers)
for a in range(len(keys)):
 for b in range(a):
  ka,kb=keys[a],keys[b]
  if ka[1]!=kb[1]:continue
  common=set(centers[ka])&set(centers[kb])
  comparisons[f'cam{ka[1]}:{ka[0]} vs {kb[0]}']={'common_corners':len(common),'median_corner_displacement_px':dist([np.linalg.norm(centers[ka][i]-centers[kb][i]) for i in common])}
result={'rgb_count':checked,'depth_count':checked,'bad_images':bad_images,'sessions':results,'table_stability':table_summary,'table_cross_sessions':comparisons}
(OUT/'image_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False))
(OUT/'image_points.json').write_text(json.dumps(observations,separators=(',',':'),allow_nan=False))
for session,cases in results.items():
 print(session,flush=True)
 for cam in sorted({c['cam'] for c in cases}):
  cs=[c for c in cases if c['cam']==cam];failed=[c for c in cs if not c['stored_gate_pass']]
  print(' cam',cam,'clipped all',sum(c.get('predicted_board_clipped',False) for c in cs),'/',len(cs),
        'reproj abs delta median',dist([abs(c['redetected_reproj']-c['stored_reproj']) for c in cs if c['redetected_reproj'] is not None and c['stored_reproj'] is not None]),flush=True)
print('table cross-session',json.dumps(comparisons),flush=True)
