from pathlib import Path
import json,sys
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from assess_dataset import inv,fit,dist
OUT=Path(__file__).parent
j=json.loads((OUT/'assessment.json').read_text()); obs=json.loads((OUT/'image_points.json').read_text())
for name,s in j['sessions'].items():
 m=json.loads(Path(s['source']).read_text());cs=[c for c in m['captures'] if c['event_id'] in s['eligible_ids']]
 d=m['board_config'];dict_=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,d['dictionary_name']))
 grid=cv2.aruco.CharucoBoard((d['squares_x'],d['squares_y']),d['square_length_m'],d['marker_length_m'],dict_,np.arange(d['marker_id_start'],d['marker_id_start']+d['marker_count'],dtype=np.int32).reshape(-1,1));grid.setLegacyPattern(d['legacy_pattern'])
 obj=grid.getChessboardCorners();A=[np.array(c['robot_pose_matrix_4x4']) for c in cs]
 for cam in m['cam_indices']:
  intr=np.load(ROOT/'intrinsics'/f'cam{cam}.npz');K=intr['color_K'];D=intr['color_D']
  B=[np.array(c['cams'][str(cam)]['charuco']['T_cam_board_4x4']) for c in cs]
  rms=[];per=[]
  for i,c in enumerate(cs):
   keep=[k for k in range(len(cs)) if k!=i]
   X,Y=fit([A[k] for k in keep],[B[k] for k in keep])
   predicted=inv(Y)@A[i]@X
   pts=obs[name].get(f"{c['event_id']}:{cam}")
   if not pts:continue
   projected=cv2.projectPoints(obj[pts['ids']],cv2.Rodrigues(predicted[:3,:3])[0],predicted[:3,3],K,D)[0].reshape(-1,2)
   error=float(np.sqrt(np.mean(np.sum((projected-np.array(pts['pixels']))**2,axis=1))))
   rms.append(error);per.append({'event_id':c['event_id'],'pixel_rmse':error})
  s['cameras'][str(cam)]['calibration']['loo_pixel_rmse']=dist(rms)
  s['cameras'][str(cam)]['calibration']['loo_pixels_per_event']=per
  print(name,'cam',cam,'heldout px',dist(rms),flush=True)
(OUT/'assessment.json').write_text(json.dumps(j,indent=2,allow_nan=False))
