from pathlib import Path
import json
import sys
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
j=json.loads((OUT/'assessment.json').read_text()); im=json.loads((OUT/'image_audit.json').read_text())
names=['cam0-1_02','cam1-3_02','cam0-3_01']; labels=['cam0–cam1','cam1–cam3','cam0–cam3'];grades=['조건부 사용 · 보완 권장','가장 양호 · 1차 계산에 사용 가능','부족 · 우선 추가 촬영']
selection={'criteria':j['criteria'],'note':'Original event_id values (zero-based). Passing detection gates is not proof of absolute calibration accuracy. Original files are unchanged.','sessions':{}}
for name in names:
 s=j['sessions'][name];m=json.loads(Path(s['source']).read_text())
 stronger=[c['event_id'] for c in m['captures'] if c['event_id'] in s['eligible_ids'] and all(c['cams'][str(k)]['charuco']['n_corners']>=20 and c['cams'][str(k)]['charuco']['reproj_error_px']<=.8 for k in m['cam_indices'])]
 s['stronger_candidate_ids_20corners_0p8px']=stronger
 selection['sessions'][name]={'meta_path':s['source'],'eligible_event_ids':s['eligible_ids'],'excluded_event_ids':s['rejected_ids'],'stricter_candidate_event_ids':stronger}
(OUT/'event_selection.json').write_text(json.dumps(selection,indent=2))
(OUT/'assessment.json').write_text(json.dumps(j,indent=2,allow_nan=False))

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
counts=[j['sessions'][n]['eligible_count'] for n in names];total=[j['sessions'][n]['n'] for n in names]
axes[0,0].bar(labels,counts,color=['#bb9144','#3c9473','#c3655b'],label='Pass both cameras')
axes[0,0].bar(labels,np.array(total)-counts,bottom=counts,color='#e2e5e9',label='Rejected')
for k,(a,b) in enumerate(zip(counts,total)):axes[0,0].text(k,b+1,f'{a}/{b} ({a/b:.0%})',ha='center')
axes[0,0].set_ylim(0,56);axes[0,0].set_ylabel('Capture events');axes[0,0].set_title('63 / 117 events pass both-camera quality gates');axes[0,0].legend(loc='upper left',fontsize=8)
positions=np.arange(3)
for idx,name in enumerate(names):
 s=j['sessions'][name]
 for n,(cam,c) in enumerate(s['cameras'].items()):
  d=c['calibration']['loo_pixel_rmse'];x=idx+(-.15 if n==0 else .15)
  axes[0,1].errorbar(x,d['median'],yerr=[[d['median']-d['min']],[d['p90']-d['median']]],fmt='o',capsize=5,color=('#4375aa' if n==0 else '#b77b39'))
  axes[0,1].annotate(f'cam{cam}',(x,d['median']),xytext=(0,8),textcoords='offset points',ha='center',fontsize=8)
axes[0,1].set_xticks(positions,labels);axes[0,1].set_ylabel('px');axes[0,1].set_title('Leave-one-out pixel error: median (min–P90)');axes[0,1].grid(axis='y',alpha=.2)
for name,label in zip(names,labels):
 s=j['sessions'][name];vals=[]
 for c in s['cameras'].values():vals.extend([v['pixel_rmse'] for v in c['calibration']['loo_pixels_per_event']])
 axes[1,0].plot(np.sort(vals),np.arange(1,len(vals)+1)/len(vals),label=label)
axes[1,0].set_xlabel('Held-out image reprojection RMSE (px)');axes[1,0].set_ylabel('Cumulative fraction');axes[1,0].set_title('Calibration consistency on unseen poses');axes[1,0].legend();axes[1,0].grid(alpha=.2)
angles=[j['sessions'][n]['diversity']['pair_rotation_deg']['mean'] for n in names]
axes[1,1].bar(labels,angles,color=['#bb9144','#3c9473','#c3655b']);axes[1,1].axhline(40,color='#666',ls='--',label='Existing recorder target: 40° (heuristic)')
for k,a in enumerate(angles):axes[1,1].text(k,a+.8,f'{a:.1f}°',ha='center')
axes[1,1].set_ylim(0,48);axes[1,1].set_ylabel('Mean pairwise rotation (degrees)');axes[1,1].set_title('Orientation diversity: usable events only');axes[1,1].legend(fontsize=8,loc='upper left')
fig.suptitle('260910 capture assessment — internal consistency, not absolute accuracy',fontsize=14)
fig.savefig(OUT/'quality_summary.png',dpi=170);plt.close(fig)

lines=[]
def add(text=''):lines.append(text)
add('**260910 촬영 결과 평가 — 원본 이미지·meta.json 기준**')
add()
add('판정: **cam1–cam3는 1차 캘리브레이션에 사용할 수 있는 데이터가 충분하다. cam0–cam1은 조건부로 사용하고 보완 촬영을 권장한다. cam0–cam3는 현재 10개 유효 자세만으로 정밀 결과를 확정하기 어렵고 추가 촬영이 우선이다.** 세 카메라 전체를 정밀하게 캘리브레이션했다고 판단할 단계는 아니다.')
add()
add('총 117회 촬영, RGB 234장과 depth 234장을 모두 읽어 검사했다. 파일 누락·이미지 디코딩 실패는 없었다. 촬영된 메타데이터는 로봇 보드 9×6, 체커 25mm, 마커 18mm, ID 90~116, legacy 패턴으로 일치한다. JPEG를 다시 검출한 오차와 저장된 오차의 차이는 카메라별 중앙값 약 0.003~0.005px로, 저장된 검출 결과를 재현할 수 있었다.')
add()
add('유효 기준은 촬영 당시 기준인 **두 카메라 각각 코너 ≥12개, PnP 재투영 RMSE ≤1.5px, 이미지 저장 성공**에 유한한 SE(3) 자세 검사까지 추가했다. **63/117회(53.8%)**가 통과한다. 이 기준은 검출 품질 기준이며 절대 위치 정확도를 보증하지 않는다.')
add()
add('| 세션 | 총 촬영 | 두 카메라 유효 | 유효율 | 유효 자세 평균 상대회전 | 평가 |')
add('| --- | ---: | ---: | ---: | ---: | --- |')
for name,grade in zip(names,grades):
 s=j['sessions'][name];add(f"| {name} | {s['n']} | {s['eligible_count']} | {s['eligible_count']/s['n']:.1%} | {s['diversity']['pair_rotation_deg']['mean']:.1f}° | {grade} |")
add()
add('더 엄격하게 두 카메라 모두 코너 ≥20개 및 재투영 오차 ≤0.8px로 제한하면 cam0–cam1은 **9개**, cam1–cam3는 **30개**, cam0–cam3는 **4개**가 남는다. 이는 비교용 후보 기준이며, 자세 다양성이 줄어들 수 있으므로 이 목록만 무조건 사용하는 것은 권하지 않는다. 한 카메라만 통과한 관측도 보존할 가치가 있다. 카메라별 개별 통과 관측은 cam0=55개, cam1=73개, cam3=46개로 총 174/234개이다. 두 카메라가 모두 필요한 쌍별 계산에는 위의 63개를 사용했다.')
add()
add('**캘리브레이션 검증 방법과 결과**')
add()
add('Shah solver의 합성 무잡음 convention self-test를 통과시킨 뒤, 유효 프레임만으로 카메라마다 T_base_camera와 T_gripper_board를 계산했다. 각 자세를 하나씩 제외해 나머지로 다시 계산하고, 제외한 로봇 자세의 보드 위치·회전 및 이미지 코너를 예측했다. 아래 mm 값은 보드 자세의 일관성 잔차이며, ground truth와 비교한 카메라 절대 위치 오차가 아니다. 픽셀 오차는 재검출된 실제 JPEG 코너와 비교한 값이다.')
add()
add('| 세션 | 카메라 | 제외 자세 보드 위치 잔차 중앙값 / P90 | 영상 예측 RMSE 중앙값 / P90 | 80% 부분집합 재계산 시 카메라 위치 변화 P90 |')
add('| --- | --- | ---: | ---: | ---: |')
for name in names:
 for cam,c in j['sessions'][name]['cameras'].items():
  cal=c['calibration'];a=cal['loo']['translation_mm'];b=cal['loo_pixel_rmse'];v=cal['subsample_80pct_100runs_change_from_full']['translation_mm']
  add(f"| {name} | cam{cam} | {a['median']:.2f} / {a['p90']:.2f} mm | {b['median']:.2f} / {b['p90']:.2f} px | {v['p90']:.2f} mm |")
add()
add('부분집합 검사는 전체 유효 자세의 약 80%를 중복 없이 추출해 100회 재계산한 결과이며, 전체 데이터로 구한 해와의 변화량이다. **cam0–cam3의 cam3는 P90 19.22mm, 최대 28.51mm**까지 달라졌다. 반면 cam1–cam3는 두 카메라 모두 P90 약 1.1~1.2mm이다. 이 차이가 현재 데이터의 안정성 차이를 보여준다. 부분집합 변동이 작아도 모든 프레임에 공통으로 들어간 보정 편향까지 검출하는 것은 아니다.')
add()
add('**이미지에서 확인한 문제**')
add()
add('- cam0–cam1: 주된 탈락 원인은 cam0의 부족한 코너와 실패한 자세 추정이다. 예를 들어 event 2, 4, 15는 보드가 잘리거나 매우 좁게 보인다. cam0의 전체 코너 중앙값은 16개이다. cam1은 단일 프레임 PnP 오차가 작아도 전체 로봇-보드 관계에 대한 예측 오차가 더 크므로, 작은 PnP 오차만으로 좋은 캘리브레이션이라고 판단하면 안 된다.')
add('- cam0–cam3: cam3의 전체 유한 PnP 오차 중앙값은 **2.87px**, 최대 **5.41px**다. event 20/32/34/39 등에서 보드가 화면의 왼쪽·위쪽 경계를 넘거나 일부만 보인다. 화면 외곽 및 부분 관측과 큰 오차가 함께 나타난다. 인쇄판 평탄도나 렌즈 보정 영향도 가능하나, 이번 데이터만으로 특정 원인을 단정할 수는 없다.')
add('- cam1–cam3: cam1이 보드를 충분히 보는 프레임이 많고, 두 카메라 모두 통과한 34개의 일관성이 가장 좋다. 다만 event 35~43 부근에는 cam3의 큰 오차나 보드 잘림이 많다. 이 구간을 포함한 탈락 프레임은 기본 선택에서 제외했다.')
add()
add(f'![요약 그래프]({OUT / "quality_summary.png"})')
add()
add('**세 세션을 함께 사용할 때**')
add()
loop=j['cross_session_loop_closure']
add(f"로봇 자세를 사용하지 않고, 동시에 본 보드 자세로 카메라 상대변환 T_cam0_cam1, T_cam1_cam3, T_cam0_cam3를 각각 추정했다. 직접 경로 0→3과 0→1→3 경로의 차이는 **{loop['translation_mm']:.2f}mm / {loop['rotation_deg']:.2f}°**다. 세 쌍의 결과가 완전히 일치하지 않으므로, 세 카메라를 통합한 정밀 결과를 확정하기 전에 cam0 관련 쌍을 보완하는 편이 좋다. 이것은 경로 일관성 검사이며 실제 절대오차를 뜻하지 않는다.")
add()
add('테이블 보드의 고정 코너를 비교했을 때 세션 사이 영상 위치 차이 중앙값은 cam0=0.29px, cam1=0.16px, cam3=0.53px였다. 공통으로 관측된 코너는 각각 50/9/43개였다. 큰 카메라/테이블 이동을 시사하는 변화는 보이지 않았지만, 이것만으로 완전한 고정을 증명하지는 못한다.')
add()
add('유효 자세의 평균 상대회전은 21.7~27.5°로 기존 레코더 목표 40°보다 낮다. 단, 40°는 촬영 지침용 목표이지 절대적인 합격선은 아니다. 회전 차이 행렬은 세 쌍 모두 full rank였고, 회전축 하나만 움직인 완전한 퇴화는 아니었다. 최대 상대회전은 약 54~67°이며, 2mm·2° 이내 중복 제거 기준에서도 유효 자세는 각각 19/34/10개로 유지된다. cam0–cam3는 표본 수와 안정성 측면의 부족이 더 크다.')
add()
add('**기존 자동 평가값을 그대로 쓰면 안 되는 이유**')
add()
add('현재 `solve_and_report()`는 `capture_gate.pass`를 검사하지 않고 보드 변환 행렬이 존재하면 포함한다. 그래서 cam0–cam3의 저장된 `shah_field_check`는 36개, cam1–cam3는 43개를 사용했지만, 실제 촬영 품질 기준을 통과한 것은 각각 10개와 34개이다. cam0–cam1에는 event **2, 8, 14, 17, 18**의 cam0 PnP가 NaN을 포함한다. 이 5건은 이미 코너 부족으로 gate에서 탈락했으므로 유효 19개에는 포함되지 않는다. 다만 자동 평가 함수에는 들어갈 수 있으며, 해당 세션의 `shah_field_check`는 null이다. 이번 재평가는 이런 항목을 제외했다. 기존 프로그램의 시뮬레이션 계수 5.4를 곱한 수치도 실제 절대 정확도로 해석하면 안 된다.')
add()
add('카메라별 host 수신 시각 차이 중앙값은 약 72~78ms, 최대 약 102ms다. 하드웨어 동기화된 노출 시각은 아니므로 이 값은 노출 시간차의 정확한 측정이 아니다. 로봇 자세를 먼저 읽고 0.5초 기다린 뒤 이미지를 순차 취득하는 현재 방식에서는 촬영 중 정지 여부가 중요하다. 메타데이터에 충분한 전후 상태 표본은 없어 완전 정지를 증명하지 못한다. 추가 촬영은 GELLO 움직임을 멈춘 상태에서 진행하는 것이 좋다.')
add()
add('**권장 다음 작업**')
add()
add('1. cam1–cam3의 유효 34개를 먼저 사용해 기준 결과를 만든다. 이번 데이터 중 우선 보존·활용할 세션이다.')
add('2. cam0–cam3는 두 카메라에서 함께 통과하는 자세를 **추가 15~20개** 정도 확보하는 것을 작업 목표로 잡는다. 보드 전체와 여백이 프레임 안에 들어오게 하고, 프레임 가장자리로 밀리는 자세는 피한다. 정확도 요구에 따라 필요한 양은 달라지므로 촬영 후 같은 안정성 검사를 반복한다.')
add('3. cam0–cam1도 **추가 10~15개** 정도의 양호한 공동 관측을 보완한다. 위치 이동만 반복하지 말고 서로 다른 축의 기울기를 넣되, 코너 수가 유지되는 범위에서 촬영한다.')
add('4. 추가 촬영에서도 카메라나 보드의 장착이 바뀌면 이전 데이터와 무조건 합치지 않는다. 품질 탈락 프레임은 보존하되 직접 solver에 넣는 목록에서 제외한다.')
add()
add('**파일과 재현**')
add()
add('- `event_selection.json`: 세션별 유효/제외/엄격 후보 event_id. 원본의 0부터 시작하는 ID이며 파일 번호와 대응한다.')
add('- `assessment.json`: 전체 수치, 유효 데이터로 재계산한 잠정 변환, leave-one-out, 100회 부분집합 검사. 배포용 최종 캘리브레이션으로 확정한 파일이 아니다.')
add('- `image_audit.json`, `image_points.json`: 전체 RGB/depth 검사와 재검출된 영상 코너.')
add('- `*_contact.jpg`: 양호/불량 예시 이미지 모음.')
add('- 원본 `datasets/260910`의 이미지와 meta.json은 수정하지 않았다. 분석은 실제 meta.json의 부모 디렉터리를 기준으로 경로를 해석했다. metadata에는 예전 `.../260910/shah/...` 경로가 남아 있으나 현재 실제 폴더는 `.../260910/세션명`이므로, 원래 절대경로만 따르는 외부 도구에서는 경로 보정이 필요하다.')
add('- 평가 중 현재 로컬 `capture/board_config.py`는 7×5 / 17mm / ID 0 설정이며 `legacy_pattern` 필드도 없어 촬영 메타의 9×6 / 25mm / ID 90 설정과 달랐다. 이 작업은 그 파일을 변경하지 않고 촬영된 meta.json의 보드 정의를 사용했다. 이후 재검출/재촬영 때 현재 코드 설정을 그대로 쓰면 안 된다.')
add()
add('재현 순서: `python reports/260910_quality/assess_dataset.py` → `python reports/260910_quality/audit_images.py` → `python reports/260910_quality/check_holdout_pixels.py` → `python reports/260910_quality/make_report.py`. 모든 경로는 저장소 루트 기준이다. 원본 이미지·메타데이터와 현재 intrinsics를 기준으로 내부 일관성을 평가했으며, 별도 ground truth로 절대 정확도를 확인한 것은 아니다. 입력 meta.json의 SHA-256 해시는 assessment.json에 기록했다.')
(OUT/'evaluation_ko.md').write_text('\n'.join(lines)+'\n')
print(OUT/'evaluation_ko.md')
print(OUT/'quality_summary.png')
print(OUT/'event_selection.json')
