import csv
import hashlib
import json
import shutil
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path('D:/Project/dog/demov3')
WORK = ROOT / 'semantic_map_work'
OUT = ROOT / 'semantic_map_v1_20261028'
SRC = ROOT / '20261027_201820'
def save(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

p = np.loadtxt(WORK / 'recovered/mat_pre.txt')
t = np.load(WORK / 'recovered/cloud_timestamps.npy')
assert p.shape == (3012, 25) and t.shape == (3012, 2)
offset = t[:, 1] - p[:, 0]
residual = (offset - np.median(offset)) * 1000
assert np.max(np.abs(residual)) < 3
(OUT / 'recovery_evidence').mkdir(exist_ok=True)
for name in ['mat_pre.txt', 'laserMapping.cpp', 'lidar_config.json', 'cloud_timestamps.npy']:
    shutil.copy2(WORK / 'recovered' / name, OUT / 'recovery_evidence' / name)
with (OUT / 'trajectory_prediction.csv').open('w', newline='', encoding='utf-8') as f:
    w = csv.writer(f)
    w.writerow(['cloud_index', 'cloud_end_unix_s', 'bag_receipt_unix_s', 'lidar_begin_relative_s', 'predicted_imu_x_m', 'predicted_imu_y_m', 'predicted_imu_z_m'])
    for i, (row, times) in enumerate(zip(p, t)):
        w.writerow([i, format(times[1], '.9f'), format(times[0], '.9f'), *row[[0,4,5,6]]])
recovery = {
    'schema': 'dog.recovered_trajectory_audit.v1',
    'status': 'approximate_prediction_positions_only_not_final_localization',
    'map_frame': 'dog_lio_map', 'position_reference': 'lidar IMU origin',
    'remote_source': '/home/robot/dog_navigation/ws/src/FAST_LIO_ROS2/Log/mat_pre.txt',
    'source_sha256': sha(WORK / 'recovered/mat_pre.txt'),
    'cloud_count': len(t), 'log_rows': len(p),
    'alignment': 'Same index, matching count and scan interval pattern; cloud stamp is scan end, log relative time is scan beginning.',
    'median_cloud_end_minus_log_relative_s': float(np.median(offset)),
    'alignment_residual_ms_min_median_max': np.percentile(residual,[0,50,100]).tolist(),
    'max_abs_alignment_residual_ms': float(np.max(np.abs(residual))),
    'limitations': [
        'Position is the IMU-propagated prediction before the current LiDAR update, not the updated localization output.',
        'Logged Euler angles belong to the previous updated frame; no pose matrix is exported as a valid final transform.',
        'No complete camera-to-LiDAR extrinsic calibration was found; positions are not used to place semantic targets.',
        'Final saved map snapshot precedes the end of camera recording; not all observations necessarily correspond to the map snapshot.'
    ],
    'log_columns': ['relative_lidar_begin_time', 'previous_updated_euler_x_y_z_scaled_by_57.3', 'current_predicted_position_x_y_z', 'lidar_extrinsic_euler_x_y_z', 'lidar_extrinsic_translation_x_y_z', 'velocity_x_y_z', 'gyro_bias_x_y_z', 'accel_bias_x_y_z', 'gravity_x_y_z'],
    'position_bbox_min_m': p[:,4:7].min(axis=0).tolist(),
    'position_bbox_max_m': p[:,4:7].max(axis=0).tolist()
}
save(OUT / 'trajectory_recovery.json', recovery)

sem = json.loads((OUT / 'semantic_observations.json').read_text(encoding='utf-8'))
sem['registration_status'] = 'blocked_missing_camera_lidar_extrinsics_and_final_updated_trajectory'
sem['recovered_trajectory'] = 'trajectory_recovery.json'
notes = {
    'obs_0029': '代表帧复核：框覆盖箱体及火苗海报，旁边的试剂瓶海报未处于该框中心；疑似错框，深度可能来自背景。',
    'obs_0025': '代表帧复核：最高分 chemical 框疑似落在火苗海报上；邻近另有试剂瓶海报，需要逐帧复核。',
    'obs_0023': '代表帧复核：chemical 框疑似覆盖火苗海报，可能类别误判。',
    'obs_0044': '代表帧复核：框包含较大地面和箱体区域，无法确认对应试剂瓶目标。',
    'obs_0001': '代表帧复核：框主要覆盖白色衣物或人员区域，疑似 chemical 误检。',
    'obs_0020': '代表帧复核：区域中可见试剂瓶海报，但框包含大量背景；中心深度不一定属于海报。',
    'obs_0015': '代表帧复核：区域中可见试剂瓶海报，但框偏大；需核查目标边界和中心深度。',
    'obs_0030': '代表帧复核：宽框覆盖火苗及背景区域，chemical 预测需要复核。',
    'obs_0040': '代表帧复核：画面边缘可见火苗海报；边缘裁切会影响框和深度。',
    'obs_0014': '代表帧复核：可见火苗海报。这只确认代表帧中的类别特征，不代表整段已审核。',
    'obs_0065': '代表帧复核：可见人员海报。这只确认代表帧中的类别特征，不代表整段已审核。',
    'obs_0051': '代表帧复核：可见人员海报。这只确认代表帧中的类别特征，不代表整段已审核。'
}
for event in sem['events']:
    if event['id'] in notes:
        event['visual_review_note'] = notes[event['id']]
        event['review_status'] = 'representative_frame_reviewed_track_not_verified'
save(OUT / 'semantic_observations.json', sem)

summary = json.loads((OUT / 'summary.json').read_text(encoding='utf-8'))
summary['missing'] = ['calibrated T_lidar_camera external transform', 'final updated timestamped trajectory / TF (prediction positions recovered only)']
summary['recovered_prediction_positions'] = 3012
summary['geometry_file_byte_reduction_percent'] = round(100*(1-summary['compact_voxel_npz_bytes']/summary['original_pcd_bytes']),2)
summary['registration_status'] = sem['registration_status']
summary['recognition_quality'] = 'Representative keyframes contain suspected chemical false positives and oversized boxes; model confidence is not verified truth.'
summary['source_map_time_limitation'] = 'Map snapshot saved before the final approximately 45 seconds of the recorded observations.'
save(OUT / 'summary.json', summary)

npz = np.load(OUT / 'voxel_map_5cm.npz')
xyz = npz['centroids_xyz']
focus = json.loads((OUT / 'viewer_data.json').read_text(encoding='utf-8'))
fig, ax = plt.subplots(figsize=(11,9), dpi=160)
mask = xyz[:,2] <= 1.8
sc = ax.scatter(xyz[mask,0],xyz[mask,1],c=xyz[mask,2],s=1,cmap='viridis',vmin=-.4,vmax=1.8,rasterized=True)
ax.plot(p[:,4],p[:,5],color='#e95c40',linewidth=1.4,label='Recovered IMU prediction path (approximate)')
ax.scatter(p[0,4],p[0,5],color='#e95c40',marker='o',s=40)
ax.set_xlim(focus['focus_min'][0]-.5, focus['focus_max'][0]+.5)
ax.set_ylim(focus['focus_min'][1]-.5, focus['focus_max'][1]+.5)
ax.set_aspect('equal')
ax.set(xlabel='X in dog_lio_map (m)',ylabel='Y in dog_lio_map (m)',title='demov3 | Indoor view | 5 cm voxels | Z <= 1.8 m')
ax.legend(loc='upper right',fontsize=9)
fig.colorbar(sc,ax=ax,label='Map Z (m)')
ax.text(.01,.01,'Semantics NOT spatially registered. Blank = unknown, not free.\nPath positions precede LiDAR update; no accuracy guarantee.',transform=ax.transAxes,fontsize=8,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
fig.tight_layout();fig.savefig(OUT / 'map_indoor_preview.png');plt.close(fig)

template = (WORK / 'viewer_template.html').read_text(encoding='utf-8')
template = template.replace('目前不能可靠地把相机目标放到雷达地图。', '已恢复 3012 条近似预测位置；仍缺相机—雷达外参和最终更新轨迹，目前不能可靠地把相机目标放到雷达地图。')
template = template.replace('<a href="README.txt">处理说明</a>', '<a href="trajectory_prediction.csv">恢复的近似轨迹</a> · <a href="map_indoor_preview.png">室内地图与轨迹预览</a> · <a href="README.txt">处理说明</a>')
payload = {'map':focus,'events':[{k:v for k,v in e.items() if k != 'observations'} for e in sem['events']]}
html = template.replace('__PAYLOAD__',json.dumps(payload,ensure_ascii=False,separators=(',',':')).replace('</','<\\/'))
(OUT / 'index.html').write_text(html,encoding='utf-8')

readme = '''demov3 地图与语义观察压缩包（2026-10-28）

当前状态：几何地图压缩完成；语义观察整理完成；语义目标尚未空间配准。
这不是已经完成目标定位的语义地图，不能据此导航到火苗、危险品或人员。

一、打开与文件
打开 index.html 可旋转点云、切换俯视并筛选三个类别的观察片段。
map_indoor_preview.png：室内范围及恢复的近似轨迹；map_top_view.png：全范围回波密度。
map_5cm.pcd：5 cm 体素中心 PCD，便于常规点云工具加载。
voxel_map_5cm.npz：压缩数组，含体素索引、中心、强度、原始点计数；semantic_class_id 全为 -1（未赋值）。
semantic_observations.json：全部 1486 次模型观察，按框连续性合并成 69 段（chemical 54、fire 5、human 10）。69 段不等于 69 个独立物体。
keyframes/：21 张直接从 v3 ROS bag 提取的真实识别画面；按 RGB 头时间戳精确匹配。
trajectory_prediction.csv：恢复的 3012 个预测位置；trajectory_recovery.json：恢复依据和限制。
recovery_evidence/：原 FAST-LIO 日志、代码及时间戳，便于追溯。
summary.json、validation.json、SHA256SUMS.json：统计、检查及产物完整性记录。

二、压缩含义
原地图 1,197,562 点、38,322,237 字节；5 cm 地图 210,302 点，点数减少 82.44%。
紧凑 NPZ 3,253,204 字节，较原 PCD 减少 91.51%；这是几何文件对比，不是整份记录包的压缩率。
每格保留全部回波的平均坐标和强度，并记录回波数量。未裁剪原地图范围、未去除离群点。
原 PCD 的 normal_x/normal_y/normal_z/curvature 全为零，精简导出中未保留这些空字段。
网页为流畅显示使用 10 cm 点云及厘米级量化；导出地图仍为 5 cm 体素中心。
俯视图空白表示未观测，不等于可通行；不是已校验的二维占据导航地图。

三、为什么暂时不把类别贴到地图
v3 bag 只有配准点云、识别图像、检测框和对象 JSON，没有保存 odometry、TF、原始 IMU 或完整深度图。
NUC 找到的 mat_pre.txt 与 3012 帧点云数量及间隔吻合，时间对齐残差最大约 2.62 ms。
经原算法代码确认，其中位置是当前帧激光更新前的 IMU 预测，Euler 角是上一帧更新后的角度，不是完整最终轨迹。
目前找到的 mounting_approx 文件是雷达—机身方向及近似位移；mount_fixture.json 明确是软件测试用，不是真实安装标定。
尚未找到相机—雷达外参。相机光学坐标 X 向右、Y 向下、Z 向前，不能直接当作地图坐标。
因此保留观察的相机坐标，地图目标坐标设为 null，不猜测目标位置。
最终地图保存时间还早于录制结束约 45 秒，因此部分后段观察可能不在这个地图快照中。

四、识别质量与后续配准
15 个优先复核候选依据：至少 3 次置信度 >= 0.60、最高 >= 0.70、跨度 >= 0.19 秒；只是复核优先级。
代表帧检查发现 chemical 有火苗海报、白色衣物和背景上的疑似误检，部分框过大，框中心深度可能来自背景。
未凭单张代表图删除整段模型输出；复核说明写入各片段，后续融合前需要逐帧确认。
完成可靠空间语义地图需要找回相机—雷达外参以及最终定位轨迹；仅找到外参也不足以保证本次近似轨迹的定位精度。
下一次录制除地图和语义外应保存定位 odometry、/tf、/tf_static、相机 camera_info、同步深度及标定版本；这些是配准信息，并非电机控制信息。
需要重新解算轨迹时才增加原始雷达和 IMU。重复观测须经位置、类别、深度与时间一致性融合，不能只按同类合并。

五、原始资料
来源：D:/Project/dog/demov3/20261027_201820；原始数据没有修改。
本次仅离线处理与读取 NUC 日志，没有启动电机、相机或雷达。
代码副本位于 processing/，本机工作脚本位于 ../semantic_map_work/。
'''
(OUT / 'README.txt').write_text(readme,encoding='utf-8')
(OUT / 'processing').mkdir(exist_ok=True)
for name in ['build_map.py','extract_keyframes.py','audit_trajectory.py','finalize_bundle.py','viewer_template.html']:
    shutil.copy2(WORK/name,OUT/'processing'/name)

assert len(xyz) == 210302 and np.isfinite(xyz).all()
assert int(npz['sample_count'].sum()) == 1197562
assert (npz['semantic_class_id'] == -1).all()
assert sum(e['frames'] for e in sem['events']) == 1486
assert all(e['map_xyz_m'] is None for e in sem['events'])
assert all((OUT/e['representative']['image']).is_file() for e in sem['events'] if e['representative']['image'])
for filename,digest in summary['source_hashes'].items():
    assert sha(SRC/filename) == digest
save(OUT/'validation.json',{'source_hashes_unchanged':True,'all_original_returns_accounted_for':True,'finite_voxel_centroids':True,'all_1486_observations_retained':True,'fabricated_semantic_positions':0,'recovered_log_cloud_count_match':True,'time_alignment_max_residual_ms':float(np.max(np.abs(residual))),'browser_review':'pending'})
save(OUT/'SHA256SUMS.json',{str(f.relative_to(OUT)).replace('\\','/'):sha(f) for f in sorted(OUT.rglob('*')) if f.is_file() and f.name!='SHA256SUMS.json'})
archive=ROOT/'semantic_map_v1_20261028.zip'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for f in sorted(OUT.rglob('*')):
        if f.is_file():z.write(f,str(f.relative_to(OUT.parent)))
print(json.dumps({'output':str(OUT),'zip_bytes':archive.stat().st_size,'recovery':recovery['max_abs_alignment_residual_ms']},ensure_ascii=False))
