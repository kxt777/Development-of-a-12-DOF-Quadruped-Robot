import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
DEMO=ROOT.parent
def save(name,data):
    (ROOT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
def transform(array):
    t=np.eye(4);t[:3,:3]=np.array(array[:9]).reshape(3,3).T;t[:3,3]=array[9:12]
    assert abs(array[12]-1)<1e-12
    return t
def rigid(rotation=None,position=None):
    t=np.eye(4)
    if rotation is not None:t[:3,:3]=rotation
    if position is not None:t[:3,3]=position
    return t
def extrinsic(e):
    assert e['lengthUnit']==1 # EEPROM/API centimetres
    return rigid(e['rotationMatrix'],[e['translation'][k]*.01 for k in 'xyz'])
def pack(t):
    r=Rotation.from_matrix(t[:3,:3])
    return {'matrix_4x4_m':t.tolist(),'translation_m':t[:3,3].tolist(),'quaternion_xyzw':r.as_quat().tolist(),'rpy_xyz_radians':r.as_euler('xyz').tolist(),'det_R':float(np.linalg.det(t[:3,:3])),'orthogonality_max_error':float(np.max(abs(t[:3,:3].T@t[:3,:3]-np.eye(3))))}

geometry=json.loads((ROOT/'solidworks_sensor_geometry.json').read_text(encoding='utf8'))
parts={p['name']:p for p in geometry['parts']}
top=json.loads((ROOT/'solidworks_sensor_transforms.json').read_text(encoding='utf8'))
oak=transform(parts['oak-d-pro-1']['transform'])
lidar=transform(parts['mid-360-asm-2']['transform'])
glass_part=next(p for p in parts.values() if 'D-PRO-GLASS-LENS' in p['name'])
glass=transform(glass_part['transform'])
assert any(f['type']=='plane' and np.allclose(f['parameters'],[0,0,1,0,0,.0006],atol=1e-12) for f in glass_part['faces'])
base_part=next(p for p in parts.values() if '/MID-360_4_2.stp-' in p['name'])
assert any(f['type']=='plane' and np.allclose(f['parameters'],[0,1,0,0,-.0259171,0],atol=1e-12) for f in base_part['faces'])

# Livox drawing: optical origin 47 mm above bottom mounting plane.
# Connector points in +CAD X; manual LiDAR +X is away from connector.
cad_lidar_from_lidar=rigid([[-1,0,0],[0,0,1],[0,1,0]],[0,-.0259171+.047,0])
assembly_from_lidar=lidar@cad_lidar_from_lidar
# The centre of the central round window is (0,0) in the glass part.
# RDF axes: right=-CAD X, down=-CAD Y, forward=CAD Z.
assembly_from_front=glass@rigid(np.diag([-1,-1,1]),[0,0,.0006])
lidar_from_front=np.linalg.inv(assembly_from_lidar)@assembly_from_front
cad_oak_from_front=np.linalg.inv(oak)@assembly_from_front

factory=json.loads((ROOT/'oak_factory_calibration.json').read_text(encoding='utf8'))
camera=dict(factory['cameraData'])
assert factory['housingExtrinsics']['toCameraSocket']==2
assert camera[2]['extrinsics']['toCameraSocket']==0
# Factory chain is H_right -> physical CAM_C -> physical CAM_A.
# H_right origin is nominally anchored to the CAD/official ROS camera reference.
A_from_C=extrinsic(camera[2]['extrinsics'])
C_from_H=extrinsic(factory['housingExtrinsics'])
H_from_A=np.linalg.inv(A_from_C@C_from_H)

# Derive the official ROS mesh's nominal depth datum from four independent case planes.
b=(ROOT/'official/OAK-D-PRO_ROS.stl').read_bytes()
mesh=np.frombuffer(b,offset=84,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
mask=abs(mesh['n'][:,2])>.99999
planes=np.unique(np.round(mesh['v'][mask,:,2].mean(1),8))
matches=[]
for cad_z in [-.002,-.005,-.0013,.0077]:
    z=planes[np.argmin(abs(planes-(cad_z-.0097309)))]
    matches.append({'cad_plane_z_m':cad_z,'official_ros_mesh_plane_z_m':float(z),'offset_m':float(cad_z-z)})
z_datum=float(np.median([x['offset_m'] for x in matches]))
assert max(abs(x['offset_m']-z_datum) for x in matches)<1e-8
nominal_right_origin=[float(cad_oak_from_front[0,3]-.0375),float(cad_oak_from_front[1,3]),z_datum]
cad_oak_from_H=rigid(np.diag([-1,-1,1]),nominal_right_origin)
raw_lidar_from_A=np.linalg.inv(assembly_from_lidar)@oak@cad_oak_from_H@H_from_A
# Round-off in EEPROM matrices is removed by polar projection onto SO(3).
u,_,vt=np.linalg.svd(raw_lidar_from_A[:3,:3]);lidar_from_A=raw_lidar_from_A.copy();lidar_from_A[:3,:3]=u@vt
assert np.linalg.det(lidar_from_A[:3,:3])>.999999
front_forward=lidar_from_front[:3,2]
optical_forward=lidar_from_A[:3,2]

result={
 'schema':'dog.cad_camera_lidar_initial_extrinsics.v1',
 'status':'complete_6dof_initial_estimate_not_jointly_calibrated',
 'calibrated':False,'valid_for_navigation':False,
 'source_frame':'dog_oak_rgb_camera_optical_frame',
 'target_frame':'MID360_native_pointcloud_coordinates',
 'convention':'Column vectors: p_lidar = R_lidar_camera @ p_camera + t_lidar_camera; all translations in metres.',
 'camera_axes':'X right, Y down, Z forward',
 'lidar_axes':'X away from connector / robot forward, Y left, Z up',
 'user_confirmed_cad_matches_demov3_installation':True,
 'assembly_path':top['document'],
 'sensor_instances':['oak-d-pro-1','mid-360-asm-2'],
 'mechanically_measured_lidar_from_rgb_front_glass':pack(lidar_from_front),
 'initial_lidar_from_rgb_optical':pack(lidar_from_A),
 'initial_rgb_optical_from_lidar':pack(np.linalg.inv(lidar_from_A)),
 'front_glass_pitch_down_deg':float(np.degrees(np.arctan2(-front_forward[2],np.linalg.norm(front_forward[:2])))),
 'front_glass_yaw_left_deg':float(np.degrees(np.arctan2(front_forward[1],front_forward[0]))),
 'factory_adjusted_optical_pitch_down_deg':float(np.degrees(np.arctan2(-optical_forward[2],np.linalg.norm(optical_forward[:2])))),
 'transform_chain':{
   'assembly_from_lidar_cad':pack(lidar),
   'assembly_from_oak_cad':pack(oak),
   'lidar_cad_from_lidar_native':pack(cad_lidar_from_lidar),
   'oak_cad_from_front_glass_rdf':pack(cad_oak_from_front),
   'oak_cad_from_nominal_right_housing_rdf':pack(cad_oak_from_H),
   'physical_CAM_A_from_CAM_C':pack(A_from_C),
   'physical_CAM_C_from_housing_origin':pack(C_from_H),
   'housing_origin_from_physical_CAM_A':pack(H_from_A)
 },
 'nominal_origin_evidence':{
   'description':'CAD back-case planar faces agree with official depthai-ros mesh after depth translation; ROS RGB camera frame is colocated with its base frame. This is a nominal mechanical datum, not a measured optical principal plane.',
   'mesh_datum_matches':matches,
   'nominal_right_camera_in_oak_cad_m':nominal_right_origin,
   'glass_front_z_in_oak_cad_m':float(cad_oak_from_front[2,3]),
   'glass_front_minus_nominal_optical_datum_m':float(cad_oak_from_front[2,3]-z_datum)
 },
 'factory_housing_api_audit':{
   'productName':factory['productName'],'boardName':factory['boardName'],'boardRev':factory['boardRev'],
   'depthai_version':'3.10.0',
   'observation':'CAM_A, FRONT_CAM_A and VESA_A returned identical transforms; no model-specific mechanical coordinates exist in the corresponding housing table. Named housing translations were NOT used as verified physical origins.',
   'used_data':'EEPROM housing orientation and calibrated CAM_C to CAM_A transform; physical housing origin supplied by CAD plus nominal official ROS mesh.'
 },
 'limitations':[
   'Full numerical SE(3) initialization; no live LiDAR-camera target calibration or reprojection validation has been performed.',
   'Optical principal point depth relative to the case is nominal; no measured uncertainty bound is available.',
   'Factory calibration was read today; no automatic pipeline calibration was run and no EEPROM write was performed. demov3 driver disabled automatic calibration.',
   'SolidWorks matrices describe design installation; manufacturing and mounting tolerances are not measured.',
   'demov3 recovered positions precede LiDAR corrections. Extrinsics do not recover the missing final localization trajectory.',
   'demov3 detector back-projected raw image box centres with K without distortion compensation; semantic points may contain lens-distortion and background-depth errors.'
 ],
 'sources':{
   'luxonis_coordinates':'https://docs.luxonis.com/software-v3/depthai/depthai-components/coordinate-systems',
   'oak_datasheet':'https://raw.githubusercontent.com/luxonis/oak-hardware/master/DM9098_OAK-D-Pro/Datasheet/OAK-D-Pro_Datasheet.pdf',
   'lidar_manual':'https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf',
   'ros_mesh':'https://github.com/luxonis/depthai-ros/blob/humble/depthai_descriptions/urdf/models/OAK-D-PRO.stl',
   'ros_frame_definition':'https://github.com/luxonis/depthai-ros/blob/humble/depthai_descriptions/urdf/include/depthai_macro.urdf.xacro',
   'housing_implementation':'https://github.com/luxonis/depthai-core/blob/v3.10.0/src/device/CalibrationHandler.cpp'
 }
}
save('camera_lidar_initial_extrinsics.json',result)

# Diagnostic projection only: previous row's updated Euler can be recovered by shifting one row.
log=np.loadtxt(DEMO/'semantic_map_work/recovered/mat_pre.txt')
times=np.load(DEMO/'semantic_map_work/recovered/cloud_timestamps.npy')[:-1,1]
rotations=Rotation.from_euler('xyz',log[1:,1:4]/57.3)
slerp=Slerp(times,rotations)
observations=json.loads((DEMO/'semantic_map_v1_20261028/semantic_observations.json').read_text(encoding='utf8'))
epoch=observations['relative_time_epoch_unix_s']
segments=[]
for e in observations['events']:
    points=[]
    for ob in e['observations']:
        stamp=epoch+ob[0]
        if ob[3] is None or not times[0]<=stamp<=times[-1]:continue
        i=min(max(1,int(np.searchsorted(times,stamp))),len(times)-1)
        if times[i]-times[i-1]>.25:continue
        pos=np.array([np.interp(stamp,times,log[:-1,j]) for j in [4,5,6]])
        t_imu_lidar=log[i,10:13]
        pc=np.array(ob[3]);pl=lidar_from_A[:3,:3]@pc+lidar_from_A[:3,3]
        pm=pos+slerp([stamp]).apply([pl+t_imu_lidar])[0]
        points.append(pm)
    if points:
        pts=np.array(points);centre=np.median(pts,axis=0)
        segments.append({'id':e['id'],'label':e['label'],'candidate_for_review':e['candidate_for_review'],'position_status':'approximate_diagnostic_not_verified','estimated_map_xyz_m':centre.tolist(),'projection_count':len(points),'spread_p90_m':float(np.percentile(np.linalg.norm(pts-centre,axis=1),90)),'review_note':e.get('visual_review_note',''),'representative_image':e['representative']['image']})
excluded=[e['id'] for e in observations['events'] if e['id'] not in {s['id'] for s in segments}]
save('demov3_projection_diagnostic.json',{'schema':'dog.cad_projection_diagnostic.v1','status':'approximate_diagnostic_not_verified','extrinsics_file':'camera_lidar_initial_extrinsics.json','trajectory':'prediction positions + shifted updated Euler; not final pose','map_frame':'dog_lio_map','segments_are_not_unique_objects':True,'excluded_segment_ids':excluded,'exclusion_reason':'No retained depth observation with time-bracketed recovered poses and scan gap <=0.25 s. Original observation file is preserved.','segments':segments})

xyz=np.load(DEMO/'semantic_map_v1_20261028/voxel_map_5cm.npz')['centroids_xyz']
fig,ax=plt.subplots(figsize=(11,9),dpi=160)
mask=xyz[:,2]<=1.8
ax.scatter(xyz[mask,0],xyz[mask,1],s=.6,color='#a5b4c3',alpha=.45,rasterized=True)
ax.plot(log[:,4],log[:,5],color='#505c6c',lw=1,label='Approximate recovered path')
colors={'fire':'#ed6e32','chemical':'#a98300','human':'#168b58'}
for label,color in colors.items():
    pool=[e for e in segments if e['label']==label and e['candidate_for_review']]
    if not pool:continue
    pp=np.array([e['estimated_map_xyz_m'] for e in pool]);ax.scatter(pp[:,0],pp[:,1],marker='x',s=65,lw=1.8,color=color,label=label+' candidates (unverified)')
    for e in pool:
        if label in ['human','fire']:
            pt=e['estimated_map_xyz_m'];ax.annotate(e['id'],(pt[0],pt[1]),xytext=(5,5),textcoords='offset points',fontsize=8,color=color)
ax.set(xlim=(-3,5),ylim=(-4,4),xlabel='Map X (m)',ylabel='Map Y (m)',title='demov3 | CAD initialization + predicted trajectory\nProjection diagnostic only; NOT a validated semantic map')
ax.set_aspect('equal');ax.legend(loc='upper right',fontsize=8)
ax.text(.01,.01,'Markers are observation segments, not unique objects.\nChemical includes suspected false positives. Empty = unknown.',transform=ax.transAxes,fontsize=8,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
fig.tight_layout();fig.savefig(ROOT/'demov3_projection_diagnostic.png');plt.close(fig)

report=f'''OAK-D Pro — MID-360 CAD 初始外参，2026-10-28

已完成：直接读取当前 SolidWorks dog.SLDASM 装配矩阵；提取传感器外壳几何；核对官方机械图；读取本机相机 EEPROM 出厂标定；计算完整 6 自由度初始变换。
用户确认：当前模型的安装位置和倾角与录制 demov3 时一致，之后没有调整。

1. 核实的机械测量
以 MID-360 原生点云坐标为基准：X 前、Y 左、Z 上。
RGB 前玻璃窗口中心：[144.426, -0.383, -51.019] mm。
也就是雷达测量原点前方约 144.4 mm、右侧 0.4 mm、下方 51.0 mm。
相机外壳光轴俯角约 {result['front_glass_pitch_down_deg']:.4f}°，偏航向右约 {-result['front_glass_yaw_left_deg']:.4f}°。
雷达测量原点按官方说明书为底部安装平面上方 47 mm；CAD 底平面 Y=-25.9171 mm，因此原点在雷达 CAD 坐标 Y=21.0829 mm。
相机玻璃零件中间圆窗为 RGB；外表面 Z=0.6 mm，使用真实子零件变换计算，未使用包围盒中心。

2. 光学外参初始值（不是联合实测标定结果）
p_lidar = R @ p_rgb_optical + t，输入输出均为米。
t = {np.round(lidar_from_A[:3,3],9).tolist()}
R =
{np.array2string(lidar_from_A[:3,:3],precision=9)}
四元数顺序 xyzw = {np.round(Rotation.from_matrix(lidar_from_A[:3,:3]).as_quat(),9).tolist()}
详见 camera_lidar_initial_extrinsics.json，其中保存逆变换、每一段坐标链、来源和检查值。

此初始值结合 CAD 安装、出厂相机间旋转/平移、出厂 housing 旋转与官方 ROS 模型的名义原点。
官方 ROS 模型与本 CAD 的后壳四个平面匹配，深度基准平移约 9.7309 mm；这是名义机械基准，不能保证等同于真实镜头主平面。
未把前玻璃中心直接称为光学原点，也未把此文件标为 calibrated=true。

3. 为什么没有直接采用 getHousingCalibration 返回值
这台设备是 {factory['productName']}，{factory['boardName']} / {factory['boardRev']}，DepthAI 3.10.0。
CAM_A、FRONT_CAM_A、VESA_A 三个不同参考点返回同一矩阵。
官方源码表明：型号没有对应的机械坐标条目时，会沿用已有 housing 数据；当前型号没有这些完整条目。
因此使用 EEPROM 中可信的旋转和相机间标定，物理位置由 CAD 和名义 ROS 几何补全。
API 和 EEPROM 的平移默认是厘米，本文件已转换为米。
读取过程中没有运行自动标定、没有写入相机 EEPROM；设备已释放。

4. demov3 初步投影检查
demov3_projection_diagnostic.png / .json 使用此初始外参与恢复的预测轨迹。
投影只是诊断：轨迹位置不是激光更新后的最终位置，识别框深度可能来自背景；69 个观察片段不能当成 69 个独立目标。
此次没有修改上一版语义地图，没有覆盖原始 demov3 数据，也没有将参数部署到冻结导航版本。
真正验收需要在多个距离和方位，把雷达点投到相机原图检查边缘重合，量化误差；必要时用联合标定板进一步优化外参。

5. 文件
solidworks_sensor_transforms.json：装配原始矩阵与路径。
solidworks_sensor_geometry.json：零件平面、圆柱面和包围信息；wrapper suppression=1 为轻量化，实体子零件=2 为已解析。
oak_factory_calibration.json / oak_calibration_report.json：只读导出的设备标定与接口结果。
official/：厂商资料、ROS 网格与代码，用于溯源。
build_extrinsics.py：可复核的全部计算与投影脚本。
'''
(ROOT/'README.txt').write_text(report,encoding='utf8')
for matrix in [lidar_from_front,lidar_from_A]:
    assert np.allclose(np.linalg.inv(matrix)@matrix,np.eye(4),atol=1e-9)
    assert np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-6)
basis=json.loads((ROOT/'solidworks_basis_validation.json').read_text(encoding='utf8'))
max_basis_error=0.
for row in basis:
    matrix=transform(parts[row['name']]['transform'])
    source=np.vstack([np.zeros(3),np.eye(3)])
    calculated=source@matrix[:3,:3].T+matrix[:3,3]
    max_basis_error=max(max_basis_error,float(np.max(abs(calculated-np.array(row['transformed_basis'])))))
assert max_basis_error<1e-12
save('validation.json',{'rigid_transform_checks_passed':True,'solidworks_native_point_transform_max_difference_m':max_basis_error,'four_mesh_reference_planes_agree':True,'input_camera_translation_cm_converted_to_m':True,'optical_frame_round_trip_passed':True,'diagnostic_segments':len(segments),'excluded_segments':len(excluded),'full_joint_calibration_verified':False})
save('SHA256SUMS.json',{str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ROOT.rglob('*')) if p.is_file() and p.name!='SHA256SUMS.json'})
print(json.dumps({'front_glass_mm':(lidar_from_front[:3,3]*1000).tolist(),'initial_optical_mm':(lidar_from_A[:3,3]*1000).tolist(),'pitch_down_deg':result['factory_adjusted_optical_pitch_down_deg'],'diagnostic_segments':len(segments)},ensure_ascii=False))
