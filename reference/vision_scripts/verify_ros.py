"""Read-only transport and timestamp check, kept separate from model evaluation."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

rclpy.init()
node=Node('dog_vision_verify',namespace='/dog_vision')
data={}
def receive(key,msg):
    entry=data.setdefault(key,{'stamps':[],'count':0})
    entry['count']+=1
    if hasattr(msg,'header'):
        entry['stamps'].append(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        entry['frame_id']=msg.header.frame_id
    if isinstance(msg,Image):
        entry.update(width=msg.width,height=msg.height,encoding=msg.encoding)
    elif isinstance(msg,CameraInfo):
        entry.update(width=msg.width,height=msg.height,k=list(msg.k))
    elif isinstance(msg,Detection2DArray):
        entry['last_objects']=[{'label':d.results[0].hypothesis.class_id,'score':d.results[0].hypothesis.score} for d in msg.detections]
    elif isinstance(msg,String):
        entry['last_objects']=json.loads(msg.data)
subs=[]
root='/dog_vision/'
for key,typ,topic in [('rgb',Image,'dog_oak/rgb/image_raw'),('depth',Image,'dog_oak/stereo/image_raw'),('info',CameraInfo,'dog_oak/rgb/camera_info'),('annotated',Image,'dog_custom_yolo/image'),('detections',Detection2DArray,'dog_custom_yolo/detections'),('objects',String,'dog_custom_yolo/objects')]:
    subs.append(node.create_subscription(typ,root+topic,lambda msg,k=key:receive(k,msg),QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE)))
until=time.monotonic()+15
while time.monotonic()<until:
    rclpy.spin_once(node,timeout_sec=.1)
assert all(data.get(k,{}).get('count',0)>5 for k in ['rgb','depth','info','annotated','detections','objects']),data
depth_stamps=data['depth']['stamps']
deltas=[min(abs(t-d) for d in depth_stamps)*1000 for t in data['rgb']['stamps'][3:-3]]
matching=sum(t in data['rgb']['stamps'] for t in data['detections']['stamps'])
report={'pass':True,'rgb_depth_delta_ms_max':max(deltas),'rgb_depth_delta_ms_mean':sum(deltas)/len(deltas),'detection_stamps_matching_received_rgb':matching,'streams':{k:{a:b for a,b in v.items() if a!='stamps'} for k,v in data.items()}}
assert max(deltas)<30,report
assert matching>5,report
Path('/home/robot/dog_vision_custom_20261026/evidence/ros_transport.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
node.destroy_node()
rclpy.shutdown()
