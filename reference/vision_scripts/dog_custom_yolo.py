#!/usr/bin/env python3
"""Live RGB YOLO and time-matched aligned depth; no robot motion interfaces."""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from dog_yolo_core import Detector, NAMES

HTML = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>三类目标实时识别</title>
<style>body{font:16px system-ui;background:#111827;color:#e5e7eb;margin:24px}h1{font-size:26px}.views{display:flex;gap:16px;flex-wrap:wrap}section{flex:1;min-width:300px}img{width:100%;max-width:800px;background:#020617;aspect-ratio:1.6}pre{white-space:pre-wrap;color:#a7f3d0}small{color:#aab4c4}</style>
<h1>OAK-D Pro · 三类目标实时识别</h1><p>YOLOv8n · fire / chemical / human · 置信度阈值 0.40</p><p id="state">正在连接…</p>
<section><h2>识别结果</h2><img id="yolo"><pre id="detections">等待输出…</pre></section>
<div class="views"><section><h2>彩色画面</h2><img id="rgb"></section><section><h2>深度画面</h2><img id="depth"></section></div>
<pre id="info"></pre><small>当前模型使用 149 张素材训练，尚无独立测试集。Z 为目标框中心区域的相机前向深度估计；打印照片测得的是纸面距离。烟雾中的识别和测距仍需实测。</small>
<script>async function tick(){try{const s=await(await fetch('/status',{cache:'no-store'})).json();const ok=s.streams.rgb&&s.streams.depth&&s.streams.yolo&&Object.values(s.streams).every(x=>x.age_s<2);document.getElementById('state').textContent=ok?'● 相机与模型运行正常':'等待数据或数据已中断';document.getElementById('info').textContent=Object.entries(s.streams).map(([k,v])=>k+': '+v.fps.toFixed(1)+' fps').join(' | ')+'\n推理耗时: '+s.inference_ms.toFixed(1)+' ms';document.getElementById('detections').textContent=!ok?'检测流未就绪':s.objects.length?s.objects.map(o=>o.label+' '+(o.score*100).toFixed(0)+'% '+(o.xyz_m?'Z='+o.xyz_m[2].toFixed(2)+' m':'深度无效')).join('\n'):'当前没有超过阈值的目标';for(const k of ['rgb','depth','yolo'])document.getElementById(k).src='/'+k+'.jpg?t='+Date.now();}catch(e){document.getElementById('state').textContent='连接中断';}}tick();setInterval(tick,500);</script></html>'''

def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec/1e9

class Viewer(Node):
    def __init__(self, args):
        super().__init__('dog_custom_yolo', namespace='/dog_vision')
        self.detector = Detector(args.model)
        self.bridge, self.lock = CvBridge(), threading.Lock()
        self.directory = Path(args.directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.stop = threading.Event()
        self.rgb = None
        self.depths = deque(maxlen=20)
        self.camera = None
        self.jpeg, self.streams = {}, {}
        self.objects, self.errors = [], []
        self.observed = set()
        self.inference_ms = 0.
        self.total, self.nonempty, self.seq = 0, 0, 0
        self.last_saved = 0.
        root = '/dog_vision/dog_oak'
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, root+'/rgb/image_raw', self.on_rgb, qos)
        self.create_subscription(Image, root+'/stereo/image_raw', self.on_depth, qos)
        self.create_subscription(CameraInfo, root+'/rgb/camera_info', self.on_info, qos)
        self.detections_pub = self.create_publisher(Detection2DArray, '/dog_vision/dog_custom_yolo/detections', 5)
        self.image_pub = self.create_publisher(Image, '/dog_vision/dog_custom_yolo/image', 2)
        self.positions_pub = self.create_publisher(String, '/dog_vision/dog_custom_yolo/objects', 5)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        self.create_timer(5., self.persist)

    def error(self, exc):
        with self.lock:
            self.errors = (self.errors + [str(exc)])[-10:]
        self.get_logger().error(str(exc))

    def record(self, key):
        self.streams.setdefault(key, deque(maxlen=100)).append(time.monotonic())

    @staticmethod
    def encode(frame):
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
        return encoded.tobytes() if ok else b''

    def on_info(self, msg):
        with self.lock:
            self.camera = msg

    def on_rgb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            jpg = self.encode(frame)
            with self.lock:
                self.seq += 1
                self.rgb = (self.seq, msg, frame, time.monotonic())
                self.jpeg['rgb'] = jpg
                self.record('rgb')
        except Exception as exc:
            self.error(exc)

    def on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
            if msg.encoding == '16UC1':
                depth /= 1000
            elif msg.encoding != '32FC1':
                raise ValueError('Unknown depth encoding '+msg.encoding)
            valid = np.isfinite(depth) & (depth > 0)
            gray = (np.clip((np.nan_to_num(depth)-.3)/5.7, 0, 1)*255).astype(np.uint8)
            display = cv2.applyColorMap(255-gray, cv2.COLORMAP_TURBO)
            display[~valid] = 0
            jpg = self.encode(display)
            with self.lock:
                self.depths.append((stamp(msg), depth))
                self.jpeg['depth'] = jpg
                self.record('depth')
        except Exception as exc:
            self.error(exc)

    def run(self):
        processed = -1
        while not self.stop.wait(.005):
            with self.lock:
                rgb = self.rgb
                depths, camera = list(self.depths), self.camera
            if rgb is None or rgb[0] == processed:
                continue
            seq, msg, frame, received = rgb
            # Wait briefly for the same camera timestamp's depth message to arrive.
            if (not depths or depths[-1][0] < stamp(msg)-.03) and time.monotonic()-received < .12:
                continue
            processed = seq
            try:
                t0 = time.monotonic()
                objects = self.detector.infer(frame)
                elapsed = (time.monotonic()-t0)*1000
                pair = min(depths, key=lambda d: abs(d[0]-stamp(msg))) if depths else None
                delta = abs(pair[0]-stamp(msg)) if pair else None
                depth = pair[1] if pair and delta <= .03 and pair[1].shape == frame.shape[:2] else None
                dets = Detection2DArray()
                dets.header = msg.header
                display = frame.copy()
                for obj in objects:
                    x1,y1,x2,y2 = obj['bbox_xyxy']
                    cx,cy = (x1+x2)/2,(y1+y2)/2
                    obj.update(xyz_m=None, depth_pair_delta_ms=delta*1000 if delta is not None else None)
                    if depth is not None and camera is not None and camera.width == frame.shape[1] and camera.height == frame.shape[0] and camera.k[0]>0 and camera.k[4]>0:
                        rx,ry = max(2,int((x2-x1)*.1)),max(2,int((y2-y1)*.1))
                        roi = depth[max(0,int(cy)-ry):min(depth.shape[0],int(cy)+ry+1),max(0,int(cx)-rx):min(depth.shape[1],int(cx)+rx+1)]
                        valid = roi[np.isfinite(roi)&(roi>=.2)&(roi<=8.)]
                        if valid.size >= 5 and valid.size >= roi.size*.2:
                            z = float(np.median(valid))
                            obj['xyz_m'] = [(cx-camera.k[2])*z/camera.k[0],(cy-camera.k[5])*z/camera.k[4],z]
                    det = Detection2D()
                    det.header = msg.header
                    det.bbox.center.position.x, det.bbox.center.position.y = cx,cy
                    det.bbox.size_x,det.bbox.size_y = x2-x1,y2-y1
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = obj['label']
                    hypothesis.hypothesis.score = obj['score']
                    det.results.append(hypothesis)
                    dets.detections.append(det)
                    color = [(0,140,255),(40,220,240),(80,230,80)][obj['class_id']]
                    cv2.rectangle(display,(int(x1),int(y1)),(int(x2),int(y2)),color,2)
                    label = f"{obj['label']} {obj['score']:.2f}" + (f" Z={obj['xyz_m'][2]:.2f}m" if obj['xyz_m'] else ' depth N/A')
                    cv2.putText(display,label,(int(x1),max(18,int(y1)-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1,cv2.LINE_AA)
                cv2.putText(display,'YOLOv8n | fire / chemical / human',(8,frame.shape[0]-10),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
                annotated = self.bridge.cv2_to_imgmsg(display, 'bgr8')
                annotated.header = msg.header
                self.image_pub.publish(annotated)
                self.detections_pub.publish(dets)
                self.positions_pub.publish(String(data=json.dumps({'stamp':stamp(msg),'frame_id':msg.header.frame_id,'depth_method':'median of central 20 percent ROI; optical-frame metres','objects':objects}, allow_nan=False)))
                jpg = self.encode(display)
                with self.lock:
                    self.jpeg['yolo'] = jpg
                    self.objects = objects
                    self.inference_ms = elapsed
                    self.record('yolo')
                    self.total += 1
                    self.nonempty += bool(objects)
                    self.observed.update(o['label'] for o in objects)
                if objects and time.monotonic()-self.last_saved > 10:
                    for label in set(o['label'] for o in objects):
                        cv2.imwrite(str(self.directory/(label+'_live.jpg')),display)
                    self.last_saved = time.monotonic()
            except Exception as exc:
                self.error(exc)
                self.stop.wait(.5)

    def status(self):
        with self.lock:
            now = time.monotonic()
            streams = {}
            for key, times in self.streams.items():
                recent = [t for t in times if t > now-5]
                streams[key] = {'age_s':now-times[-1],'fps':(len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0}
            return {'model':'dog_fire_chemical_human_yolov8n.onnx','classes':NAMES,'threshold':.4,'backend':'ONNX Runtime CPU','streams':streams,'inference_ms':self.inference_ms,'objects':self.objects,'total_frames':self.total,'nonempty_frames':self.nonempty,'observed_labels':sorted(self.observed),'errors':self.errors,'independent_validation':False}

    def persist(self):
        tmp = self.directory/'status.tmp'
        tmp.write_text(json.dumps(self.status(),ensure_ascii=False,indent=2),encoding='utf-8')
        tmp.replace(self.directory/'status.json')
        with self.lock:
            images = self.jpeg.copy()
        for key, jpg in images.items():
            (self.directory/(key+'_latest.jpg')).write_bytes(jpg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',required=True)
    parser.add_argument('--directory',required=True)
    parser.add_argument('--port',type=int,default=8082)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Viewer(args)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/': body,typ = HTML.encode('utf-8'),'text/html; charset=utf-8'
            elif path == '/status': body,typ = json.dumps(node.status(),allow_nan=False).encode(),'application/json'
            elif path in ('/rgb.jpg','/depth.jpg','/yolo.jpg'):
                with node.lock: body = node.jpeg.get(path[1:-4])
                if body is None: self.send_error(503); return
                typ = 'image/jpeg'
            else: self.send_error(404); return
            self.send_response(200)
            self.send_header('Content-Type',typ)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError): pass
    server = ThreadingHTTPServer(('0.0.0.0',args.port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print('Custom three-class viewer listening on',args.port,flush=True)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop.set()
        node.worker.join(timeout=5)
        node.persist()
        server.shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
