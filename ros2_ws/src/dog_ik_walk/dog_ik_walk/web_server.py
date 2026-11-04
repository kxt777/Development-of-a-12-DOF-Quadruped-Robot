"""Same-LAN HTTP control bridge. No direct serial access or startup actuation."""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlsplit


class ControlAPI:
    def __init__(self,backend,vmax=.08,wmax=.15):
        self.backend=backend;self.vmax=vmax;self.wmax=wmax
        self.lock=threading.RLock();self.sessions={};self.owner=None;self.owner_until=0.;self.last_drive=None

    def session(self):
        token=secrets.token_urlsafe(24)
        with self.lock:
            if len(self.sessions)>=128:
                expired=[k for k,v in self.sessions.items() if time.monotonic()-v['seen']>3600 and k!=self.owner]
                for k in expired:del self.sessions[k]
                if len(self.sessions)>=128:raise ValueError('Too many control pages; restart unused browsers')
            self.sessions[token]=dict(seq=-1,seen=time.monotonic(),blocked=set())
        return dict(token=token,max_forward_m_s=self.vmax,max_yaw_rad_s=self.wmax,command_timeout_s=.5,stop_ramp_s=2.)

    def request(self,path,data):
        if path=='/api/action':
            # Never hold the lease lock while waiting for a ROS service; a
            # disable request must not queue behind a slow enable response.
            with self.lock:
                token=data.get('token');session=self.sessions.get(token);seq=data.get('seq')
                if session is None or type(seq) is not int or seq<=session['seq']:raise ValueError('页面会话或指令已失效')
                session.update(seq=seq,seen=time.monotonic())
                action=data.get('action');stamp=time.monotonic()
                if action not in ('enable','disable','stand','zero','teleop_stop'):raise ValueError('未知操作')
                if action not in ('disable','teleop_stop') and self.owner not in (None,token) and stamp<self.owner_until:raise ValueError('另一台手机正在控制')
                if self.last_drive:self.sessions[self.last_drive['token']]['blocked'].add(self.last_drive['gesture'])
                self.owner=None;self.owner_until=0.
            return self.backend.action(action)
        with self.lock:
            token=data.get('token');session=self.sessions.get(token);seq=data.get('seq')
            if session is None:raise ValueError('页面会话已失效，请刷新')
            if type(seq) is not int or seq<=session['seq']:raise ValueError('已丢弃过期指令')
            session.update(seq=seq,seen=time.monotonic())
            stamp=time.monotonic()
            if path!='/api/drive':raise ValueError('未知接口')
            gesture=data.get('gesture');active=data.get('active')
            if not isinstance(gesture,str) or not 1<=len(gesture)<=64 or type(active) is not bool:raise ValueError('无效摇杆指令')
            if not active:
                session['blocked'].add(gesture)
                if self.owner==token:
                    self.backend.velocity(dict(gesture=gesture,seq=seq,vx=0.,wz=0.,active=False,issued_monotonic=stamp))
                    self.owner=None;self.owner_until=0.
                return dict(ok=True,message='正在收步')
            if gesture in session['blocked']:raise ValueError('请松开摇杆后重新操作')
            if self.owner is not None and stamp>=self.owner_until and self.last_drive:
                self.sessions[self.last_drive['token']]['blocked'].add(self.last_drive['gesture'])
                self.owner=None
                if gesture in session['blocked']:raise ValueError('连接中断过，请松开摇杆后重试')
            if self.owner not in (None,token):raise ValueError('另一台手机正在控制')
            v=float(data.get('vx'));w=float(data.get('wz'))
            if not math.isfinite(v+w) or abs(v)>self.vmax+1e-9 or abs(w)>self.wmax+1e-9:raise ValueError('速度超出当前步态范围')
            state=self.backend.status()
            if not state.get('controller_online') or not state.get('enabled') or not state.get('stand_ready'):raise ValueError('请先使能并完成站立')
            self.owner=token;self.owner_until=stamp+.5;self.last_drive=dict(token=token,gesture=gesture)
            self.backend.velocity(dict(gesture=gesture,seq=seq,vx=v,wz=w,active=True,issued_monotonic=stamp))
            return dict(ok=True,message='速度指令已发送')


def make_handler(api,static):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,status,content,kind='application/json; charset=utf-8'):
            payload=json.dumps(content,ensure_ascii=False).encode() if isinstance(content,dict) else content
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(payload)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers();self.wfile.write(payload)
        def do_GET(self):
            path=urlsplit(self.path).path
            try:
                if path=='/api/session':return self.reply(200,api.session())
                if path=='/api/status':return self.reply(200,api.backend.status())
                assets={'/':('index.html','text/html; charset=utf-8'),'/app.js':('app.js','text/javascript; charset=utf-8'),'/style.css':('style.css','text/css; charset=utf-8')}
                if path not in assets:return self.reply(404,dict(error='Not found'))
                file,kind=assets[path];self.reply(200,(static/file).read_bytes(),kind)
            except (ValueError,OSError) as e:self.reply(503,dict(error=str(e)))
        def do_POST(self):
            try:
                origin=self.headers.get('Origin')
                if origin and urlsplit(origin).netloc!=self.headers.get('Host'):return self.reply(403,dict(error='仅接受本页面指令'))
                if self.headers.get('Content-Type','').split(';')[0]!='application/json':return self.reply(415,dict(error='JSON required'))
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=4096:return self.reply(413,dict(error='Invalid request size'))
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict):raise ValueError('Invalid request')
                self.reply(200,api.request(urlsplit(self.path).path,data))
            except (ValueError,TypeError,KeyError) as e:self.reply(409,dict(error=str(e)))
            except Exception as e:self.reply(503,dict(error=str(e)))
    return Handler


class MockBackend:
    def __init__(self):self.enabled=False;self.ready=False;self.last={}
    def status(self):return dict(mock=True,controller_online=True,enabled=self.enabled,stand_ready=self.ready,mode='hold',imu_fresh=True,yaw_deg=0.,heading_error_deg=0.,teleop=self.last,message='页面演示 · 未连接电机')
    def action(self,name):
        if name=='enable':self.enabled=True
        if name=='disable':self.enabled=False;self.ready=False
        if name=='stand':self.ready=self.enabled
        if name=='zero':self.ready=False
        if name=='teleop_stop':self.last={}
        return dict(ok=True,message='演示操作已接收')
    def velocity(self,data):self.last=data


def ros_backend():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    class Bridge(Node):
        def __init__(self):
            super().__init__('dog_web_bridge');self.latest={};self.received=0.;self.lock=threading.Lock()
            self.sub=self.create_subscription(String,'/dog_ik_walk_imu/status',self.receive,1)
            self.pub=self.create_publisher(String,'/dog_ik_walk_imu/teleop',1)
            self.action_clients={a:self.create_client(Trigger,'/dog_ik_walk_imu/'+a) for a in ('enable','disable','zero','stand','teleop_stop')}
        def receive(self,msg):
            try:s=json.loads(msg.data)
            except ValueError:return
            with self.lock:self.latest=s;self.received=time.monotonic()
        def status(self):
            with self.lock:s=self.latest;age=time.monotonic()-self.received
            h=s.get('heading',{});sample=h.get('sample') or {};correction=h.get('correction',{})
            return dict(controller_online=age<1.5,enabled=s.get('enabled',False),stand_ready=s.get('stand_ready',False),
                mode=s.get('mode'),imu_fresh=h.get('fresh',False) and age<1.5,
                yaw_deg=None if sample.get('yaw') is None else math.degrees(sample['yaw']),
                heading_error_deg=None if correction.get('heading_error_rad') is None else math.degrees(correction['heading_error_rad']),
                teleop=s.get('teleop',{}),message=s.get('message','等待 ROS 控制器'),disable_state=s.get('disable_state'),
                status_age_s=None if not self.received else age)
        def velocity(self,data):
            msg=String();msg.data=json.dumps(data);self.pub.publish(msg)
        def action(self,name):
            client=self.action_clients[name]
            if not client.service_is_ready():raise ValueError('ROS 控制接口未连接')
            future=client.call_async(Trigger.Request());done=threading.Event();future.add_done_callback(lambda _:done.set())
            if not done.wait(3):raise ValueError('响应超时，请查看实际电机状态，勿重复点击')
            result=future.result()
            if not result.success:raise ValueError(result.message)
            return dict(ok=True,message='请求已接收，请等待状态更新')
    rclpy.init();node=Bridge();threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start();return node


def main():
    p=argparse.ArgumentParser();p.add_argument('--host',default='0.0.0.0');p.add_argument('--port',type=int,default=8080)
    p.add_argument('--mock',action='store_true');p.add_argument('--static',type=Path,default=Path(__file__).parent/'web')
    args=p.parse_args();backend=MockBackend() if args.mock else ros_backend()
    server=ThreadingHTTPServer((args.host,args.port),make_handler(ControlAPI(backend),args.static));server.daemon_threads=True
    print(f'Dog control page listening on {args.host}:{args.port}; mock={args.mock}',flush=True)
    try:server.serve_forever()
    finally:server.server_close()


if __name__=='__main__':main()
