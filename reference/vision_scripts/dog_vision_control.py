#!/usr/bin/env python3
"""Start/stop only this isolated camera deployment, retaining process-group identity."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

BASE=Path('/home/robot/dog_vision_custom_20261026')
STATE=BASE/'processes.json'

def starttime(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()[19]
    except FileNotFoundError:
        return None

def live(entry):
    return starttime(entry['pid'])==entry['starttime']

def main():
    p=argparse.ArgumentParser()
    p.add_argument('action',choices=['start','stop','status'])
    action=p.parse_args().action
    state=json.loads(STATE.read_text()) if STATE.exists() else {}
    if action=='start':
        if any(live(e) for e in state.values()):
            raise SystemExit('Existing custom deployment is running; use status/stop first.')
        # Do not compete with another driver for exclusive camera access.
        processes=subprocess.check_output(['ps','-eo','args'],text=True)
        if any('depthai_ros_driver_v3' in line and ('driver.launch.py' in line or 'component_container' in line) for line in processes.splitlines()):
            raise SystemExit('Another OAK ROS driver is running.')
        import socket
        with socket.socket() as s:
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            try: s.bind(('0.0.0.0',8082))
            except OSError: raise SystemExit('Viewer port 8082 is occupied.')
        state={}
        for name in ['camera','viewer']:
            with (BASE/'logs'/f'{name}.log').open('ab') as log:
                process=subprocess.Popen(['bash',str(BASE/'scripts'/f'start_{name}.sh')],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            state[name]={'pid':process.pid,'starttime':starttime(process.pid)}
            STATE.write_text(json.dumps(state,indent=2))
        print(json.dumps(state))
    elif action=='stop':
        for entry in state.values():
            if live(entry): os.killpg(entry['pid'],signal.SIGINT)
        deadline=time.monotonic()+12
        while any(live(e) for e in state.values()) and time.monotonic()<deadline:
            time.sleep(.2)
        print(json.dumps({k:dict(v,running=live(v)) for k,v in state.items()}))
    else:
        print(json.dumps({k:dict(v,running=live(v)) for k,v in state.items()}))

if __name__=='__main__': main()
