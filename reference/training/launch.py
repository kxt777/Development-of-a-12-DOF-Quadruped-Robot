from pathlib import Path
import subprocess, sys, json, os
root=Path('/mnt/d/Project/dog/dog_vision_training/v1_20261026')
state=root/'status.json'
if state.exists():
    s=json.loads(state.read_text())
    if s.get('state') in ('starting','training'):
        try:os.kill(s['pid'],0)
        except ProcessLookupError:pass
        else:raise RuntimeError('Training process already running')
log=open(root/'train.log','ab',buffering=0)
p=subprocess.Popen([sys.executable,'-u',str(root/'train.py')],stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True,cwd='/home/robot/dog_vision_train_v1')
(root/'launch.json').write_text(json.dumps({'pid':p.pid,'log':str(root/'train.log'),'python':sys.executable},indent=2))
print('LAUNCHED',p.pid)
