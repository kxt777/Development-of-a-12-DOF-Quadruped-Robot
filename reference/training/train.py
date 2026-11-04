from pathlib import Path
import os, json, time, traceback, shutil
from datetime import datetime
from zoneinfo import ZoneInfo
import torch
from ultralytics import YOLO, __version__
from ultralytics.models.yolo.detect import DetectionTrainer
from ultralytics.utils.torch_utils import strip_optimizer

ROOT=Path('/mnt/d/Project/dog/dog_vision_training/v1_20261026')
START=time.time()
def status(**kwargs):
    p=ROOT/'status.json'
    old=json.loads(p.read_text()) if p.exists() else {}
    if kwargs.get('state')=='starting': old={}
    old.update(kwargs,updated_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat())
    tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(old,ensure_ascii=False,indent=2));tmp.replace(p)

class AllTrainingOnly(DetectionTrainer):
    """All samples train, as requested. Never score reused training data as validation."""
    def _setup_train(self):
        super()._setup_train()
        self.metrics={}
        self.fitness=-1e20
        # No validation-based best checkpoint selection.
        self.best_fitness=1e20
    def validate(self):
        return {}, self.fitness
    def final_eval(self):
        print('Independent validation omitted: all 149 supplied images are training data.',flush=True)

def epoch_done(trainer):
    trainer.metrics={}
    trainer.fitness=-sum(float(v) for v in trainer.label_loss_items(trainer.tloss).values())

def checkpoint_saved(trainer):
    losses={k:float(v) for k,v in trainer.label_loss_items(trainer.tloss).items()}
    status(state='training',epoch=int(trainer.epoch+1),epochs=trainer.epochs,losses=losses,elapsed_s=round(time.time()-START,1),checkpoint=str(trainer.last),independent_validation=False)
    print('DOG_PROGRESS '+json.dumps(dict(epoch=trainer.epoch+1,epochs=trainer.epochs,losses=losses)),flush=True)

if __name__=='__main__':
    try:
        assert torch.cuda.is_available(),'CUDA is required for this run'
        images=list((ROOT/'dataset/images/train').glob('*.jpg'))
        labels=list((ROOT/'dataset/labels/train').glob('*.txt'))
        assert len(images)==len(labels)==149
        counts={0:0,1:0,2:0}
        for p in labels:
            line=p.read_text().split();assert len(line)==5
            cls=int(line[0]); x,y,w,h=map(float,line[1:]);assert cls in counts and 0<x<1 and 0<y<1 and 0<w<=1 and 0<h<=1
            assert x-w/2>=-1e-5 and y-h/2>=-1e-5 and x+w/2<=1.00001 and y+h/2<=1.00001
            counts[cls]+=1
        params=dict(data=str(ROOT/'dataset/data.yaml'),epochs=80,imgsz=640,batch=8,device=0,workers=2,
            optimizer='AdamW',lr0=0.001,lrf=0.01,cos_lr=True,warmup_epochs=3,weight_decay=0.0005,
            patience=0,val=False,amp=True,cache=False,save=True,save_period=10,plots=False,
            seed=26,deterministic=True,project=str(ROOT/'runs'),name='yolov8n_all_train',exist_ok=False,
            mosaic=0.3,close_mosaic=10,degrees=15,translate=0.1,scale=0.3,fliplr=0.5,flipud=0.1,
            hsv_h=0.01,hsv_s=0.25,hsv_v=0.3,mixup=0.0)
        info=dict(model='yolov8n.pt',ultralytics=__version__,torch=torch.__version__,gpu=torch.cuda.get_device_name(0),classes=['fire','chemical','human'],class_counts=counts,training_images=149,independent_validation=False,validation_note='data.yaml val aliases train only to satisfy the framework loader; validation and final evaluation are disabled in AllTrainingOnly. No mAP or accuracy claims.',params=params)
        (ROOT/'training_config.json').write_text(json.dumps(info,indent=2))
        status(state='starting',pid=os.getpid(),epoch=0,epochs=80,**{k:v for k,v in info.items() if k!='params'})
        model=YOLO('/home/robot/dog_vision_train_v1/yolov8n.pt')
        model.add_callback('on_train_epoch_end',epoch_done)
        model.add_callback('on_model_save',checkpoint_saved)
        model.train(trainer=AllTrainingOnly,**params)
        final=ROOT/'dog_fire_chemical_human_yolov8n.pt'
        shutil.copy2(model.trainer.last,final)
        strip_optimizer(str(final))
        loaded=YOLO(str(final));assert loaded.names=={0:'fire',1:'chemical',2:'human'}
        sample=next((ROOT/'dataset/images/train').glob('fire__IMG*.jpg'))
        predictions=loaded.predict(str(sample),imgsz=640,device=0,verbose=False)
        assert len(predictions)==1
        status(state='completed',epoch=80,final_model=str(final),elapsed_s=round(time.time()-START,1),smoke_test='model load and inference succeeded on one training image; not independent evaluation')
    except Exception:
        status(state='failed',error=traceback.format_exc())
        raise
