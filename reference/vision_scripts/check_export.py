import argparse
import json
from pathlib import Path
import cv2

p = argparse.ArgumentParser()
p.add_argument('--reference',action='store_true')
a = p.parse_args()
if a.reference:
    from ultralytics import YOLO
    base=Path('/mnt/d/Project/dog/dog_vision_training/v1_20261026')
    model=YOLO(str(base/'dog_fire_chemical_human_yolov8n.pt'))
    results={}
    for cls in ['fire','chemical','human']:
        result=model.predict(str(base/'dataset/images/train'/f'{cls}__{cls}_01.jpg'),imgsz=640,rect=False,conf=.4,iou=.45,device='cpu',verbose=False)[0]
        results[cls]=[{'class_id':int(c),'score':float(s),'bbox_xyxy':box.tolist()} for c,s,box in zip(result.boxes.cls,result.boxes.conf,result.boxes.xyxy)]
    target=Path('/mnt/d/Project/dog/dog_vision_custom_20261026/evidence')
    target.mkdir(exist_ok=True)
    (target/'pytorch_reference.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results))
else:
    from dog_yolo_core import Detector
    base=Path('/home/robot/dog_vision_custom_20261026')
    model=Detector(base/'models/dog_fire_chemical_human_yolov8n.onnx')
    reference=json.loads((base/'evidence/pytorch_reference.json').read_text())
    report={}
    for cls in ['fire','chemical','human']:
        objects=model.infer(cv2.imread(str(base/'evidence'/f'{cls}_train_check.jpg')))
        assert any(o['label']==cls for o in objects), (cls,objects)
        assert len(objects)==len(reference[cls]), (cls,len(objects),len(reference[cls]))
        for ref,obj in zip(reference[cls],objects):
            assert ref['class_id']==obj['class_id']
            assert abs(ref['score']-obj['score']) < .001
            assert max(abs(a-b) for a,b in zip(ref['bbox_xyxy'],obj['bbox_xyxy'])) < 1.
        report[cls]=objects
    (base/'evidence/export_parity.json').write_text(json.dumps({'pass':True,'scope':'3 training images, deployment parity only; not independent accuracy validation','detections':report},indent=2))
    print(json.dumps(report))
