from prepare_annotations import *
import hashlib
boxes=json.loads((OUT/'source_boxes.json').read_text())
overrides=json.loads((OUT/'manual_overrides.json').read_text()) if (OUT/'manual_overrides.json').exists() else {}
rows=[]
for cat in ('fire','chemical','human'):
    for p in sorted((ROOT/cat).glob('*.png')):
        rows.append(dict(file=str(p),category=cat,source_id=p.stem,box=boxes[p.stem],method='manual source object bounding box'))
for r in json.loads((OUT/'registrations.json').read_text()):
    p=Path(r['file']); im=load(p); h,w=im.shape[:2]
    if p.name in overrides:
        r.update(overrides[p.name]);r['method']='manual visual correction'
    elif r['H'] is not None:
        src=load(ROOT/r['category']/(r['source_id']+'.png'));sh,sw=src.shape[:2]
        a,b,c,d=boxes[r['source_id']]
        corners=np.float32([[a*sw,b*sh],[c*sw,b*sh],[c*sw,d*sh],[a*sw,d*sh]]).reshape(-1,1,2)
        pts=cv2.perspectiveTransform(corners,np.array(r['H'])).reshape(-1,2)
        r['box']=[max(0,float(pts[:,0].min()/w)),max(0,float(pts[:,1].min()/h)),min(1,float(pts[:,0].max()/w)),min(1,float(pts[:,1].max()/h))]
        r['method']='source object box projected by SIFT/RANSAC; visual review required'
    else:r['box']=None
    rows.append(r)
(OUT/'annotation_draft.json').write_text(json.dumps(rows,indent=2),encoding='utf8')
for cat in ('fire','chemical','human'):
    sheet([r for r in rows if r['category']==cat and r['file'].endswith('.jpg')],'boxes_'+cat)
sheet([r for r in rows if r['file'].endswith('.png')],'boxes_original')
if '--commit' in sys.argv:
    assert len(rows)==149 and all(r['box'] for r in rows)
    dest=OUT/'dataset'
    for sub in ('images/train','labels/train'): (dest/sub).mkdir(parents=True,exist_ok=True)
    for r in rows:
        cat=r['category'];cls=('fire','chemical','human').index(cat);p=Path(r['file'])
        name=cat+'__'+p.stem
        im=Image.fromarray(load(p));im.thumbnail((1600,1600));im.save(dest/'images/train'/(name+'.jpg'),quality=95)
        x1,y1,x2,y2=r['box'];assert 0<=x1<x2<=1 and 0<=y1<y2<=1
        (dest/'labels/train'/(name+'.txt')).write_text(f'{cls} {(x1+x2)/2:.6f} {(y1+y2)/2:.6f} {x2-x1:.6f} {y2-y1:.6f}\n')
        r['dataset_image']='images/train/'+name+'.jpg';r['original_sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    (dest/'data.yaml').write_text('path: /mnt/d/Project/dog/dog_vision_training/v1_20261026/dataset\ntrain: images/train\nval: images/train\nnames:\n  0: fire\n  1: chemical\n  2: human\n',encoding='utf8')
    (dest/'annotations_manifest.json').write_text(json.dumps(rows,indent=2),encoding='utf8')
    print('COMMITTED',len(rows), 'training images; val path aliases train for loader only; independent validation unavailable.')
