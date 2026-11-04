from pathlib import Path
import json, sys
import cv2
import numpy as np
from PIL import Image, ImageOps, ImageDraw
ROOT=Path('/mnt/d/Project/dog/训练照片-dog')
OUT=Path('/mnt/d/Project/dog/dog_vision_training/v1_20261026')
OUT.mkdir(parents=True,exist_ok=True)
def load(p):
    with Image.open(p) as im: return np.array(ImageOps.exif_transpose(im).convert('RGB'))
def small(a,maxdim=1200):
    s=min(1,maxdim/max(a.shape[:2])); return cv2.resize(a,None,fx=s,fy=s),s
def sheet(rows,name,cols=4):
    for start in range(0,len(rows),16):
        canvas=Image.new('RGB',(cols*400,((min(16,len(rows)-start)+cols-1)//cols)*340),'#dddddd'); draw=ImageDraw.Draw(canvas)
        for i,r in enumerate(rows[start:start+16]):
            arr=load(Path(r['file'])); im=Image.fromarray(arr); w,h=im.size
            if r.get('box'):
                d=ImageDraw.Draw(im); b=r['box']; d.rectangle([b[0]*w,b[1]*h,b[2]*w,b[3]*h],outline='lime',width=max(3,w//180))
            im.thumbnail((390,300)); x=(i%cols)*400;y=(i//cols)*340
            canvas.paste(im,(x+(400-im.width)//2,y+(300-im.height)//2))
            draw.text((x+4,y+303),f"{start+i+1:02d} {Path(r['file']).name}",fill='black')
            draw.text((x+4,y+319),(r.get('source_id') or 'UNMATCHED')+' '+str(r.get('inliers','')),fill='black')
        canvas.save(OUT/f'{name}_{start//16+1}.jpg',quality=94)

if __name__=='__main__':
    originals=[]
    for cat in ('fire','chemical','human'):
        rows=[dict(file=str(p)) for p in sorted((ROOT/cat).glob('*.png'))]
        sheet(rows,'original_'+cat)
        originals+=rows
    sift=cv2.SIFT_create(nfeatures=5000,contrastThreshold=0.015)
    refs={}
    for r in originals:
        p=Path(r['file']); a,s=small(load(p)); g=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY)
        k,d=sift.detectAndCompute(g,None); refs[p.stem]=(p.parent.name,k,d,a.shape,s)
    result=[]
    bf=cv2.BFMatcher()
    for cat in ('fire','chemical','human'):
        for p in sorted((ROOT/cat).glob('*.jpg')):
            a,scale=small(load(p)); gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY)
            k,d=sift.detectAndCompute(gray,None); candidates=[]
            for id,(rc,rk,rd,shape,rs) in refs.items():
                if rc!=cat or d is None or rd is None: continue
                pairs=bf.knnMatch(rd,d,k=2); good=[m for m,n in pairs if m.distance<0.72*n.distance]
                if len(good)<8:continue
                src=np.float32([rk[m.queryIdx].pt for m in good]); dst=np.float32([k[m.trainIdx].pt for m in good])
                H,mask=cv2.findHomography(src,dst,cv2.RANSAC,4)
                if H is None:continue
                count=int(mask.sum()); corners=np.float32([[0,0],[shape[1],0],[shape[1],shape[0]],[0,shape[0]]]).reshape(-1,1,2)
                projected=cv2.perspectiveTransform(corners,H).reshape(-1,2)
                area=cv2.contourArea(projected)
                if not cv2.isContourConvex(projected) or area<0.02*a.shape[0]*a.shape[1] or area>2*a.shape[0]*a.shape[1]:continue
                Hfull=np.diag([1/scale,1/scale,1])@H@np.diag([rs,rs,1])
                candidates.append(dict(source_id=id,inliers=count,H=Hfull.tolist()))
            best=max(candidates,key=lambda c:c['inliers']) if candidates else dict(source_id=None,inliers=0,H=None)
            row=dict(file=str(p),category=cat,**best);result.append(row)
            print(p.name,best['source_id'],best['inliers'],flush=True)
    (OUT/'registrations.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print('TOTAL',len(result),'LOW',sum(r['inliers']<12 for r in result),flush=True)
