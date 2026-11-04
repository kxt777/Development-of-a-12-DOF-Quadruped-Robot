"""Decode this recording's sensor_msgs/Image CDR with explicit bounds/shape checks."""
import json
from pathlib import Path
import sqlite3
import struct
import cv2
import numpy as np
from PIL import Image,ImageDraw

ROOT=Path('D:/Project/dog/demov3')
SRC=ROOT/'20261027_201820'
OUT=ROOT/'semantic_map_v1_20261028'
requests=json.loads((OUT/'keyframe_requests.json').read_text())
events=json.loads((OUT/'semantic_observations.json').read_text())
event_by_id={e['id']:e for e in events['events']}

class CDR:
    def __init__(self,data):
        assert data[:2]==b'\x00\x01',data[:4]
        self.data=data;self.offset=4
    def u32(self):
        self.offset=4+((self.offset-4+3)//4)*4
        v=struct.unpack_from('<I',self.data,self.offset)[0];self.offset+=4
        return v
    def string(self):
        n=self.u32();assert 0<n<4096
        value=self.data[self.offset:self.offset+n];self.offset+=n
        assert value[-1]==0
        return value[:-1].decode()
    def u8(self):
        v=self.data[self.offset];self.offset+=1;return v

def image_decode(data):
    c=CDR(data)
    sec,nanosec=c.u32(),c.u32()
    frame=c.string()
    h,w=c.u32(),c.u32()
    encoding=c.string();endian=c.u8();step=c.u32();n=c.u32()
    assert encoding=='bgr8' and endian==0 and step==w*3 and h==400 and w==640
    assert n==h*step and c.offset+n==len(data)
    pixels=np.frombuffer(data,np.uint8,count=n,offset=c.offset).reshape(h,w,3).copy()
    return sec+nanosec/1e9,frame,pixels

dbs=[sqlite3.connect(f'file:{p}?mode=ro',uri=True) for p in sorted((SRC/'rosbag').glob('*.db3'))]
all_images=[]
for di,db in enumerate(dbs):
    topicid=db.execute("SELECT id FROM topics WHERE name='/dog_vision/dog_custom_yolo/image'").fetchone()[0]
    all_images.extend((timestamp,di,rowid) for rowid,timestamp in db.execute('SELECT id,timestamp FROM messages WHERE topic_id=?',(topicid,)))
all_images.sort()
times=np.array([v[0]/1e9 for v in all_images])
thumbs=[]
for request in requests:
    # Choose using the RGB header timestamp, never assuming bag receipt time is the capture time.
    j=int(np.searchsorted(times,request['stamp']))
    candidates=[]
    for entry in all_images[max(0,j-12):min(len(all_images),j+12)]:
        data=dbs[entry[1]].execute('SELECT data FROM messages WHERE id=?',(entry[2],)).fetchone()[0]
        stamp,frame,img=image_decode(data)
        candidates.append((abs(stamp-request['stamp']),stamp,frame,img))
    delta,stamp,frame,img=min(candidates,key=lambda v:v[0])
    assert delta<.001,(request['id'],delta)
    name='keyframes/'+request['id']+'.jpg'
    cv2.imwrite(str(OUT/name),img,[cv2.IMWRITE_JPEG_QUALITY,92])
    event=event_by_id[request['id']]
    event['representative'].update(image=name,image_timestamp=stamp,image_match_delta_ms=delta*1000)
    tile=Image.new('RGB',(320,235),'white')
    tile.paste(Image.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB)).resize((320,200)),(0,0))
    ImageDraw.Draw(tile).text((5,203),f"{request['id']} {request['label']} {event['score_max']:.2f} t={request['time_s']:.1f}s",fill='black')
    ImageDraw.Draw(tile).text((5,218),'unregistered | '+('candidate' if event['candidate_for_review'] else 'low evidence'),fill='black')
    thumbs.append(tile)
for index in range(0,len(thumbs),9):
    part=thumbs[index:index+9]
    sheet=Image.new('RGB',(960,235*((len(part)+2)//3)),(235,235,235))
    for i,tile in enumerate(part):sheet.paste(tile,((i%3)*320,(i//3)*235))
    sheet.save(OUT/f'keyframe_review_{index//9+1}.jpg',quality=90)
(OUT/'semantic_observations.json').write_text(json.dumps(events,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
for db in dbs:db.close()
print('Extracted',len(thumbs),'keyframes with exact RGB timestamp matching.')
