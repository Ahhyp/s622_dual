"""Record selected native Pose_V entries with native timestamps and pose ages."""
import json
import os
import re
import subprocess
import threading

WANTED={'c43_sample','left_wrist3_link','left_finger1','left_finger2'}
if os.environ.get('C43_LAB_MODE')=='dual_transport':
    WANTED.update({'right_wrist3_link','right_finger1','right_finger2'})


def blocks(text,field):
    for m in re.finditer(r'\b'+field+r'\s*\{',text):
        start=m.end();end=start;depth=1
        while end<len(text) and depth:
            depth+=(text[end]=='{')-(text[end]=='}');end+=1
        yield text[start:end-1]


def parse(text):
    stamp=next(blocks(text,'stamp'),'');d=dict(re.findall(r'\b(sec|nsec):\s*(\d+)',stamp))
    sim=int(d.get('sec',0))+1e-9*int(d.get('nsec',0));poses={}
    for b in blocks(text,'pose'):
        match=re.search(r'\bname:\s*"([^"]+)"',b)
        if not match:continue
        name=match.group(1).split('::')[-1]
        if name not in WANTED:continue
        pos=next(blocks(b,'position'),'');ori=next(blocks(b,'orientation'),'')
        p=dict(re.findall(r'\b([xyz]):\s*(\S+)',pos));q=dict(re.findall(r'\b([xyzw]):\s*(\S+)',ori))
        poses[name]=[float(p.get(k,0)) for k in 'xyz']+[float(q.get(k,0 if ori else (1 if k=='w' else 0))) for k in 'xyzw']
    return sim,poses


class NativePoseRecorder:
    def __init__(self,out):
        self.latest_sample=None
        self.file=(out/'native_poses.jsonl').open('w');self.examples=(out/'native_pose_examples.txt').open('w')
        self.proc=subprocess.Popen(['ign','topic','-e','-t','/world/c43_s4_world/pose/info'],
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        self.thread=threading.Thread(target=self.read,daemon=True);self.thread.start()

    def read(self):
        buf=[];latest={};times={};examples=0
        for line in self.proc.stdout:
            if line.strip():buf.append(line);continue
            if not buf:continue
            text=''.join(buf);buf=[];sim,poses=parse(text)
            for name,pose in poses.items():latest[name]=pose;times[name]=sim
            if "c43_sample" in poses:self.latest_sample=(sim,poses["c43_sample"])
            if examples<5 and WANTED.issubset(poses):self.examples.write(text+'\n');examples+=1
            if poses:self.file.write(json.dumps(dict(sim=sim,poses=latest,ages_s={n:sim-t for n,t in times.items()}))+'\n')

    def close(self):
        self.proc.terminate()
        try:self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
        self.thread.join(timeout=5);self.file.close();self.examples.close()
