#!/usr/bin/env python3
"""Audit STL topology and the diagnostic three-box envelope fit."""
import collections
import json
import math
from pathlib import Path
import struct
import sys
import numpy as np

boxes=[(-.01,-.0072,.0365,.0615),(-.0072,-.0022,.026,.0615),(-.0072,.0251,.02,.026)]
a=math.radians(.5);r=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
report={}
for i in (1,2):
    b=(Path('src/s622_moveit_descriptions/meshes')/f'finger{i}.STL').read_bytes();n=struct.unpack_from('<I',b,80)[0]
    tri=np.array([struct.unpack_from('<9f',b,84+50*k+12) for k in range(n)]).reshape(-1,3,3)
    vertices,index=np.unique(tri.reshape(-1,3),axis=0,return_inverse=True);edges=collections.Counter()
    for face in index.reshape(-1,3):
        for x,y in zip(face,np.roll(face,-1)):edges[tuple(sorted((int(x),int(y))))]+=1
    sample=np.concatenate([tri.reshape(-1,3),tri.mean(1),(tri[:,0]+tri[:,1])/2,(tri[:,1]+tri[:,2])/2,(tri[:,2]+tri[:,0])/2])
    local=(r.T@sample.T).T;local[:,0]*=(1 if i==1 else -1);dist=[]
    for lo,hi,bottom,top in boxes:
        low=np.array([lo,bottom,-.01]);high=np.array([hi,top,.01])
        dist.append(np.linalg.norm(np.maximum(np.maximum(low-local,local-high),0),axis=1))
    report[f'finger{i}']=dict(triangles=n,unique_vertices=len(vertices),
        edge_incidence_histogram=dict(collections.Counter(edges.values())),
        signed_volume_m3=float(np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2])).sum()/6),
        boxes_volume_m3=sum((hi-lo)*(top-bottom)*.02 for lo,hi,bottom,top in boxes),
        envelope_test_points=len(sample),max_sample_outside_box_union_um=float(np.min(dist,axis=0).max()*1e6))
Path(sys.argv[1]).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
