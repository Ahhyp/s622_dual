#!/usr/bin/env python3
"""Render saved native collision geometry; no simulator or GUI required."""
import json
from pathlib import Path
import struct
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from c43_escape_audit import rotation


def mesh(i):
    b=(Path('src/s622_moveit_descriptions/meshes')/f'finger{i}.STL').read_bytes()
    n=struct.unpack_from('<I',b,80)[0]
    return np.array([struct.unpack_from('<9f',b,84+50*k+12) for k in range(n)]).reshape(-1,3,3)


def box_faces(pose):
    vertices=np.array([[x,y,z] for x in (-.06,.06) for y in (-.025,.025) for z in (-.025,.025)])
    world=(rotation(pose[3:])@vertices.T).T+pose[:3]
    return [[world[i]*1000 for i in face] for face in ((0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5))]


baseline=Path('log/c43_force_20261111/escape_native_force5_r1')
shift=Path('log/c43_escape_20261111/shift_x50_force5_r3')
b=json.loads((baseline/'native_escape_audit.json').read_text());s=json.loads((shift/'native_escape_audit.json').read_text())
events={e['event']:e for e in b['events']}
selections=[(baseline,b['prelift']['sim'],'Original: before lift',0),
    (baseline,events['pitch_1deg']['sim'],'Original: tilt starts at support edge',0),
    (baseline,events['native_opening_below_48mm']['sim'],'Original: later loss of enclosure',0),
    (shift,s['last']['sim'],'Shifted +50 mm: held after lift',.05)]
fig=plt.figure(figsize=(11,9));colors=['#0c8db5','#b13b96']
for k,(path,t,title,dx) in enumerate(selections,1):
    records=[json.loads(l) for l in (path/'native_poses.jsonl').open()]
    record=min((r for r in records if len(r['poses'])==4),key=lambda r:abs(r['sim']-t));poses=record['poses']
    ax=fig.add_subplot(2,2,k,projection='3d')
    ax.add_collection3d(Poly3DCollection(box_faces(poses['c43_sample']),facecolor='#e9a444',edgecolor='#7b4a0b',alpha=.45,linewidth=.7))
    for i in (1,2):
        pose=poses[f'left_finger{i}'];tri=mesh(i);world=(rotation(pose[3:])@tri.reshape(-1,3).T).T+pose[:3]
        ax.add_collection3d(Poly3DCollection(world.reshape(-1,3,3)*1000,facecolor=colors[i-1],edgecolor='#263747',alpha=.85,linewidth=.3))
    theta=np.linspace(0,2*np.pi,49);x=dx*1000+40*np.cos(theta);y=40*np.sin(theta)
    ax.add_collection3d(Poly3DCollection([list(zip(x,y,np.full(len(theta),250)))],facecolor='#a7afb8',alpha=.55))
    ax.plot(x,y,np.full(len(theta),250),color='#56606b',lw=1)
    for j in range(0,48,6):ax.plot([x[j],x[j]],[y[j],y[j]],[230,250],color='#87919d',lw=.7)
    p=np.array(poses['c43_sample'][:3])*1000;ax.scatter(*p,color='#a02910',s=18)
    r=rotation(poses['c43_sample'][3:]);pitch=np.degrees(np.arcsin(np.clip(-r[2,0],-1,1)))
    ax.set_title(title+f'\nt={record["sim"]:.3f}s, pitch={pitch:.2f} deg',fontsize=10,pad=6)
    ax.set(xlim=(-65,120),ylim=(-50,50),zlim=(230,400),xlabel='world X (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
    ax.set_box_aspect((185,100,170));ax.view_init(elev=19,azim=-57);ax.tick_params(labelsize=7)
fig.suptitle('C43: exact native poses + collision geometry',fontsize=15,y=.98)
fig.text(.5,.015,'Orange: sample | cyan/magenta: finger collision STL | grey: support. Saved simulation states; no GUI capture.',ha='center',fontsize=9)
fig.subplots_adjust(left=.02,right=.98,bottom=.06,top=.90,hspace=.20,wspace=.06)
out=Path(sys.argv[1]);fig.savefig(out.with_suffix('.png'),dpi=160);fig.savefig(out.with_suffix('.svg'))
print(out.with_suffix('.png'))
