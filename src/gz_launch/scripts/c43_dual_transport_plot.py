#!/usr/bin/env python3
"""Plot recorded sample motion and measured wrist translation (no synthetic data)."""
import csv
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(run, destination):
    summary=json.loads((run/'summary.json').read_text())
    event=next(e for e in summary['events'] if e['event']=='prelift')
    t0=event['sim_last']; initial=np.array(event['detail']['sample'][:3])
    wrist0=np.array(event['detail']['wrists']['left'][:3])
    rows=[r for r in csv.DictReader((run/'telemetry.csv').open()) if r['sim'] and float(r['sim'])>=t0 and r['sample_z']]
    t=np.array([float(r['sim'])-t0 for r in rows])
    y=np.array([float(r['sample_y'])-initial[1] for r in rows])*1000
    z=np.array([float(r['sample_z'])-initial[2] for r in rows])*1000
    carry=np.array([r['phase'] in ('settle','lift','hold_lift','transfer','place','place_settle') for r in rows])
    wy=np.array([float(r['wrist_y'])-wrist0[1] for r in rows])*1000
    wz=np.array([float(r['wrist_z'])-wrist0[2] for r in rows])*1000
    wy[~carry]=np.nan;wz[~carry]=np.nan
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,3,figsize=(13,3.8),layout='constrained')
    for ax,data,wrist,title,target in zip(axes[:2],(z,y),(wz,wy),('A  Lift and placement','B  Lateral transport'),(60,120)):
        ax.plot(t,data,label='Sample (measured)',lw=2,color='#007f8b')
        ax.plot(t,wrist,label='Left wrist translation',lw=1.3,ls='--',color='#b65b20')
        ax.axhline(target,color='gray',lw=.7,ls=':');ax.set(xlabel='Time from prelift [s]',ylabel='Displacement [mm]',title=title)
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    ax=axes[2];ax.plot(y,z,color='#007f8b',lw=2)
    ax.scatter([0,120],[0,0],color=['#3867a6','#b65b20'],zorder=3,s=35)
    ax.annotate('Source',(0,0),xytext=(5,-17),textcoords='offset points')
    ax.annotate('Destination',(120,0),xytext=(-55,-17),textcoords='offset points')
    ax.set(xlabel='Lateral displacement Y [mm]',ylabel='Height above source [mm]',title='C  Actual sample path',ylim=(-8,70))
    ax.grid(alpha=.2)
    fig.suptitle('C43 physical dual-arm carry — '+run.name+' — four fixed 5 N drives',fontsize=13)
    destination.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(destination,dpi=180)
    plt.close(fig)


if __name__=='__main__':
    plot(Path(sys.argv[1]),Path(sys.argv[2]))
