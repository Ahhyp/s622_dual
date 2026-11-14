"""Non-RT native-contact witness. No FCL in the control update, no safety certification."""
import math
import re
import numpy as np


def rotation(q):
    q=np.asarray(q,dtype=float);n=float(q@q)
    if not np.isfinite(q).all() or abs(n-1)>.01:raise ValueError('invalid sample quaternion')
    x,y,z,w=q/math.sqrt(n)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def points(block):
    out=[]
    for b in re.findall(r'\bposition\s*\{([^{}]*)\}',block):
        d=dict(re.findall(r'\b([xyz]):\s*([^\s}]+)',b))
        p=[float(d.get(k,0)) for k in 'xyz']
        if not all(math.isfinite(x) for x in p):raise ValueError('nonfinite contact point')
        out.append(p)
    return np.asarray(out,dtype=float).reshape((-1,3))


def disk_intersects_bottom(center,radius,sample,rot):
    # Exact projected rectangle/disk intersection, not a support bounding box.
    rectangle=np.array([[-.06,-.025,-.025],[.06,-.025,-.025],[.06,.025,-.025],[-.06,.025,-.025]])
    polygon=(rectangle@rot.T+np.asarray(sample[:3]))[:,:2];point=np.asarray(center[:2])
    signs=[];distance=float('inf')
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        edge=b-a;delta=point-a;cross=edge[0]*delta[1]-edge[1]*delta[0];signs.append(cross)
        t=np.clip(delta@edge/(edge@edge),0.,1.);distance=min(distance,float(np.linalg.norm(point-a-t*edge)))
    inside=min(signs)>=0 or max(signs)<=0
    return inside or distance<=radius


def contact_violation(record,sample,phase,source_allowed,supports,tolerance=.001):
    """Contact points must be on allowed faces; exact bottom footprint must intersect support disk. tolerance is existing 1mm support band.

    Source support is allowed during initial supported approach/grasp/liftoff only;
    destination is allowed from LOWER onward. Cylinder disk is exact, not its AABB.
    Finger names alone never exempt wrist/unknown links or a top/bottom-only contact.
    """
    r=rotation(sample[3:]);center=np.asarray(sample[:3]);half=np.array([.06,.025,.025])
    blocks=record.get('sample_contact_blocks')
    if blocks is None:raise ValueError('contact acquisition lacks per-pair geometry')
    for item in blocks:
        names=item['names']
        other=[n for n in names if n!='c43_sample::base_link::collision']
        if len(other)!=1:return 'invalid sample contact pair'
        name=other[0];p=points(item['raw_block'])
        if not len(p):return 'contact has no position witness'
        local=(p-center)@r
        if any(name==f's622_dual_arm::{side}_finger{i}::{side}_finger{i}_collision' for side in ('left','right') for i in (1,2)):
            if phase not in ('GRASP','LIFT','TRANSPORT','LOWER','RELEASE'):return 'finger contact outside grasp/release phase'
            sideface=(np.abs(np.abs(local[:,0])-half[0])<=tolerance)|(np.abs(np.abs(local[:,1])-half[1])<=tolerance)
            if not (sideface & (np.abs(local)<=half+tolerance).all(axis=1)).all():return 'finger contact outside sample side faces'
            continue
        support=next((s for s in supports if name==s['name']+'::base_link::collision'),None)
        if support:
            allowed=(support['name']=='c43_support' and source_allowed and phase in ('IDLE','APPROACH','GRASP','LIFT')) or (support['name']=='c43_destination' and phase in ('LOWER','RELEASE','RETREAT','COMPLETE'))
            if not allowed:return 'support contact outside allowed phase'
            disk=disk_intersects_bottom(support['center'],support['radius'],sample,r)
            top=np.abs(p[:,2]-support['top_z'])<=tolerance
            bottom=np.abs(local[:,2]+half[2])<=tolerance
            if not disk or not (top & bottom).all():return 'support side/bottom contact without exact bottom-face/top-disk intersection'
            continue
        return 'unlisted sample contact: '+name
    return ''


def obb_overlap(ca,ra,ha,cb,rb,hb):
    """Exact 15-axis SAT for finite OBBs; touching counts as overlap."""
    ca,ra,ha,cb,rb,hb=map(np.asarray,(ca,ra,ha,cb,rb,hb))
    if not all(np.isfinite(x).all() for x in (ca,ra,ha,cb,rb,hb)):raise ValueError('invalid OBB')
    axes=[*ra.T,*rb.T]+[np.cross(a,b) for a in ra.T for b in rb.T]
    delta=cb-ca
    for axis in axes:
        if axis@axis<1e-24:continue
        reach=np.abs(axis@ra)@ha+np.abs(axis@rb)@hb
        if abs(delta@axis)>reach:return False
    return True
