"""Add known collision fixtures while preserving the official-compatible ZSL1."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

SOURCE=Path('/mnt/llm/quadruped_dev/models/zsl1_official_adapter/xgb/xgb.xml')
OUT=Path('/mnt/llm/quadruped_dev/models/competition_official')

def build(case):
    tree=ET.parse(SOURCE);root=tree.getroot();world=root.find('worldbody');shapes=[]
    if case=='stairs12':
        for i,h in enumerate([.12,.24,.36,.36,.24,.12]):shapes.append((1.+(i+.5)*.35,0,h/2,.175,1.1,h/2,0))
        spec={'riser_m':.12,'tread_m':.35,'highest_surface_m':.36,'end_x_m':3.1}
    elif case=='slope20':
        up=math.radians(20);down=math.radians(18);length=1.;rise=length*math.tan(up);down_length=rise/math.tan(down);thick=.04
        shapes=[(1.+length/2,0,rise/2-thick/math.cos(up),length/(2*math.cos(up)),1.1,thick,-up),
            (1.+length+down_length/2,0,rise/2-thick/math.cos(down),down_length/(2*math.cos(down)),1.1,thick,down)]
        spec={'up_deg':20,'down_deg':18,'highest_surface_m':rise,'end_x_m':1+length+down_length}
    elif case=='gap15':
        angle=math.atan(.25);thick=.04
        shapes=[(1.4,0,.125-thick/math.cos(angle),1/(2*math.cos(angle)),1.1,thick,-angle),
            (2.075,0,.125,.175,1.1,.125,0),(2.675,0,.125,.275,1.1,.125,0),
            (3.45,0,.125-thick/math.cos(angle),1/(2*math.cos(angle)),1.1,thick,angle)]
        spec={'gap_width_m':.15,'gap_start_x_m':2.25,'gap_end_x_m':2.4,'platform_height_m':.25,'end_x_m':3.95}
    elif case=='recover_side':
        # Change only the initial joint pose. A home-body rotation and the same
        # joint-reference offset cancel for every common encoder configuration.
        for alias in ['FAR','RAR']:
            for joint,initial in [('HIP',2.2),('KNEE',-2.4)]:
                body=root.find(f".//body[@name='{alias}_{joint}_LINK']");hinge=body.find('joint')
                delta=initial-float(hinge.get('ref','0'));axis=np.fromstring(hinge.get('axis','0 0 1'),sep=' ')
                axis/=np.linalg.norm(axis);anchor=np.fromstring(hinge.get('pos','0 0 0'),sep=' ')
                oldq=np.fromstring(body.get('quat','1 0 0 0'),sep=' ');oldr=np.empty(9);mujoco.mju_quat2Mat(oldr,oldq);oldr=oldr.reshape(3,3)
                dq=np.empty(4);mujoco.mju_axisAngle2Quat(dq,axis,delta);dr=np.empty(9);mujoco.mju_quat2Mat(dr,dq);dr=dr.reshape(3,3)
                newq=np.empty(4);mujoco.mju_mulQuat(newq,oldq,dq)
                pos=np.fromstring(body.get('pos','0 0 0'),sep=' ')+oldr@(anchor-dr@anchor)
                body.set('pos',' '.join(map(str,pos)));body.set('quat',' '.join(map(str,newq)));hinge.set('ref',str(initial))
        spec={'initial_condition':'right legs folded while passive; fall must be observed','requires_measured_fallen_pose':True}
    else:raise ValueError(case)
    path=OUT/case/'xgb';path.mkdir(parents=True,exist_ok=True)
    vertices=[];faces=[]
    for i,(x,y,z,sx,sy,sz,angle) in enumerate(shapes):
        ET.SubElement(world,'geom',name=f'competition_{case}_{i}',type='box',pos=f'{x} {y} {z}',size=f'{sx} {sy} {sz}',
            euler=f'0 {angle} 0',contype='1',conaffinity='1',group='5',friction='.8 .02 .002',rgba='.25 .5 .7 1')
        rotation=np.array([[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]])
        base=len(vertices)
        for a,b,c in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]:
            vertices.append(rotation@np.array([a*sx,b*sy,c*sz])+[x,y,z])
        for f in [(1,3,2),(1,4,3),(5,6,7),(5,7,8),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,4,8),(3,8,7),(4,1,5),(4,5,8)]:faces.append([base+n for n in f])
    if shapes:
        obj=path/'terrain.obj';obj.write_text(''.join('v '+' '.join(map(str,v))+'\n' for v in vertices)+''.join('f '+' '.join(map(str,f))+'\n' for f in faces))
        ET.SubElement(root.find('asset'),'mesh',name='competition_terrain_visual',file=str(obj))
        ET.SubElement(world,'geom',name='competition_terrain_visual',type='mesh',mesh='competition_terrain_visual',group='2',contype='0',conaffinity='0',rgba='.25 .5 .7 1')
    output=path/'xgb.xml';ET.indent(tree,space='  ');tree.write(output,encoding='unicode')
    m=mujoco.MjModel.from_xml_path(str(output));original=mujoco.MjModel.from_xml_path(str(SOURCE))
    assert m.nu==original.nu==12 and m.nq==original.nq
    assert np.allclose(m.body_mass,original.body_mass) and np.allclose(m.body_inertia,original.body_inertia)
    if case=='recover_side':
        a=mujoco.MjData(original);b=mujoco.MjData(m);b.qpos[:]=a.qpos
        mujoco.mj_forward(original,a);mujoco.mj_forward(m,b)
        pose_error=float(np.max(np.abs(a.xpos-b.xpos)));rotation_error=float(np.max(np.abs(a.xmat-b.xmat)))
        assert pose_error<1e-7 and rotation_error<1e-7,(pose_error,rotation_error)
        spec.update(common_pose_position_error_m=pose_error,common_pose_rotation_error=rotation_error)
    report={'case':case,'model':str(output),'robot_physics_unchanged':True,'controller_id':'xgb','spec':spec}
    (path/'fixture.json').write_text(json.dumps(report,indent=2));return report

if __name__=='__main__':
    print(json.dumps([build(c) for c in ['stairs12','slope20']],indent=2))
