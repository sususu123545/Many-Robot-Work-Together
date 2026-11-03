"""Build collidable MuJoCo terrain fixtures independently of the control policy."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np


def build(source,output,kind,level):
    tree=ET.parse(source);root=tree.getroot();root.set('model',f'ZSL1_{kind}_{level}')
    compiler=root.find('compiler');compiler.set('meshdir',str((source.parent/compiler.get('meshdir','')).resolve()))
    world=root.find('worldbody')
    ET.SubElement(world,'geom',name='terrain_floor',type='plane',size='0 0 .1',group='5',friction='.8 .02 .002',rgba='.65 .69 .72 1')
    ET.SubElement(world,'light',pos='0 0 5',dir='0 0 -1')
    ET.SubElement(world,'camera',name='follow',mode='trackcom',pos='1.5 -2 1',xyaxes='.8 .6 0 -.24 .32 .92')
    vis=ET.SubElement(root,'visual');ET.SubElement(vis,'global',offwidth='1280',offheight='720')
    probes=[]
    def box(name,x,y,z,sx,sy,sz,color='.3 .5 .7 1',angle=0):
        ET.SubElement(world,'geom',name=name,type='box',pos=f'{x} {y} {z}',size=f'{sx} {sy} {sz}',euler=f'0 {angle} 0',group='5',friction='.8 .02 .002',rgba=color)
    if kind=='stairs':
        height=[.02,.04,.06,.08,.10][level];run=.35
        # Solid steps ascend and descend; the plateau has no hidden gaps.
        heights=[height*i for i in [1,2,3,4,5,5,5,4,3,2,1]]
        for i,h in enumerate(heights):
            x=.9+(i+.5)*run;box(f'terrain_step_{i}',x,0,h/2,run/2,1.1,h/2,color=f'{.25+.04*(i%3)} .5 .7 1');probes.append([x,0,h])
        spec={'riser_m':height,'tread_m':run,'width_m':2.2,'highest_surface_m':max(heights)}
    elif kind=='slope':
        angle=math.radians([3,5,7,10,12][level]);half=1.25;thickness=.04
        rise=2*half*math.sin(angle)
        box('terrain_slope_up',.9+half*math.cos(angle),0,rise/2-thickness/math.cos(angle),half,1.1,thickness,angle=-angle)
        box('terrain_slope_down',.9+3*half*math.cos(angle),0,rise/2-thickness/math.cos(angle),half,1.1,thickness,angle=angle)
        spec={'angle_deg':math.degrees(angle),'highest_surface_m':rise}
        probes.append([.9+half*math.cos(angle),0,rise/2])
    elif kind=='rough':
        rng=np.random.default_rng(402+level);maximum=[.01,.02,.03,.04,.05][level]
        for i in range(16):
            for j in range(8):
                h=float(rng.uniform(.003,maximum));x=1.+(i+.5)*.25;y=(j-3.5)*.25
                box(f'terrain_rough_{i}_{j}',x,y,h/2,.125,.125,h/2,color='.4 .55 .35 1');probes.append([x,y,h])
        spec={'maximum_block_height_m':maximum,'block_width_m':.25}
    else:spec={'surface':'flat'}
    output.parent.mkdir(parents=True,exist_ok=True);ET.indent(tree,space='  ');tree.write(output,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(output));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    groups=np.zeros(6,dtype=np.uint8);groups[5]=1;errors=[]
    for x,y,h in probes:
        geom=np.array([-1],dtype=np.int32);distance=mujoco.mj_ray(model,data,np.array([x,y,3.]),np.array([0.,0.,-1.]),groups,1,-1,geom)
        actual=3-distance;errors.append(abs(actual-h))
    error=max(errors,default=0.)
    if error>1e-5:raise RuntimeError(f'Terrain ray verification failed: {error}')
    return {'kind':kind,'level':level,'path':str(output),'nq':model.nq,'nu':model.nu,'terrain_collision_geometry_count':int(np.sum(model.geom_group==5)),'ray_height_max_error_m':error,**spec}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=Path('/mnt/llm/quadruped_dev/models/zsl1/zsl1_calibrated.xml'));p.add_argument('--out',type=Path,default=Path('/mnt/llm/quadruped_dev/models/zsl1_terrains'));a=p.parse_args()
    reports=[build(a.source,a.out/'flat.xml','flat',0)]
    for kind in ['stairs','slope','rough']:
        for level in range(5):reports.append(build(a.source,a.out/f'{kind}-{level}.xml',kind,level))
    (a.out/'terrain-validation.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports,indent=2))
