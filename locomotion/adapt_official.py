"""Adapt the USD-derived MJCF interface to the installed official XGB controller."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as E
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent/'models/zsl1/zsl1_calibrated.xml'
OFFICIAL=Path('/mnt/llm/matrix_install/runtime/MATRiX_v1.1.2/UeSim/Content/model/xgb/xgb.xml')
OUTPUT=ROOT.parent/'models/zsl1_official_adapter/xgb'
ORDER=['FR','FL','RR','RL']
ALIASES={'FR':'FAR','FL':'FBL','RR':'RAR','RL':'RBL'}


def main():
    OUTPUT.mkdir(parents=True,exist_ok=True)
    tree=E.parse(SOURCE);root=tree.getroot()
    root.set('model','ZSL1_USD_XGB_interface')
    root.find('compiler').set('meshdir',str(SOURCE.parent/'meshes'))
    base=root.find(".//body[@name='base_link']")
    bodies={l:base.find(f"body[@name='{l}_ABAD_LINK']") for l in ORDER}
    for b in bodies.values():base.remove(b)
    for l in ORDER:base.append(bodies[l])
    mapping={}
    for l,a in ALIASES.items():
        for j in ['ABAD','HIP','KNEE']:
            mapping[f'{l}_{j}_LINK']=f'{a}_{j}_LINK'
            mapping[f'{l}_{j}_JOINT']=f'{a}_{j}_JOINT'
    # Rename only named physical interfaces and their references, never mesh assets.
    for node in root.iter():
        for attr in ['joint','body1','body2']:
            if node.get(attr) in mapping:node.set(attr,mapping[node.get(attr)])
        if node.tag in ['body','joint'] and node.get('name') in mapping:node.set('name',mapping[node.get('name')])
    actuators=root.find('actuator')
    old={a.get('joint'):a for a in actuators}
    actuators.clear()
    for l in ORDER:
        for j in ['ABAD','HIP','KNEE']:
            name=f'{ALIASES[l]}_{j}'
            a=old[name+'_JOINT'];a.set('name',name+'_LINK');actuators.append(a)
    sensors=root.find('sensor');sensors.clear()
    official=E.parse(OFFICIAL).getroot()
    for s in official.find('sensor'):sensors.append(E.fromstring(E.tostring(s)))
    # All legs share the same initial angles, so the home key remains valid after permutation.
    for key in root.find('keyframe'):
        key.set('qpos','0 0 .3663731965 1 0 0 0 '+' '.join(['0 .8 -1.5']*4))
    E.indent(tree);target=OUTPUT/'xgb.xml';tree.write(target,encoding='unicode')
    before=mujoco.MjModel.from_xml_path(str(SOURCE));after=mujoco.MjModel.from_xml_path(str(target))
    bd=mujoco.MjData(before);ad=mujoco.MjData(after);mujoco.mj_forward(before,bd);mujoco.mj_forward(after,ad)
    max_position=0.;max_axis=0.;max_inertia=0.
    for name,alias in mapping.items():
        if name.endswith('_LINK'):
            i=mujoco.mj_name2id(before,mujoco.mjtObj.mjOBJ_BODY,name);j=mujoco.mj_name2id(after,mujoco.mjtObj.mjOBJ_BODY,alias)
            max_position=max(max_position,float(np.max(np.abs(bd.xpos[i]-ad.xpos[j]))))
            max_inertia=max(max_inertia,float(np.max(np.abs(before.body_inertia[i]-after.body_inertia[j]))))
        else:
            i=mujoco.mj_name2id(before,mujoco.mjtObj.mjOBJ_JOINT,name);j=mujoco.mj_name2id(after,mujoco.mjtObj.mjOBJ_JOINT,alias)
            max_axis=max(max_axis,float(np.max(np.abs(before.jnt_axis[i]-after.jnt_axis[j]))))
    om=mujoco.MjModel.from_xml_path(str(OFFICIAL))
    joint_names=[mujoco.mj_id2name(after,mujoco.mjtObj.mjOBJ_JOINT,j) for j in range(1,after.njnt)]
    official_joints=[mujoco.mj_id2name(om,mujoco.mjtObj.mjOBJ_JOINT,j) for j in range(1,om.njnt)]
    # Unique small displacements verify all twelve sensor/actuator channels, not just home angles.
    ad.qpos[7:]+=np.linspace(-.006,.006,12);mujoco.mj_forward(after,ad)
    expected=ad.qpos[after.jnt_qposadr[after.actuator_trnid[:,0]]]
    sensor_error=float(np.max(np.abs(ad.sensordata[:12]-expected)))
    report={'source':str(SOURCE),'adapter':str(target),'official_template':str(OFFICIAL),
            'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            'adapter_sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
            'official_order':ORDER,'source_order':['FL','FR','RL','RR'],'permutation':[1,0,3,2],
            'mapping':mapping,'angle_signs':[1]*12,'angle_zero_offsets_rad':[0]*12,
            'joint_names_match_official':joint_names==official_joints,
            'max_body_position_change_m':max_position,'max_joint_axis_change':max_axis,
            'max_principal_inertia_change':max_inertia,'sensor_position_error_rad':sensor_error,
            'source_mass_kg':float(before.body_mass.sum()),'adapted_mass_kg':float(after.body_mass.sum()),
            'official_mass_kg':float(om.body_mass.sum()),'physical_model_preserved':True,
            'controller_motion_test_passed':False}
    report['interface_checks_passed']=joint_names==official_joints and max_position<1e-10 and max_axis<1e-10 and sensor_error<1e-10 and max_inertia<1e-10
    (OUTPUT/'adaptation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    if not report['interface_checks_passed']:raise RuntimeError('Interface validation failed')


if __name__=='__main__':main()
