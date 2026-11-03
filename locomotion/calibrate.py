"""Build and validate a reversible simulation baseline from the original USD conversion."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL = ROOT.parent / 'models' / 'zsl1'
LEGS = ['FL', 'FR', 'RL', 'RR']


def build():
    tree = ET.parse(MODEL / 'zsl1_matrix.xml')
    root = tree.getroot()
    root.set('model', 'ZSL1_sim_calibrated_v1')
    for body in root.findall('.//body'):
        for geom in list(body.findall('geom')):
            if geom.get('contype') != '0':
                body.remove(geom)
    def geom(body, name, typ, **attrs):
        ET.SubElement(body, 'geom', name=name, type=typ, contype='1', conaffinity='1',
                      group='3', density='0', rgba='.2 .5 .9 .35',
                      friction='.8 .02 .002', solref='.01 1', solimp='.9 .95 .001', **attrs)
    base = root.find(".//body[@name='base_link']")
    geom(base, 'base_collision', 'box', pos='.004 0 .015', size='.255 .095 .055')
    for leg in LEGS:
        sign = 1 if leg.endswith('L') else -1
        abad = root.find(f".//body[@name='{leg}_ABAD_LINK']")
        hip = root.find(f".//body[@name='{leg}_HIP_LINK']")
        knee = root.find(f".//body[@name='{leg}_KNEE_LINK']")
        foot = root.find(f".//body[@name='{leg}_FOOT_LINK']")
        geom(abad, f'{leg}_abad_collision', 'capsule', fromto=f'0 0 0 0 {sign*.07} 0', size='.032')
        geom(hip, f'{leg}_motor_collision', 'capsule', fromto=f'0 {-sign*.065} 0 0 0 0', size='.035')
        geom(hip, f'{leg}_thigh_collision', 'capsule', fromto='0 0 -.025 0 0 -.175', size='.024')
        geom(knee, f'{leg}_shin_collision', 'capsule', fromto='0 0 -.025 .006 0 -.185', size='.017')
        geom(foot, f'{leg}_foot_collision', 'sphere', pos='-.003 0 -.006', size='.028')
        ET.SubElement(foot, 'site', name=f'{leg}_sole', pos='-.003 0 -.034', size='.005')
    contact = ET.SubElement(root, 'contact')
    # Overlapping housings in the same hip assembly are mechanically connected.
    # All other nonadjacent robot links retain self collision.
    for leg in LEGS:
        ET.SubElement(contact, 'exclude', body1='base_link', body2=f'{leg}_HIP_LINK')
    for actuator in root.find('actuator'):
        actuator.set('forcelimited', 'true')
        actuator.set('forcerange', '-28 28')
    ET.indent(tree)
    tree.write(MODEL / 'zsl1_calibrated.xml', encoding='unicode')
    scene = ET.Element('mujoco', model='ZSL1_flat_training')
    ET.SubElement(scene, 'include', file='zsl1_calibrated.xml')
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='720')
    world = ET.SubElement(scene, 'worldbody')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='0 0 .1', contype='1', conaffinity='1',
                  friction='.8 .02 .002', rgba='.75 .8 .85 1', solref='.01 1')
    ET.SubElement(world, 'light', pos='0 0 3', dir='0 0 -1', diffuse='.8 .8 .8')
    ET.SubElement(world, 'camera', name='follow', mode='trackcom', pos='1 -1 .7', xyaxes='.7 .7 0 -.35 .35 1')
    ET.indent(scene)
    ET.ElementTree(scene).write(MODEL / 'train_scene.xml', encoding='unicode')
    parameters = {
        'status': 'simulation baseline, not physical system identification',
        'source': '/mnt/modol/zsl1_hifid.usd',
        'usd_values': {'torque_limit_nm': 28., 'joint_speed_limit_rad_s': 28.,
                       'mass_kg': 15.186161667, 'limits_and_inertias': 'preserved from source'},
        'assumptions': {'armature_kg_m2': .01, 'viscous_damping_nm_s_rad': .15,
                        'coulomb_friction_nm': .05, 'motor_lag_s': .004,
                        'foot_ground_friction': .8},
        'control': {'kp': 70., 'kd': 2., 'physics_dt': .002, 'policy_dt': .02},
        'randomization': {'friction': [.5, 1.1], 'mass_multiplier': [.9, 1.1],
                          'motor_strength': [.85, 1.0], 'motor_lag_s': [.002, .006]},
        'collision': {'self_collision': True, 'connected_housing_exclusions': 4,
                      'foot_radius_m': .028, 'fitting': 'primitives matched to CAD bounds and joint centers'}
    }
    (ROOT / 'parameters.json').write_text(json.dumps(parameters, indent=2))
    return parameters


def validate(parameters):
    m = mujoco.MjModel.from_xml_path(str(MODEL / 'train_scene.xml'))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    initial_self = [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                     mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2), float(c.dist))
                    for c in d.contact if m.geom_bodyid[c.geom1] and m.geom_bodyid[c.geom2]]
    home = np.tile([0., .8, -1.5], 4)
    rows = []
    max_pen = 0.
    for _ in range(5000):
        d.ctrl[:] = np.clip(70*(home-d.qpos[7:])-2*d.qvel[6:], -28, 28)
        mujoco.mj_step(m, d)
        if d.time > 2:
            rows.append([d.qpos[2], 2*np.arccos(min(1, np.hypot(d.qpos[3], d.qpos[6]))),
                         np.max(np.abs(home-d.qpos[7:]))])
            max_pen = max(max_pen, max([-float(c.dist) for c in d.contact] + [0]))
    arr = np.asarray(rows)
    # Fixed-base, gravity-free joint steps assess numerical response, not real motor dynamics.
    fixed = ET.parse(MODEL / 'zsl1_calibrated.xml')
    r = fixed.getroot(); b = r.find(".//body[@name='base_link']"); b.remove(b.find('freejoint'))
    r.remove(r.find('keyframe')); r.find('option').set('gravity', '0 0 0')
    r.find('compiler').set('meshdir', str(MODEL/'meshes'))
    fm = mujoco.MjModel.from_xml_string(ET.tostring(r, encoding='unicode'))
    steps = []
    for kp, kd in [(40., 1.5), (55., 2.), (70., 2.), (90., 3.)]:
        fd = mujoco.MjData(fm); target = home + np.tile([.08, .08, -.08], 4)
        errors = []; peak_tau = 0.
        for _ in range(1000):
            fd.ctrl[:] = np.clip(kp*(target-fd.qpos)-kd*fd.qvel, -28, 28)
            peak_tau = max(peak_tau, float(np.max(np.abs(fd.ctrl))))
            mujoco.mj_step(fm, fd)
            if fd.time > 1: errors.append(np.max(np.abs(target-fd.qpos)))
        steps.append({'kp': kp, 'kd': kd, 'last_second_max_error_rad': float(max(errors)), 'peak_torque_nm': peak_tau})
    warnings = [int(x.number) for x in d.warning]
    report = {'mass_kg': float(m.body_mass.sum()), 'actuators': m.nu,
              'initial_self_contacts': initial_self, 'warnings': warnings,
              'stand_seconds': 10, 'steady_height_min_m': float(arr[:,0].min()),
              'steady_height_max_m': float(arr[:,0].max()),
              'max_tilt_deg': float(np.rad2deg(arr[:,1].max())),
              'max_joint_error_rad': float(arr[:,2].max()), 'max_contact_penetration_m': max_pen,
              'joint_step_sweep': steps, 'physical_identification': False}
    report['passed'] = not initial_self and not any(warnings) and arr[:,0].min()>.28 and arr[:,1].max()<.1 and max_pen<.01
    (ROOT/'calibration-report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    if not report['passed']: raise RuntimeError('Calibration gates failed')


if __name__ == '__main__':
    validate(build())
