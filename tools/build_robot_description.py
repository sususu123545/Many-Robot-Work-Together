#!/usr/bin/env python3
"""Build the ArmPi Pro URDF: chassis + arm + camera, in one coherent TF tree.

Why generate it instead of hand-writing
---------------------------------------
The vendor's ArmPi-FPV URDF is the ARM only, and it hangs off a ``base_link``
that is really the FPV *desktop* base plate (177 x 288 mm).  The Pro is that arm
bolted to a mecanum chassis, so the tree the project actually needs is:

    base_footprint           on the floor, centre of the chassis
      └─ base_link           chassis reference frame
           ├─ chassis_*      plates, guard, bay, four wheels
           └─ arm_base       where the arm is bolted, offset FORWARD
                └─ link1 .. link5        (vendor chain, untouched)
                     ├─ grasping_frame   the tool point our FK returns
                     └─ camera_link      the eye-in-hand camera

``arm_base`` is deliberately named: it is the vendor frame every tool in
``tools/`` computes in, and making it an explicit link means the TF tree and the
Python code use the same word for the same thing.

Numbers and where they come from
--------------------------------
    FLOOR_Z = -0.094   MEASURED   floor below arm_base (this project)
    ARM_OFFSET_Y       user photos: arm sits at the front, overhanging
    chassis sizes      sim/chassis.py (published + inferred, see that file)
    arm chain          Hiwonder/ArmPi-FPV, verified to 0.000000 m against
                       arm_kinematics over 200 random poses (sim/urdf_chain.py)

    python3 tools/build_robot_description.py
    python3 tools/build_robot_description.py --out /tmp/x.urdf --print
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'sim'))

import chassis                                              # noqa: E402

VENDOR_URDF = os.path.join(REPO, 'sim', 'assets', 'armpi_fpv', 'urdf',
                           'armpi_fpv.urdf')
DEFAULT_OUT = os.path.join(REPO, 'ros2_ws', 'src', 'robot_description',
                           'urdf', 'armpi_pro.urdf')

ARM_OFFSET_Y = -chassis.CHASSIS_SHIFT_Y     # arm is FORWARD of the chassis centre
ARM_HEIGHT = -chassis.FLOOR_Z               # floor is below arm_base, so it is above

# joints in the vendor file that belong to the camera or the arm, kept verbatim
# joints that exist in the vendor URDF but are never published in /joint_states
# joint5 (wrist roll) is deliberately frozen too: arm_kinematics.py documents that
# the vendor IK is four-axis and callers must hold joint5 fixed, so letting the
# planner use it would make the URDF describe a robot the control code cannot
# reason about.  MoveIt duly planned joint5 to 562 ticks on the first successful
# run -- a 62-tick wrist twist our FK does not model at all.
FREEZE_JOINTS = {'joint5', 'l_joint', 'l_in_joint', 'l_out_joint',
                 'r_in_joint', 'r_out_joint'}
KEEP = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'camera_joint',
        'grasping_frame_joint', 'gripper_base_joint',
        'r_joint', 'r_in_joint', 'r_out_joint',
        'l_joint', 'l_in_joint', 'l_out_joint')


def fmt(values):
    return ' '.join('%.6g' % value for value in values)


def add_link(root, name, size=None, colour=None, mesh=None, mass=0.2, geom=True):
    link = ET.SubElement(root, 'link', {'name': name})
    if geom:
        for tag in ('visual', 'collision'):
            node = ET.SubElement(link, tag)
            ET.SubElement(node, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
            geometry = ET.SubElement(node, 'geometry')
            if size is not None:
                ET.SubElement(geometry, 'box', {'size': fmt(size)})
            if tag == 'visual' and colour:
                material = ET.SubElement(node, 'material', {'name': name + '_mat'})
                ET.SubElement(material, 'color', {'rgba': colour})
        if mesh:
            visual = link.find('visual')
            visual.find('geometry').clear()
            ET.SubElement(visual.find('geometry'), 'mesh', {'filename': mesh})
    if mass is not None:
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        ET.SubElement(inertial, 'mass', {'value': '%.3f' % mass})
        ET.SubElement(inertial, 'inertia', {'ixx': '0.001', 'ixy': '0', 'ixz': '0',
                                            'iyy': '0.001', 'iyz': '0', 'izz': '0.001'})
    return link


def add_joint(root, name, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0),
              kind='fixed', axis=None, limit=None):
    joint = ET.SubElement(root, 'joint', {'name': name, 'type': kind})
    ET.SubElement(joint, 'parent', {'link': parent})
    ET.SubElement(joint, 'child', {'link': child})
    ET.SubElement(joint, 'origin', {'xyz': fmt(xyz), 'rpy': fmt(rpy)})
    if axis is not None:
        ET.SubElement(joint, 'axis', {'xyz': fmt(axis)})
    if limit is not None:
        ET.SubElement(joint, 'limit', dict(zip(('lower', 'upper', 'effort', 'velocity'),
                                               ('%.4f' % v for v in limit))))
    return joint


def build():
    vendor = ET.parse(VENDOR_URDF).getroot()
    robot = ET.Element('robot', {'name': 'armpi_pro'})

    # --- ground and chassis reference -------------------------------------
    add_link(robot, 'base_footprint', mass=None, geom=False)
    add_link(robot, 'base_link', mass=None, geom=False)
    add_joint(robot, 'base_footprint_joint', 'base_footprint', 'base_link')

    # --- chassis body, expressed in base_link (chassis centre, floor frame) -
    lower_bottom, lower_top = chassis.LOWER_PLATE_Z
    upper_bottom, upper_top = chassis.UPPER_PLATE_Z
    width, length = chassis.PLATE_SIZE
    for name, z in (('chassis_lower', (lower_bottom, lower_top)),
                    ('chassis_upper', (upper_bottom, upper_top))):
        centre = (0.0, -chassis.CHASSIS_SHIFT_Y, (z[0] + z[1]) / 2.0 - chassis.FLOOR_Z)
        add_link(robot, name, size=(width, length, z[1] - z[0]),
                 colour='0.12 0.13 0.15 1', mass=0.6)
        add_joint(robot, name + '_joint', 'base_link', name, xyz=centre)

    bay = (0.0, chassis.BAY_Y - chassis.CHASSIS_SHIFT_Y,
           upper_top + chassis.BAY_SIZE[2] / 2.0 - chassis.FLOOR_Z)
    add_link(robot, 'chassis_bay', size=chassis.BAY_SIZE, colour='0.20 0.22 0.26 1',
             mass=0.4)
    add_joint(robot, 'chassis_bay_joint', 'base_link', 'chassis_bay', xyz=bay)

    guard = (0.0, chassis.GUARD_Y - chassis.CHASSIS_SHIFT_Y,
             lower_bottom + chassis.GUARD_SIZE[2] / 2.0 - chassis.FLOOR_Z)
    add_link(robot, 'chassis_guard', size=chassis.GUARD_SIZE,
             colour='0.15 0.16 0.19 1', mass=0.15)
    add_joint(robot, 'chassis_guard_joint', 'base_link', 'chassis_guard', xyz=guard)

    # --- four mecanum wheels ----------------------------------------------
    for sx, sy, tag in ((-1, 1, 'fl'), (1, 1, 'fr'), (-1, -1, 'rl'), (1, -1, 'rr')):
        name = 'wheel_' + tag
        link = add_link(robot, name, mass=0.25, geom=True)
        visual = link.find('visual')
        geometry = visual.find('geometry')
        ET.SubElement(geometry, 'cylinder', {'radius': '%.5f' % chassis.WHEEL_RADIUS,
                                             'length': '%.5f' % chassis.WHEEL_WIDTH})
        material = ET.SubElement(visual, 'material', {'name': name + '_mat'})
        ET.SubElement(material, 'color', {'rgba': '0.14 0.15 0.17 1'})
        origin = visual.find('origin')
        origin.set('rpy', '0 1.5707963 0')
        collision = link.find('collision')
        cgeometry = collision.find('geometry')
        ET.SubElement(cgeometry, 'cylinder', {'radius': '%.5f' % chassis.WHEEL_RADIUS,
                                              'length': '%.5f' % chassis.WHEEL_WIDTH})
        collision.find('origin').set('rpy', '0 1.5707963 0')
        xyz = (sx * chassis.WHEEL_X,
               sy * chassis.WHEEL_Y - chassis.CHASSIS_SHIFT_Y,
               chassis.WHEEL_Z - chassis.FLOOR_Z)
        # FIXED, not continuous: the chassis publishes /odom directly and never
        # reports wheel angles, so a continuous joint leaves move_group stuck on
        # "the complete state of the robot is not yet known" and its collision
        # model incomplete.  The wheels do not turn during a grasp.
        add_joint(robot, name + '_joint', 'base_link', name, xyz=xyz, kind='fixed')

    # --- arm mounting point -------------------------------------------------
    add_link(robot, 'arm_base', size=(0.10, 0.10, 0.008),
             colour='0.10 0.36 0.67 1', mass=0.3)
    add_joint(robot, 'arm_base_joint', 'base_link', 'arm_base',
              xyz=(0.0, ARM_OFFSET_Y, ARM_HEIGHT))

    # --- vendor arm chain, reparented onto arm_base ------------------------
    for joint in vendor.findall('joint'):
        name = joint.get('name')
        if name == 'base_joint':
            continue                       # that was the desktop base plate
        if name not in KEEP:
            continue
        copy = ET.SubElement(robot, 'joint', dict(joint.attrib))
        for child in joint:
            copy.append(child)
        if copy.find('parent').get('link') == 'base_link':
            copy.find('parent').set('link', 'arm_base')
        # The gripper's secondary links are continuous in the vendor file but the
        # robot only ever reports r_joint.  Anything unpublished makes move_group
        # refuse to plan ("complete state of the robot is not yet known"), and the
        # real gripper is one servo driving both jaws anyway, so freeze them.
        if name in FREEZE_JOINTS:
            copy.set('type', 'fixed')
            for tag in ('axis', 'limit'):
                node = copy.find(tag)
                if node is not None:
                    copy.remove(node)

    # The vendor file also carries its own base_footprint and base_link; both are
    # replaced here (the first by ours, the second by arm_base).  Copying them
    # makes urdf_parser reject the file with "link is not unique" -- caught by
    # running the real parser on the car, not by ElementTree.
    replaced = {'base_link', 'base_footprint'}
    for link in vendor.findall('link'):
        if link.get('name') in replaced:
            continue
        robot.append(link)

    ET.indent(robot, space='  ')
    return ET.tostring(robot, encoding='unicode')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', default=DEFAULT_OUT)
    parser.add_argument('--print', action='store_true', dest='show')
    args = parser.parse_args()

    text = build()
    print('links %d, joints %d' % (text.count('<link '), text.count('<joint ')))
    if args.show:
        print(text)
        return 0
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as handle:
        handle.write(text)
        handle.write('\n')
    print('written -> %s (%d bytes)' % (args.out, len(text)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
