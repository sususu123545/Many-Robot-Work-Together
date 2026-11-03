# Everything the policy is allowed to see, stamped once so that the consumer can
# tell a fresh observation from a coherent-looking stale one.
#
# The 14-dimensional internal state referenced by the design document is derived
# from this message by the bridge (6 joint angles + gripper + TCP position 3 +
# TCP quaternion 4); it is not stored here, because the message has to stay
# usable before any particular model's input convention is chosen.
std_msgs/Header header

# Natural-language instruction, verbatim. No templating, no re-writing.
string instruction

# Which physical robot configuration produced this observation, e.g.
# "armpi_pro_v1". Guards against replaying a dataset recorded on another arm.
string robot_config_id

# TCP pose in the action frame of the chunk this observation will be answered
# with. The chunk's deltas are relative to this pose, so it must be the pose at
# header.stamp, not the pose at request time.
geometry_msgs/Point tcp_position_m
geometry_msgs/Quaternion tcp_orientation

# Joint state in physical units (radians). Names are carried explicitly; the
# consumer checks the set rather than trusting the order.
string[] joint_names
float64[] joint_positions_rad

# Gripper opening, normalised to [0, 1] by the hardware model. 0 = jaws closed,
# 1 = mechanically fully open. On the ArmPi Pro this is NOT linear in servo
# ticks; see vla_controller.spec.gripper_tick_to_opening().
float64 gripper_opening

# Wrist (end-effector) camera. Compressed keeps the message small enough to be
# carried over the wired link without stalling the executor; an uncompressed
# variant is a later change, not a silent one.
builtin_interfaces/Time wrist_image_stamp
sensor_msgs/CompressedImage wrist_image

# Optional scene camera. Empty data means "no scene camera on this platform",
# which the policy adapter must treat differently from "camera died".
builtin_interfaces/Time front_image_stamp
sensor_msgs/CompressedImage front_image
