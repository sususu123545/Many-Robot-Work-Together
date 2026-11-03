# A predicted sequence of H delta actions plus everything a consumer needs to
# decide whether it may still execute them.
#
# The validity window is explicit on purpose. A chunk is stale the moment the
# observation that produced it stops describing the world; a policy server that
# dies must not leave the robot replaying an old plan.
std_msgs/Header header

# Identifier of the serving policy/checkpoint. Logged with every verdict so a
# regression can be traced to a model version, not just to "the VLA".
string policy_id

# Must match the consumer's expected action space. A mismatch is a hard reject:
# two models with the same 7 numbers and different statistics will look fine and
# behave wrong.
string action_space_id

# Frame the deltas are expressed in, e.g. "arm_base". Never empty.
string action_frame

# Nominal time between actions, in seconds.
float64 control_dt

# How long after header.stamp the chunk stays executable. Zero or negative
# means it is already expired and must be rejected by any consumer.
builtin_interfaces/Duration validity

# H actions, oldest first.
vla_interfaces/VlaAction[] actions

# Seven entries, 1.0 = the platform can realise this component, 0.0 = it cannot.
# Lets a consumer reject or explicitly project instead of silently dropping a
# commanded rotation. Must be exactly 7 entries when supplied.
float64[] realizable_mask
