# The safety layer's answer for exactly one action. Published for every action
# that reaches the filter, accepted or not, so that a failed run can be read
# back instead of guessed at.
std_msgs/Header header

bool accepted

# Stable machine-readable code, e.g. "ok", "expired", "workspace",
# "unrealizable". Empty when accepted is false only in a programming error.
string reason_code

# Human-readable explanation, including the numbers that failed the check.
string reason

# Uniform scale the filter applied to the action before accepting it, 1.0 when
# nothing was limited. Actions are scaled as a whole; components are never
# clamped independently, because that bends the commanded path.
float64 scale_applied

# Component names (not indices) that were dropped as non-realizable, e.g.
# ["dry", "drz"]. Empty unless the deployment opted into projection.
string[] dropped_components

# The action as it will actually be executed. Zero when rejected.
vla_interfaces/VlaAction action_out

# True when this verdict latched the safety state: further actions are refused
# until an explicit reset. Rejections of a single action do not latch.
bool latched

string policy_id
