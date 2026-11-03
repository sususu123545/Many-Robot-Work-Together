# One action from a VLA policy.
#
# The first six components are a DELTA relative to the TCP pose carried in the
# observation the policy saw: metres for dx/dy/dz, radians for the rotation
# vector drx/dry/drz. They are not absolute targets and not joint commands.
#
# The seventh component is NOT a delta. `gripper` is an absolute target opening
# in [0, 1], where 1 means mechanically fully open. Openness is normalised
# through a non-linear hardware mapping, so a "delta opening" would not mean
# anything physical; the filter bounds the per-step change from the current
# opening instead.
#
# A wrong component order or a wrong unit here moves a real arm to a real place,
# so the order is fixed in vla_controller.spec.ACTION_COMPONENTS and callers
# should not re-derive it.
float64 dx
float64 dy
float64 dz
float64 drx
float64 dry
float64 drz
float64 gripper
