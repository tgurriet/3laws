# This list must be of the same size.
# An index correspond to a valid place to use the zone.

# i.e if the break points are:
# float64[] vx_breakpoints = [-2,-1,0,1,2]
# float64[] vy_breakpoints = [-3,3]
# float64[] wz_breakpoints = [-0.3,-0.1, 0, 0.1, 0.3]

# And the validity are:
# uint64[] vx_zone_validity = [1,2,1,2]
# uint64[] vy_zone_validity = [0,0,0,0]
# uint64[] wz_zone_validity = [1,1,2,2]

# It means that this zones has to be used when the vx is between:
# - [-1,0](index 1 of the breakpoint list)
# - [0,1] (index 2 of the breakpoint list)
# AND when wz is between [-0.1, 0] and [0.1, 1] (index 1,2 of the wz_breakpoint list)

uint64[] vx_zone_validity
uint64[] vy_zone_validity
uint64[] wz_zone_validity
