# The type of the shape
int32 POINT = 0
int32 BOX = 1
int32 SPHERE = 2
int32 ELLIPSOID = 3
int32 CAPSULE = 4
int32 CONE = 5
int32 CYLINDER = 6
int32 MESH = 7

int32 type

# Dimensions of the shape
# POINT: []
# BOX: [size_x, size_y, size_z]
# SPHERE: [radius]
# ELLIPSOID: [radius_x, radius_y, radius_z]
# CAPSULE: [radius, length]
# CONE: [radius, length]
# CYLINDER: [radius, length]
float64[] parameters

# Identifier of the mesh if type==MESH
string mesh_id
