std_msgs/Header header

# Identifier of the object
string id

# Identifier of the type of the object
string type_id

# Object geometry, and pose of geometry in object frame
ObjectGeometry geometry

# Object pose w.r.t world frame
geometry_msgs/PoseWithCovariance pose

# Object velocity w.r.t world frame, expressed in object frame
geometry_msgs/TwistWithCovariance velocity
