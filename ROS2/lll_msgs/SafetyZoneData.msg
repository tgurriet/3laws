# Name of the safety zone (Unique)
string name


# Constants for zone types
uint8 TYPE_SAFETY = 0
uint8 TYPE_WARNING = 1
uint8 TYPE_MASKED = 2
uint8 TYPE_UNKNOWN = 3

# Type of zone (One of the above constants)
uint8 type

# Polygon defining the safety zone
geometry_msgs/Polygon zone

# This parameters represent tuple of points in the divided velocity space specified by the breakpoints
SafetyZoneParameters[<=1] prms
