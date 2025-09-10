# This message define all zones associated to one sensor.

string sensor_id
SafetyZoneData[] safety_zones

# Break points will be used to divide the velocity space and assign zones to this velocity cubes
float64[] vx_breakpoints
float64[] vy_breakpoints
float64[] wz_breakpoints
