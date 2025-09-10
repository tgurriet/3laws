builtin_interfaces/Time start_time # in unix nanosecond timestamp (when this changes, consider previous task as finished)
string incident_type
uint64 expected_duration_ms # Duration in millisecond
float64 expected_distance_linear # Expected length in meter
float64 expected_distance_angular # Expected total angular distance in radian
string[] tags # Arbitrary list of tag to sort and query tasks
