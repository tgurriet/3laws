# ##
# ## Severity level constants
# ##
# byte LOW=3 #debug level
# byte MEDIUM=2  #general level
# byte SEVERE=1  #warning level
# byte CRITICAL=0 #error level


# ##
# ## Type level constants
# ##
# byte SYSTEM   = 1
# byte HARDWARE = 2
# byte SAFETY   = 3
# byte CLOCK    = 4
# byte SENSOR   = 5
# byte SIGNAL   = 6
# byte TRACKING = 7
# byte DYNAMIC  = 8

# ##
# ## SubType level constants
# ##
# # system
# byte TIMEOUT = 10
# byte NOT_OK  = 11

# # Hardware
# byte FULL_RAM  = 20
# byte FULL_CPU  = 21
# byte FULL_DISK = 22

# # Collision
# byte COLLISION      = 30
# byte NEAR_COLLISION = 31

# # Clock
# byte RTC_OFFSET = 40
# byte UTC_OFFSET = 41

# # Sensor
# byte OBSTRUCTION = 50
# byte NOISE       = 51

# # Signal
# byte NAN_VALUES      = 60
# byte SUBNORMAL_FLOAT = 61
# byte INFINITE_VALUES = 62
# byte EXACT_ZERO      = 63
# byte BAD_TIMESTAMP   = 64

# # tracking
# byte TRACKING_ERROR = 70

# # Dynamic
# byte INPUT_DOMAIN     = 80
# byte STATE_DOMAIN     = 81
# byte STATE_DIFFERENCE = 82

# ##
# ## Fields
# ##

# byte type
# byte sub_type
# byte severity


bool in_progress
string name
string detail
string domain
