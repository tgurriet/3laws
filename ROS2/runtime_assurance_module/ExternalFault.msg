std_msgs/Header header

# Source of the fault, must be one of the associated enum values
int32 source

# Detailed identifier for the fault source, can be left empty
string id

# Status of the fault, true if the fault is active, false if it is cleared
bool status

# Detailed description of the fault
string reason


int32 GLOBAL=0
int32 MOTION_PLANNING=1
int32 LOCALIZATION=2
int32 PERCEPTION=3
int32 HARDWARE=4
int32 COMPUTE=5
int32 NETWORK=6
int32 SENSORS=7
