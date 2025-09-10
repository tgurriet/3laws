std_msgs/Header header

uint64 nx  # Size of state vector
uint64 nu  # Size of input vector
uint64 length  # Number of points in trajectory

int64[] times  # List of timestamps (size: 'length')
float64[] states  # List of states (in column major format, size: 'nx*length')
float64[] inputs  # List of inputs (in column major format, size: 'nu*length') [optional]
