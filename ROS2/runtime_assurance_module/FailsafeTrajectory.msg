# Header for timestamp and frame info
std_msgs/Header header

# Dimensions
uint64 nx  # Size of state vector
uint64 nu  # Size of input vector
uint64 length  # Number of points in trajectory
uint64 n_obstacles  # Number of obstacles

# # Core trajectory data
int64[] times  # List of timestamps (size: length)

# Core trajectory data as matrices
std_msgs/Float64MultiArray states          # Matrix [length][nx]
std_msgs/Float64MultiArray inputs          # Matrix [length][nu]
std_msgs/Float64MultiArray safety_values   # Matrix [length][n_obstacles]

# Matrices for derivatives
std_msgs/Float64MultiArray sensitivity     # Matrix [length][nx][nx]
std_msgs/Float64MultiArray lfh            # Matrix [length][n_obstacles]
std_msgs/Float64MultiArray lgh            # Matrix [length][n_obstacles][nu]
