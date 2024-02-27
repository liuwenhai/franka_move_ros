from frankx import *

robot = Robot("172.16.0.2")

# Recover from errors
robot.recover_from_errors()

# Set velocity, acceleration and jerk to 5% of the maximum
robot.set_dynamic_rel(0.05)

# Alternatively, you can define each constraint individually
robot.velocity_rel = 0.2
robot.acceleration_rel = 0.1
robot.jerk_rel = 0.01

# Get the current pose
current_pose = robot.current_pose() # [x,y,z,r,p,y] "ZYX"
print(current_pose)