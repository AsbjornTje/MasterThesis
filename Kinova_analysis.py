from roboticstoolbox import ERobot
from roboticstoolbox.backends.PyPlot import PyPlot
from spatialmath import SE3
import numpy as np
import time

# Load robot model
robot = ERobot.URDF(r"C:\Users\Asbjo\Desktop\kinova_assembly_description\urdf\kinova_assembly.urdf")

# Start pyplot
backend = PyPlot()
backend.launch()

# Add robot to backend
backend.add(robot)

# Define Cartesian goal poses (translation + rotation using SE3, converted to meters)
goal_poses = [
    SE3(1.466, 0.0, -1.583) * SE3.Rz(0), # Pose 1
    SE3(0.552, 0.0, -1.583) * SE3.Rx(0), # Pose 2
]

# Plot goal poses on the PyPlot axes
for pose in goal_poses:
    position = pose.t  # translational part (x, y, z)
    orientation = pose.R  # rotational part (3x3 matrix)
    
    # Plot the position as a point
    backend.ax.scatter(position[0], position[1], position[2], color='red', s=50, label='Goal Pose')
    
    # Plot a small coordinate frame for the orientation
    scale = 0.1  # Scale of the coordinate frame
    x_axis = position + orientation[:, 0] * scale 
    y_axis = position + orientation[:, 1] * scale  
    z_axis = position + orientation[:, 2] * scale 
    
    # Plot axes
    backend.ax.plot([position[0], x_axis[0]], [position[1], x_axis[1]], [position[2], x_axis[2]], color='red')
    backend.ax.plot([position[0], y_axis[0]], [position[1], y_axis[1]], [position[2], y_axis[2]], color='green')
    backend.ax.plot([position[0], z_axis[0]], [position[1], z_axis[1]], [position[2], z_axis[2]], color='blue')

# Add legend for goal poses
backend.ax.legend(["Goal Poses"])

# Pause to examine the visualization
input("Robot and goal poses visualized. Press Enter to continue...")

# Solve inverse kinematics for each pose to get joint configurations
goal_configs = []
for idx, pose in enumerate(goal_poses):
    ik_solution = robot.ikine_LM(pose)  # Solve IK
    if ik_solution.success:
        goal_configs.append(ik_solution.q)  # Append the joint configuration
    else:
        print(f"Failed to solve IK for pose {idx + 1}: {pose}")
        exit()

# Function to interpolate between joint configurations
def interpolate_joint_motion(q_start, q_goal, steps=50):
    return np.linspace(q_start, q_goal, steps)

# Move the robot through the goal configurations
for i in range(len(goal_configs)):
    q_goal = goal_configs[i]
    
    # If this is not the first pose, interpolate from the previous one
    if i > 0:
        q_start = goal_configs[i - 1]
        joint_trajectory = interpolate_joint_motion(q_start, q_goal, steps=50)
        for q in joint_trajectory:
            robot.q = q  # Update the robot's joint configuration
            backend.step()  # Update the visualization
            time.sleep(0.05)  # Add a small delay for smooth animation
    else:
        # For the first pose, just set the configuration
        robot.q = q_goal
        backend.step()
    
    # Pause at the goal pose
    print(f"Reached goal pose {i + 1}.")
    input("Press Enter to move to the next goal pose...")

# Hold the visualization open
backend.hold()
