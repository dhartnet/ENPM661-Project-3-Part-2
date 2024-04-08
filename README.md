# ENPM-661-Project3
Project 3 Part 2 of ENPM-661 Planning For Autonomous Robots Course
# Written by: Rey Roque-Pérez and Dustin Hartnett
# UID: reyroque: 120498626
# UID: dhartnet: 115262249
Project 3 Part 2 of ENPM-661 Planning For Autonomous Robots Course

Implementation of the A* Algorithm for a Point Robot with ROS/Gazebo Simulation

# How to Run
Download the ros package to a location in your computer

In the terminal navigate to where the ros package is saved and then navigate to <project3_ws/src/turtlebot3_project3/scripts>

Build your package and launch "ros2 launch turtlebot3_project3 competition_world.launch.py"

Run "python3 A_ros.py" in a new terminal, but ensure you are also in the <project3_ws/src/turtlebot3_project3/scripts> location

Enter the goal coordinates as prompted. The code will check that the provided coordinates are within the workspace and not inside any obstacles.

To enter the goal coordinates, just enter a goal location with the units in centimeters. The x-coordinate will be prompted first, then the y-coordinate.

# Dependencies
The following Python libraries are used:

opencv-python: For visualizing the workspace and creating a video.

queue: For using the PriorityQueue function

numpy: For resizing the arrays used to draw the obstacles with opencv.

time: For calculating the duration of computation

geometry_msgs.msg: For gazebo implementation

rclpy: For gazebo implementation

# Output
A video file called a_star_output.mp4 will be generated in the same directory were the code is ran from after it is done running.

A video file showing our gazebo implementation is also in the Git repository

The following link also contains both videos in a Google Drive: https://drive.google.com/drive/folders/1C94iOVgEsiReOIYn9gn1Z_hnCKDnO6l6?usp=drive_link

# Github Repository: 
https://github.com/dhartnet/ENPM661-Project-3-Part-2 
