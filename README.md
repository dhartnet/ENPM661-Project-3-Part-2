# ENPM-661-Project3
Project 3 Part 2 of ENPM-661 Planning For Autonomous Robots Course
# Written by: Rey Roque-Pérez and Dustin Hartnett
# UID: reyroque: 120498626
# UID: dhartnet: 115262249
Project 3 Part 2 of ENPM-661 Planning For Autonomous Robots Course

Implementation of the A* Algorithm for a Point Robot with ROS/Gazebo Simulation

# How to Run
Create a ros environment on your computer following the instructions provided to us here: https://github.com/shantanuparabumd/turtlebot3_project3

Save and move both "A_star.py" and "A_star_ros.py" to the ros environment and save them in the "scripts" folder

Open the competition world with the robot spawned

Open both "A_star.py" and "A_star_ros.py"

Run the "A_star_ros.py" program by opening it in an IDE. Ensure "A_star.py" is also opened so it can run simultaneously. 

Enter the goal coordinates as prompted. The code will check that the provided coordinates are within the workspace and not inside any obstacles.

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

A video file showing our gazebo implementation are in the following google drive folder and in the Git repository
https://drive.google.com/drive/folders/1C94iOVgEsiReOIYn9gn1Z_hnCKDnO6l6?usp=drive_link 

# Github Repository: 
https://github.com/dhartnet/ENPM661-Project-3-Part-2 
