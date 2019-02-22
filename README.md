# mobility-plus-manipulation

## Goal

In this session, you'll make use of the navigation and manipulation modules you worked on in 
this week to perform a task: Navigating to a set of waypoints where your robot would find
an object of interest to collect.

## Task Description

You'll be using the Husky with ABB arm mounted on it from the 
[manipulation topic](https://github.com/ros-workshop/manipulation) yesterday, that will navigate to known waypoints and perform the task of
 
* Picking up a cube-object with AprilTag attached on its sides, and
* Dropping the cube-object on the robot's back.  

## How this all comes together
### Manipulation
Pull down the latest solution branch for manipulation. This will have a new robot with a Hokuyo LiDAR for slam navigation and mapping.
The Grasp is now a ROS Service, you can run it after starting with a ros service command.

### Spawn Additional Cubes

### Initialise gmapping and move_base
This is following the week 2 day 2 tutorial https://github.com/ros-workshop/slam-navigation
These nodes must be started when your simulated Husky is in the center of the Gazebo simulation

### Pull down this repo
This repo contains the mobility planner, which you'll need to modify to run the grasp service from the manipulation.

Start this node last, and see the robot move to the various locations in the locations.csv file.

