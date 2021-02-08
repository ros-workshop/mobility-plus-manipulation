# Manipulation

**Goal:**

The session aims to familiaries you with one of the most crucial aspects of modern robotics, *manipulation*.

**What do we mean by manipulation?**

Whenever a robots physically interacts with its surrounding and modifies its environement, we talk about manipulation.

**What are the challenges of robotic manipulation?**
+ *Path Planning:* Generally, a robotic arm is used for manipulation ([ABB IRB 120](https://new.abb.com/products/robotics/industrial-robots/irb-120), [UR5](https://www.scottautomation.com/products/ur5-universal-robot/)). An arm often has between 5 and 7 Dof. Powerful planning algorithms must be used ([RRTs](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) for example) to find a path in joint or end-effector space.
+ *Grasping:* Once the arm knows how to travel from one place to another, its time to actually grasping something. Grasping in a vast and vibrant research topic mostly because how challenigng it be be for robots to find suitable grasp poses fro everyday objects.


**What tools do we have to solve this problem?**

+ *ROS:* ROS allows us to integrate all aspects needed for manipulation.
+ *[Moveit](https://moveit.ros.org/):* Moveit is a ROS package for motion planning build on top of libraries such as [OMPL](http://ompl.kavrakilab.org/).

**What else do we need to know?**
+ Manipulation works only if the robot knows what it looks like so we will be using URDFs
+ The [MoveGroup](http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html) class will be useful.

## Let's begin

### Workspace
By now you should have already set up a catkin workspace. Clone this repo in `workshop_ws/src`.

<details><summary>Click for Hint</summary>
  
<p>

```python
cd ~/workshop_ws/src
git clone https://github.com/ros-workshop/manipulation.git
```
</p>

</details>

### Dependencies

There are quite a few packages that need to be installed for the session:

+ moveit_ros
+ moveit_core
+ apriltag2_ros
+ image_geometry
+ husky_description
+ controller_manager
+ gazebo_ros_control
+ abb_resources

<details><summary>Click for Hint</summary>
  
You can `sudo apt-get install` almost all of these packages but you will have to clone [apriltag2_ros](https://github.com/dmalyuta/apriltags2_ros)

<br/>

<details><summary>Click to cheat!</summary>
  
```
sudo apt install ros-kinetic-moveit-ros ros-kinetic-moveit-core \
                 ros-kinetic-image-geometry ros-kinetic-husky-description \
                 ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control \
                 ros-kinetic-abb-resources
cd ~/workshop_ws/src
git clone https://github.com/dmalyuta/apriltags2_ros.git
```

</details>
</details>

## Everything a Robot needs

**ACTION**
Launch the robot in gazebo. 

<details><summary>Click for Hint</summary>
  
<p>
  
```
workshop_ws
│   devel
│   build   
└───src
│   │   pkg1
│   │   pkg2
│   └───robot_pkg
│       │   robot_description
|       └───robot_gazebo
|           |   robot_spawn.launch
|           |   robot.launch
```

</p> 

</details>
<br>

You should see something like this:

![robot_gazebo](./resources/images/gazebo_robot)

The arm and fingers should stand up straight and be static.

### Controllers

Each joint or motor in the arm and hand needs a controller for actuation. Ask yourself this: What controllers do I expect for a mobile robotic arm with an end-effector? Make sure the controllers have loaded appropriatly. 

**ACTION**
Check which controllers are loaded.

<details><summary>Click for Hint</summary>
  
<p>
There should be a least 3 controllers.
  
+ Mobile Base Controller
+ Arm Controller
+ Hand Controller

There is a useful rqt plugin to check your controllers 

![controller_manager](./resources/images/controller_manager)

</p> 

<details><summary>Click to cheat!</summary>

Install and run [rqt_controller_manager](http://wiki.ros.org/rqt_controller_manager)

```
sudo apt install ros-kinetic-rqt-controller-manager  
source /opt/ros/kinetic/setup.bash 
rosrun rqt_controller_manager rqt_controller_manager 
```

</details>
</details>
<br>

Now to see the controllers in action, jog the arm and fingers manually.

**ACTION**
Move the arm and fingers.

<details><summary>Click for Hint</summary>
  
<p>

There is an rqt plugin for that!

![traj_controller](./resources/images/traj_controller)

<details><summary>Click to cheat!</summary>

Install and run [rqt_joint_trajectory_controller](http://wiki.ros.org/rqt_joint_trajectory_controller)

```
sudo apt install ros-kinetic-rqt-joint-trajectory*
source /opt/ros/kinetic/setup.bash 
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

</details>

</details>

### Joint States

**ACTION** Check that you are getting feedback from your robots `joint_states`.

<details><summary>Click for Hint</summary>

Use `rostopic echo /joint_states` and you should see something like:

```
header: 
  seq: 206558
  stamp: 
    secs: 3364
    nsecs: 470000000
  frame_id: ''
name: [abb2_joint_1, abb2_joint_2, abb2_joint_3, abb2_joint_4, abb2_joint_5, abb2_joint_6,
  finger_1_dist_joint, finger_1_med_joint, finger_1_prox_joint, finger_2_dist_joint,
  finger_2_med_joint, finger_2_prox_joint, finger_3_dist_joint, finger_3_med_joint,
  front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
position: [0.00032209507270231086, 0.029705080560157526, -0.045566170951020446, 0.0013755875530101491, 0.011665304611981675, 0.0042873037305675155, 0.6000000000889099, 9.921607535545718e-09, -1.3813679089480502e-09, 0.6000000000623906, 9.533138722872536e-09, 4.088242899769057e-08, 0.5999999999707182, 8.249177341212999e-08, -3.791679371458411, -3.7625546328459993, -3.82143007050297, -3.825014721779551]
velocity: [0.0031072944551323338, -0.00024589280552260895, -0.0003502345108150297, -0.01642694515454311, 0.0002473546599235343, 0.015647407170457883, 8.890796740994584e-09, 1.4727589797510176e-08, -5.6046476257774576e-08, 6.238657163250017e-09, 1.0317774130493553e-08, -1.666632588062669e-07, -2.9254809393228006e-09, -1.212269241301142e-07, 9.066189840782844e-05, 0.00020114626581779229, 8.929489784498638e-05, 0.0001963030063448588]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

</details>

### Kinect

**ACTION** Check that you are getting images and point clouds from the simulated Kinect sensor

<details><summary>Click for Hint</summary>

View this image topic: `/kinect2/rgb/image_raw`

View this Pointcloud topic: `/kinect2/depth_registered/points` (e.g. using `rviz`)

</details>

## MoveIt

Bringing up the robot in gazebo and start moveit.

**ACTION**
Launch the robot in gazebo and launch the `moveit-planning-execution-gazebo` pkg.

<details><summary>Click for Hint</summary>
  
<p>
  
```
workshop_ws
│   devel
│   build   
└───src
│   │   pkg1
│   │   pkg2
│   └───robot_pkg
│       │   robot_description
|       └───robot_moveit_config
|           |   moveit_planning_execution_gazebo.launch
|           |   demo.launch
```

</p> 

<details><summary>Click to cheat</summary>

```
roslaunch husky_abb_moveit_config moveit_planning_execution_gazebo.launch                                                    
```

</details>
</details>
<br>

An rviz window will pop up. In Rviz, add the Motion Planning plugin. Once added you should have something like this:

![rviz1](./resources/images/rviz1)

In the *planning* tab of the motion planning pluggin, you can click *update* to give the arm a random valid goal and click *plan and execute*. You should see the robot planning the path and move.... 

<details><summary>Click for  Important Hint</summary>
  
<p>
  
IT'S NOT GOING TO MOVE
  
Have a look at the terminal where you've launched moveit from. You should see an error.... that's right, two links are in collision, in fact, all links are in collision! Something is wrong in our moveit configuration  

</p> 

</details>

### Moveit setup assisstant

**ACTION**
Install moveit-setup-assistant

Utimatly, the setup assistant will help us create a package containing all the moveit configuration files along with launch files to get the robot up and running. 

**ACTION**
Launch the moveit-setup assistant and load the brocken moveit configuration file.

<details><summary>Click for Hint</summary>
  
<p>
  
`roslaunch moveit_setup_assistant setup_assistant.launch`

</p> 

</details>
<br>

Once loaded you should see a model of your robot appear on the right:

![setup_assistant](./resources/images/setup_assistant)

Now take a look at the *Self-Collision* tab (our issue had to do with link collisions). You will notice that there are no collisions defined. Go ahead a generate a collision matrix. 

**ACTION**
Generate a collision matrix.

Two other important tabs in the setup assistant are *Planning Groups* and *End Effectors*. The first one is where we define the joints and links the hand and arm will use for planning. The name are the group are important to know. 

**ACTION**
Inspect how the arm and hand group are formed in the setup assistant.

The *End Effectors* tab is where we difine the end-effector of our robot. This crucial when we want to grasp objects. 

If you want to understand the Moveit setup assistant better, go through this [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html) in your own time.

You can now go to the bottom most tab *Configuration Files*. This is where we generate the moveit pkg and all relevant files. By generating the collision matrix, you would have modified the *.srdf*  file. Before generating the package make sure you select the *.srdf* so that it gets regenerated. All the other boxes can be left as they are.

![set_assistant2](./resources/images/set_assistant2)

**ACTION**
Select `config/husky_ur5.srdf` and click Generate Package.

You can now leave the setup assistant and retry launching `moveit_planning_execution_gazebo.launch`.

You should now be able to plan a path and see the robot move in Gazebo.

## Using Moveit

**A quick example a giving a goal to Moveit in a .cpp file**

```rosrun husky_abb_manipulation  moveit_expl```

**ACTION**
Inpect this file, see what it does and how. You will need this knowledge later.


Obviously we want to use our newly acquired super-tool to do more than move an arm around using Rviz. It is time to create a application for our arm. A common one is to grasp an object which position is determined using sensors. Here we will be using an image and apriltags.

Start by cloning the `apriltag2_ros` pkg in your `src` directory.

Then launch the following two files: `apriltag_spawn` and `tag_detection`.

**ACTION**
Clone `apriltag2_ros` and launch `apriltag_spawn` and `tag_detection`.

<details><summary>Click for Hint</summary>
  
<p>

```
 roslaunch apriltags_gazebo apriltag_spawn.launch
 roslaunch apriltags_gazebo tag_detection.launch 

```  
</p> 

</details>
<br>

Can you detect the tag? Look in Rviz or at the `/tag_detections` topic.

**Applicaiton = Integration**

To create any application in ROS we need to integrate several modules together. From the [perception](https://github.com/ros-workshop/perception) workshop, we now have the pose of our apriltag.

 + Module 1 : Apriltag detection 
    - output: tf from `camera_link` to `tag_link`
    
What we want is to grasp the object with the tag
 
 + Module 3 : Object grasping
    - output: arm and hand trajectory msgs
    - input: pose of the object (`geometry_msgs/Pose`)
 
 We are not quite there! the output of module 1 does not match the input of module 3. We need module 2.
 
 ### Transform listener
 
 Have a look at the node `transform_tag_frame.cpp` located in `husky_abb_manipulation`.
 
 Run this node and see what it does. Modify it so we obtain a `geometry_msgs/Pose` out of it.
 
 **ACTION**
Run `transform_tag_frame`node and modify it to get a `geometry_msgs/Pose`.
 
 <details><summary>Click for Hint</summary>

<p>

Don't worry about the orientation.

The transform should be from the `planning frame` to the tag frame . You can find out the planning frame when running `moveit_expl`

</p> 

</details>
<br>

You should now have the module 2:

 + Module 2 : msg transform
    - output: pose of the object (`geometry_msgs/Pose`)
    - input: tf from `planning frame` to `tag_link`



### The Manipulation Pipeline

We now have all 3 modules required. Make sure that you have module 1 and 2 running. 

<details><summary>Quick Recap</summary>
  
<p>

To have everything up at running you need to have launched the following:

```
roslaunch husky_abb_gazebo husky_abb.launch
roslaunch husky_abb_moveit_config  moveit_planning_execution_gazebo.launch 
roslaunch apriltags_gazebo apriltag_spawn.launch 
roslaunch apriltags_gazebo tag_detection.launch 
rosrun husky_abb_manipulation transform_tag_frame 
```

</p> 

</details>
<br>

Now run `husky_abb_grab_object`. You should see the end-effector move to the front of the object. Modify the code to make the hand grasp of object and drop it on the husky base

**ACTION**
Modify `husky_abb_grab_object.cpp` to make hand grasp the object and that arm drop it on the husky base.

<details><summary>Hint</summary>
  
<p>

Create a MoveGroup for the barrette hand .

Use joint targets to close the hand.

The Object will stick to the hand after a few seonds of contact.

</p> 

</details>
<br>


## Goals Strech

**Goal:** make the arm grasp the object whislt avoiding the environment

Restart the Gazebo simulation and launch `obsatcle_apriltag_spawn`. Grasp the object without hittting obstacles.


<details><summary>Hint</summary>
  
<p>

Moveit will do the obstacle avoidance for you provided an OctoMap

An OctoMap can be created using a depth camera

Start the moveit sensor manager (only a few lines to comment in the right file)

</p> 

</details>
<br>
