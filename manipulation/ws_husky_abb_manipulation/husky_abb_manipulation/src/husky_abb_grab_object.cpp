#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <signal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <string>
// start of the GraspTag class//

using namespace tf;

void SigintHandler(int sig)
{

	ROS_INFO("SHUTING DOWN()");
	ros::shutdown();
}

class GraspTag
{

public:
	
	GraspTag();
	
	void setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose);
	
	bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
	
	void DontExecuteGrasp();
	
	void setSuccess(bool success);
	
	bool ExecuteGrasp();
	
	void AttachGripper();
	
	void DetachGripper();
	
	geometry_msgs::Pose getTarget();

private:
	
	geometry_msgs::Pose target_pose;
	
	ros::NodeHandle n;
	
	ros::Subscriber pose_sub;
	
	bool _execute_grasp;
	
	bool _succeeded;
	
	bool _srv_success;
	
	trajectory_msgs::JointTrajectory _gripper_angle_traj;
	
	trajectory_msgs::JointTrajectoryPoint _gripper_angle_point;
	
	gazebo_ros_link_attacher::Attach _attach_srv;
	
	ros::ServiceClient attach_tag_client;
	
	ros::ServiceClient detach_tag_client;
	
	int tag_number;
};

GraspTag::GraspTag()
{

	pose_sub = n.subscribe("/tag_pose", 1000, &GraspTag::setPoseCallback, this);

	attach_tag_client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

	detach_tag_client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

	DontExecuteGrasp();

	_attach_srv.request.model_name_1 = "/";

	_attach_srv.request.link_name_1 = "abb2_link_6";

	_attach_srv.request.model_name_2 = "apriltag_";

	_attach_srv.request.link_name_2 = "my_box";

	tag_number = 0;
}

bool GraspTag::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{

	ROS_INFO("service called");

	_execute_grasp = true;

	_succeeded = false;

	while (_succeeded == false && !ros::isShuttingDown())
	{
		sleep(0.1);
	}

	if (_srv_success)
	{
		res.success = _srv_success;
		
		res.message = "all motion succeeded";
	}

	else
	{
		res.success = _srv_success;
		
		res.message = "falied to exceute all motion";
	}

	return true;
}

void GraspTag::setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose)
{
	target_pose.position = pose->position;
	
	target_pose.orientation = createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);
}

geometry_msgs::Pose GraspTag::getTarget()
{
	return (target_pose);
}

void GraspTag::DontExecuteGrasp()
{
	_execute_grasp = false;
}

void GraspTag::setSuccess(bool success)
{
	_srv_success = success;
	
	_succeeded = true;
}

bool GraspTag::ExecuteGrasp()
{
	return _execute_grasp;
}

void GraspTag::AttachGripper()
{
	_attach_srv.request.model_name_2 = "apriltag_" + std::to_string(tag_number);
	
	attach_tag_client.call(_attach_srv);
}

void GraspTag::DetachGripper()
{
	detach_tag_client.call(_attach_srv);
	
	tag_number++;
}

class command_vel
{
public:
	void start_publisher()
	{
		husky_commands = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
	}

	void stop_husky()
	{
		velocity_commands.linear.x = 0.0;
		
		velocity_commands.linear.y = 0.0;
		
		velocity_commands.linear.z = 0.0;

		velocity_commands.angular.x = 0.0;
		
		velocity_commands.angular.y = 0.0;
		
		velocity_commands.angular.z = 0.0;
		
		husky_commands.publish(velocity_commands);
	}

	bool drive_forwards()
	{
		ROS_INFO("STARTED DRIVING FORWARDS");
		
		geometry_msgs::Pose cube_position = get_pose.getTarget();
		
		ros::Rate rate(10);

		stop_husky();
		
		ros::Time now = ros::Time::now();
		
		ros::Time then = ros::Time::now();
		
		while ((then - now).toSec() < 5)
		{
			cube_position = get_pose.getTarget();
			
			then = ros::Time::now();
			
			rate.sleep();
			
			velocity_commands.linear.x = 0.2;
			
			husky_commands.publish(velocity_commands);
		}

		stop_husky();

		sleep(5.0);
	}

	bool drive_backwards()
	{
		ROS_INFO("STARTED DRIVING BACKWARDS");
		
		ros::Rate rate(10);
		
		ros::Time now = ros::Time::now();
		
		ros::Time then = ros::Time::now();

		while ((then - now).toSec() < 10)
		{
			then = ros::Time::now();

			velocity_commands.linear.x = -0.1;
			
			velocity_commands.linear.y = 0.0;
			
			velocity_commands.linear.z = 0.0;

			velocity_commands.angular.x = 0.0;
			
			velocity_commands.angular.y = 0.0;
			
			velocity_commands.angular.z = 0.0;
			
			husky_commands.publish(velocity_commands);
			
			rate.sleep();
		}

		velocity_commands.linear.x = 0.0;
		
		velocity_commands.linear.y = 0.0;
		
		velocity_commands.linear.z = 0.0;

		velocity_commands.angular.x = 0.0;
		
		velocity_commands.angular.y = 0.0;
		
		velocity_commands.angular.z = 0.0;
		
		husky_commands.publish(velocity_commands);
	}

private:
	
	ros::NodeHandle nh;
	
	ros::Publisher husky_commands;
	
	float x;
	
	GraspTag get_pose;
	
	geometry_msgs::Twist velocity_commands;
};

int main(int argc, char **argv)
{
	//delcarations

	signal(SIGINT, SigintHandler);

	ros::init(argc, argv, "object_grasp_server");
	
	ros::NodeHandle node_handle;
	
	ros::AsyncSpinner spinner(2);
	
	spinner.start();

	ros::Rate loop_rate(1);
	
	ros::NodeHandle n;
	
	geometry_msgs::Pose temp_pose; //temporary pose to check when the same target is receive
	
	GraspTag grasObj;			   // instance of the class GraspTag

	bool move_before_grasp;
	
	if (!(n.getParam("/move_before_grasp", move_before_grasp)))
	{
		ROS_ERROR("param 'move_before_grasp' not found");
		
		ros::shutdown();
	}

	moveit::planning_interface::MoveGroupInterface::Plan my_plan; // plan containing the trajectory
	
	static const std::string PLANNING_GROUP = "abb_arm";		  // planning group
	
	static const std::string PLANNING_GROUP2 = "barrett_hand";
	
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // planning interface
	
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);			 // planning group
	
	moveit::planning_interface::MoveGroupInterface hand(PLANNING_GROUP2);		 // planning group
	
	arm.setPlannerId("RRTConnectkConfigDefault");

	arm.setGoalTolerance(0.01);
	
	arm.setPlanningTime(2.0);

	hand.setPlannerId("RRTConnectkConfigDefault");

	hand.setGoalJointTolerance(0.1);
	
	hand.setPlanningTime(2.0);

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;

	ROS_INFO("Pose:HOME");
	
	arm.setNamedTarget("home");
	
	arm.plan(my_plan); // check if plan succeded
	
	arm.move();

	hand.setJointValueTarget("finger_2_med_joint", 0);
	
	hand.setJointValueTarget("finger_1_med_joint", 0);
	
	hand.setJointValueTarget("finger_3_med_joint", 0);
	
	hand.plan(my_plan);
	
	hand.move();
	
	moveit::planning_interface::MoveItErrorCode success;

	sleep(4.0);
	
	command_vel HUSKY;
	
	if (move_before_grasp)
	{
		HUSKY.start_publisher();
	}
	
	ros::ServiceServer service = n.advertiseService("grasp", &GraspTag::serviceCallback, &grasObj); // creating the server called grasp
	
	ROS_INFO("READY");

	while (!ros::isShuttingDown()) // keep on running until stoped
	{
		
		grasObj.DontExecuteGrasp();
		
		ros::topic::waitForMessage<geometry_msgs::Pose>("/tag_pose");
		
		ROS_INFO("tag detected");

		ROS_INFO("Ready for Grasp service call");
		
		while (!grasObj.ExecuteGrasp() && !ros::isShuttingDown())
		{
			//wait
			loop_rate.sleep();
		}
		if (move_before_grasp)
		
			HUSKY.drive_forwards();
		
		sleep(5.0);
		
		ROS_INFO("Grasp server active");
		
		temp_pose = grasObj.getTarget(); //set the desired pose to the position of the cube

		ROS_INFO("Tag location:\n x:%lf \n c:%lf \n z:%lf", temp_pose.position.x, temp_pose.position.y, temp_pose.position.z);

		temp_pose.position.z += 0.25; //offset by .25 meters in the z axis
		
		temp_pose.position.x -= 0.05;
		
		temp_pose.position.y += 0.05;

		temp_pose.orientation.w = 0.707;
		
		temp_pose.orientation.x = 0.0;
		
		temp_pose.orientation.y = 0.707;
		
		temp_pose.orientation.z = 0.0; // and pointed straight down

		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());

		arm.plan(my_plan); // check if plan succeded

		arm.move();

		//sleep(5.0);
		sleep(1.0);

		temp_pose.position.z -= 0.08; // move down towards the cube
		
		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());

		success = arm.plan(my_plan); // check if plan succeded

		arm.move();
		
		sleep(1.0);
		
		ROS_INFO("Closing Gripper");
		
		if (success.val == 1)
		
		{
			hand.setJointValueTarget("finger_2_med_joint", 1.1); //closing hand
		
			hand.setJointValueTarget("finger_1_med_joint", 1.1);
		
			hand.setJointValueTarget("finger_3_med_joint", 1.1);
		
			hand.plan(my_plan);
		
			hand.move();

			//sleep(2.0);
			sleep(1.0);

			grasObj.AttachGripper(); //attaching object
		
			ROS_INFO("Attaching objects");

			temp_pose.position.z += 0.2; //raising cube in the air
		
			arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
		
			arm.plan(my_plan); // check if plan succeded
		
			arm.move();

			// moving object to next stand

			arm.setJointValueTarget("abb2_joint_1", -2.88); //moving hand to the back of the husky
		
			arm.setJointValueTarget("abb2_joint_2", 0.54);
		
			arm.setJointValueTarget("abb2_joint_3", -0.51);
		
			arm.setJointValueTarget("abb2_joint_4", -0.17);
		
			arm.setJointValueTarget("abb2_joint_5", 1.34);
		
			arm.setJointValueTarget("abb2_joint_6", -6.70);
		
			arm.plan(my_plan);
		
			arm.move();

			sleep(7.0);
		
			ROS_INFO("Detaching Object");
		}
		grasObj.DetachGripper();						   // Detaching object
		
		hand.setJointValueTarget("finger_2_med_joint", 0); //closing hand
		
		hand.setJointValueTarget("finger_1_med_joint", 0);
		
		hand.setJointValueTarget("finger_3_med_joint", 0);
		
		hand.plan(my_plan);
		
		hand.move();

		sleep(1);
		
		ROS_INFO("Moving to Home");
		
		arm.setNamedTarget("home"); // This is needed so that the robot arm will not block the LIDAR
		
		arm.plan(my_plan);
		
		arm.move();

		ROS_INFO("finished motion plan");
		
		if (move_before_grasp)
			HUSKY.drive_backwards();
		
		sleep(3.0);

		grasObj.setSuccess(true);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}