#ifndef HUSKY_ABB_GRAB_OBJECT 
#define HUSKY_ABB_GRAB_OBJECT


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

//using namespace tf;

void SigintHandler(int sig);


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
	void SetTagFlag(bool flag);
	bool TagIsSeen();
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
	ros::ServiceServer _service;
	bool tag_flag;
};

class MoveHusky
{
public:
	MoveHusky();
	void stop_husky();
	bool drive_forwards();
	bool drive_backwards();
	

private:
	ros::NodeHandle _nh;
	ros::Publisher _husky_commands;
	float _x;
	GraspTag _graps_tag_obj;
	geometry_msgs::Twist _velocity_commands;
};


class CommandAbbBarret
{
	public:
	CommandAbbBarret();
	bool arm_to_cube(geometry_msgs::Pose pose);
	bool close_hand();
	bool open_hand();
	bool retract_arm();
	bool arm_home();
	bool lift_object();

	private:

	moveit::planning_interface::MoveGroupInterface::Plan _my_plan; // plan containing the trajectory
	moveit::planning_interface::PlanningSceneInterface _planning_scene_interface; // planning interface
	moveit::planning_interface::MoveGroupInterface arm;			 // planning group
	moveit::planning_interface::MoveGroupInterface hand;		 // planning group


};

#endif 