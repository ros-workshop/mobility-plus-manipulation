#include <husky_abb_manipulation/husky_abb_grab_object.h>

void SigintHandler(int sig)
{
	// Do some custom action.
	// For example, publish a stop message to some other nodes.

	// All the default sigint handler does is call shutdown()
	ROS_INFO("SHUTING DOWN()");
	ros::shutdown();
}

/* GraspTag Class */

GraspTag::GraspTag()
{

	ros::AsyncSpinner spinner(0);
	spinner.start();
	signal(SIGINT, SigintHandler);
	pose_sub = n.subscribe("/tag_pose", 1000, &GraspTag::setPoseCallback, this);
	_service = n.advertiseService("grasp", &GraspTag::serviceCallback,this); // creating the server called grasp
	ROS_INFO("READY");
	attach_tag_client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
	detach_tag_client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
	DontExecuteGrasp();
	_attach_srv.request.model_name_1 = "/";
	_attach_srv.request.link_name_1 = "abb2_link_6";
	_attach_srv.request.model_name_2 = "apriltag_";
	_attach_srv.request.link_name_2 = "my_box";
	tag_number = 0;
}

void GraspTag::setPoseCallback(const geometry_msgs::Pose::ConstPtr &pose)
{

	target_pose.position = pose->position;
	target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);
	tag_flag = true;
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
		res.message = "failed to exceute all motion";
	}

	return true;
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

geometry_msgs::Pose GraspTag::getTarget(){
	return target_pose;
}

void GraspTag::SetTagFlag(bool flag){
	tag_flag = flag;

}

bool GraspTag::TagIsSeen(){
	return tag_flag;

}

/*********************************
 * ********************************
 * ********************************
 * ********************************
 * ********************************/

MoveHusky::MoveHusky()
{

	ros::AsyncSpinner spinner(0);
	spinner.start();
	_husky_commands = _nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
}

void MoveHusky::stop_husky()
{

	_velocity_commands.linear.x = 0.0;
	_velocity_commands.linear.y = 0.0;
	_velocity_commands.linear.z = 0.0;

	_velocity_commands.angular.x = 0.0;
	_velocity_commands.angular.y = 0.0;
	_velocity_commands.angular.z = 0.0;
	_husky_commands.publish(_velocity_commands);
}

bool MoveHusky::drive_forwards()
{
	ROS_INFO("STARTED DRIVING FORWARDS");
	geometry_msgs::Pose cube_position = _graps_tag_obj.getTarget();
	ros::Rate rate(10);

	stop_husky();
	ros::Time now = ros::Time::now();
	ros::Time then = ros::Time::now();
	while (cube_position.position.x > 0.85 && ((then - now).toSec() < 5)) //while cube is more than .85 meters and loop has run for less than 5 seconds
	{
		cube_position = _graps_tag_obj.getTarget();
		then = ros::Time::now();
		rate.sleep();
		_velocity_commands.linear.x = 0.2;
		_husky_commands.publish(_velocity_commands);
	}

	stop_husky();

	sleep(3.0);
}

bool MoveHusky::drive_backwards()
{
	ROS_INFO("STARTED DRIVING BACKWARDS");
	ros::Rate rate(10);
	ros::Time now = ros::Time::now();
	ros::Time then = ros::Time::now();

	while ((then - now).toSec() < 10.0)
	{
		then = ros::Time::now();

		_velocity_commands.linear.x = -0.1;
		_velocity_commands.linear.y = 0.0;
		_velocity_commands.linear.z = 0.0;

		_velocity_commands.angular.x = 0.0;
		_velocity_commands.angular.y = 0.0;
		_velocity_commands.angular.z = 0.0;
		_husky_commands.publish(_velocity_commands);
		rate.sleep();
	}

	stop_husky();
}

/************************************************/
/************************************************/
/************************************************/
/************************************************/
/************************************************/

CommandAbbBarret::CommandAbbBarret() : arm("abb_arm"), hand("barrett_hand")
{

	ros::AsyncSpinner spinner(0);
	spinner.start();
	arm.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	arm.setGoalTolerance(0.01);
	arm.setPlanningTime(2.0);

	hand.setPlannerId("RRTConnectkConfigDefault");
	//can be modified as desired
	hand.setGoalJointTolerance(0.1);
	hand.setPlanningTime(2.0);

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());
}

bool CommandAbbBarret::arm_to_cube(geometry_msgs::Pose pose)
{

	ROS_INFO("Tag location:\n x:%lf \n c:%lf \n z:%lf", pose.position.x, pose.position.y, pose.position.z);

	pose.position.z += 0.25; //offset by .25 meters in the z axis
	// pose.position.x -= 0.05;
	pose.position.y -= 0.05;

	pose.orientation.w = 0.707;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.707;
	pose.orientation.z = 0.0; // and pointed straight down

	arm.setPoseTarget(pose, arm.getEndEffectorLink().c_str());

	arm.plan(_my_plan); // check if plan succeded

	success = arm.move();

	sleep(1.0);
	if (success.val==1){
	pose.position.z -= 0.07; // move down towards the cube
	arm.setPoseTarget(pose, arm.getEndEffectorLink().c_str());
	arm.plan(_my_plan); // check if plan succeded
	success = arm.move();
	}

	return success.val;
}

bool CommandAbbBarret::close_hand()
{
	hand.setJointValueTarget("finger_2_med_joint", 1.1); //closing hand
	hand.setJointValueTarget("finger_1_med_joint", 1.1);
	hand.setJointValueTarget("finger_3_med_joint", 1.1);
	hand.plan(_my_plan);
	hand.move();

	//sleep(2.0);
	sleep(1.0);
}

bool CommandAbbBarret::lift_object()
{
	geometry_msgs::Pose pose = arm.getCurrentPose(arm.getEndEffectorLink().c_str()).pose;
	pose.position.z += 0.2; //raising cube in the air
	arm.setPoseTarget(pose, arm.getEndEffectorLink().c_str());
	arm.plan(_my_plan); // check if plan succeded
	arm.move();
}

bool CommandAbbBarret::retract_arm()
{
	arm.setJointValueTarget("abb2_joint_1", -2.88); //moving hand to the back of the husky
	arm.setJointValueTarget("abb2_joint_2", 0.54);
	arm.setJointValueTarget("abb2_joint_3", -0.51);
	arm.setJointValueTarget("abb2_joint_4", -0.17);
	arm.setJointValueTarget("abb2_joint_5", 1.34);
	arm.setJointValueTarget("abb2_joint_6", -6.70);
	arm.plan(_my_plan);
	arm.move();
}

bool CommandAbbBarret::open_hand()
{
	hand.setJointValueTarget("finger_2_med_joint", 0); //closing hand
	hand.setJointValueTarget("finger_1_med_joint", 0);
	hand.setJointValueTarget("finger_3_med_joint", 0);
	hand.plan(_my_plan);
	hand.move();
}

bool CommandAbbBarret::arm_home()
{
	ROS_INFO("Moving to Home");
	arm.setNamedTarget("home"); // This is needed so that the robot arm will not block the LIDAR
	arm.plan(_my_plan);
	arm.move();
	return 1;
}

/**********************************************************/

// int main(int argc, char **argv)
// {
// 	//delcarations

// 	signal(SIGINT, SigintHandler);

// 	ros::init(argc, argv, "object_grasp_server");
// 	ros::NodeHandle node_handle;
// 	ros::AsyncSpinner spinner(2);
// 	spinner.start();
// 	ros::Rate loop_rate(1);
// 	ros::NodeHandle n;
// 	ros::Duration grasp_timeout(5.0)
// 		geometry_msgs::Pose temp_pose; //temporary pose to check when the same target is receive
// 	GraspTag grasObj;				   // instance of the class GraspTag

// 	bool move_before_grasp;
// 	if (!(n.getParam("/move_before_grasp", move_before_grasp)))
// 	{
// 		ROS_ERROR("param 'move_before_grasp' not found");
// 		ros::shutdown();
// 	}

// 	moveit::planning_interface::MoveGroupInterface::Plan my_plan; // plan containing the trajectory
// 	static const std::string PLANNING_GROUP = "abb_arm";		  // planning group
// 	static const std::string PLANNING_GROUP2 = "barrett_hand";
// 	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // planning interface
// 	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);			 // planning group
// 	moveit::planning_interface::MoveGroupInterface hand(PLANNING_GROUP2);		 // planning group
// 	arm.setPlannerId("RRTConnectkConfigDefault");
// 	//can be modified as desired
// 	arm.setGoalTolerance(0.01);
// 	arm.setPlanningTime(2.0);
// 	//hand.setPlannerId("LBKPIECEkConfigDefault");
// 	hand.setPlannerId("RRTConnectkConfigDefault");
// 	//can be modified as desired
// 	hand.setGoalJointTolerance(0.1);
// 	hand.setPlanningTime(2.0);
// 	//arm.setPlanningTime(10.0);
// 	//arm.setPlanningTime(15.0);

// 	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
// 	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

// 	temp_pose = (arm.getCurrentPose(arm.getEndEffectorLink().c_str())).pose;

// 	ROS_INFO("Pose:HOME");
// 	arm.setNamedTarget("home");
// 	arm.plan(my_plan); // check if plan succeded
// 	arm.move();

// 	hand.setJointValueTarget("finger_2_med_joint", 0);
// 	hand.setJointValueTarget("finger_1_med_joint", 0);
// 	hand.setJointValueTarget("finger_3_med_joint", 0);
// 	hand.plan(my_plan);
// 	hand.move();
// 	moveit::planning_interface::MoveItErrorCode success;

// 	sleep(4.0);
// 	command_vel HUSKY;
// 	if (move_before_grasp)
// 	{

// 		HUSKY.start_publisher();
// 	}
// 	ros::ServiceServer service = n.advertiseService("grasp", &GraspTag::serviceCallback, &grasObj); // creating the server called grasp
// 	ROS_INFO("READY");
// 	while (ros::ok) // keep on running until stoped
// 	{
// 		grasObj.DontExecuteGrasp();
// 		ROS_INFO("tag detected");

// 		ROS_INFO("Ready for Grasp service call");
// 		while (!grasObj.ExecuteGrasp() && !ros::isShuttingDown())
// 		{
// 			//wait
// 			loop_rate.sleep();
// 		}
// 		if (move_before_grasp)
// 			HUSKY.drive_forwards();
// 		sleep(5.0);
// 		ROS_INFO("Grasp server active");

// 		ros::topic::waitForMessage<geometry_msgs::Pose>("/tag_pose", grasp_timeout);

// 		temp_pose = grasObj.getTarget(); //set the desired pose to the position of the cube

// 		ROS_INFO("Tag location:\n x:%lf \n c:%lf \n z:%lf", temp_pose.position.x, temp_pose.position.y, temp_pose.position.z);

// 		temp_pose.position.z += 0.25; //offset by .25 meters in the z axis
// 		temp_pose.position.x -= 0.05;
// 		temp_pose.position.y += 0.05;

// 		temp_pose.orientation.w = 0.707;
// 		temp_pose.orientation.x = 0.0;
// 		temp_pose.orientation.y = 0.707;
// 		temp_pose.orientation.z = 0.0; // and pointed straight down

// 		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());

// 		arm.plan(my_plan); // check if plan succeded

// 		arm.move();

// 		//sleep(5.0);
// 		sleep(1.0);

// 		temp_pose.position.z -= 0.08; // move down towards the cube
// 		arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());

// 		success = arm.plan(my_plan); // check if plan succeded

// 		arm.move();
// 		sleep(1.0);
// 		ROS_INFO("Closing Gripper");
// 		if (success.val == 1)
// 		{
// 			hand.setJointValueTarget("finger_2_med_joint", 1.1); //closing hand
// 			hand.setJointValueTarget("finger_1_med_joint", 1.1);
// 			hand.setJointValueTarget("finger_3_med_joint", 1.1);
// 			hand.plan(my_plan);
// 			hand.move();

// 			//sleep(2.0);
// 			sleep(1.0);

// 			grasObj.AttachGripper(); //attaching object
// 			ROS_INFO("Attaching objects");

// 			temp_pose.position.z += 0.2; //raising cube in the air
// 			arm.setPoseTarget(temp_pose, arm.getEndEffectorLink().c_str());
// 			arm.plan(my_plan); // check if plan succeded
// 			arm.move();

// 			// moving object to next stand

// 			arm.setJointValueTarget("abb2_joint_1", -2.88); //moving hand to the back of the husky
// 			arm.setJointValueTarget("abb2_joint_2", 0.54);
// 			arm.setJointValueTarget("abb2_joint_3", -0.51);
// 			arm.setJointValueTarget("abb2_joint_4", -0.17);
// 			arm.setJointValueTarget("abb2_joint_5", 1.34);
// 			arm.setJointValueTarget("abb2_joint_6", -6.70);
// 			arm.plan(my_plan);
// 			arm.move();

// 			sleep(7.0);
// 			ROS_INFO("Detaching Object");
// 		}
// 		grasObj.DetachGripper();						   // Detaching object
// 		hand.setJointValueTarget("finger_2_med_joint", 0); //closing hand
// 		hand.setJointValueTarget("finger_1_med_joint", 0);
// 		hand.setJointValueTarget("finger_3_med_joint", 0);
// 		hand.plan(my_plan);
// 		hand.move();

// 		sleep(1);
// 		ROS_INFO("Moving to Home");
// 		arm.setNamedTarget("home"); // This is needed so that the robot arm will not block the LIDAR
// 		arm.plan(my_plan);
// 		arm.move();

// 		ROS_INFO("finished motion plan");
// 		if (move_before_grasp)
// 			HUSKY.drive_backwards();
// 		sleep(3.0);

// 		grasObj.setSuccess(true);

// 		ros::spinOnce();

// 		loop_rate.sleep();
// 	}
// 	return 0;
// }