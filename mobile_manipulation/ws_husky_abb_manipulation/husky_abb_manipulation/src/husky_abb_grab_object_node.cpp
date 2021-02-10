#include <husky_abb_manipulation/husky_abb_grab_object.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "object_grasp_server");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	ros::Duration grasp_timeout(10);
	bool move_before_grasp;
	if (!(n.getParam("/move_before_grasp", move_before_grasp)))
	{
		ROS_ERROR("param 'move_before_grasp' not found");
		ros::shutdown();
	}

	GraspTag grasp_tag_obj;
	MoveHusky move_husky_obj;
	CommandAbbBarret command_abb_barret_obj;

	command_abb_barret_obj.arm_home();
	command_abb_barret_obj.open_hand();

	while (ros::ok())
	{

		grasp_tag_obj.DontExecuteGrasp();

		while (!grasp_tag_obj.ExecuteGrasp() && !ros::isShuttingDown())
		{
			//wait
			loop_rate.sleep();
		}

		if (move_before_grasp)
			move_husky_obj.drive_forwards();
		grasp_tag_obj.SetTagFlag(false);

		ros::topic::waitForMessage<geometry_msgs::Pose>("/tag_pose", grasp_timeout);

		if (grasp_tag_obj.TagIsSeen())
		{
			command_abb_barret_obj.arm_to_cube(grasp_tag_obj.getTarget());
			command_abb_barret_obj.close_hand();
			grasp_tag_obj.AttachGripper();
			command_abb_barret_obj.lift_object();
			command_abb_barret_obj.retract_arm();
			command_abb_barret_obj.open_hand();
			grasp_tag_obj.DetachGripper();
		}

		command_abb_barret_obj.arm_home();
		move_husky_obj.drive_backwards();
		grasp_tag_obj.setSuccess(true);
	}
}

