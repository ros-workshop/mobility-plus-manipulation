#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>




int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_tag_location");
	geometry_msgs::Pose msg;
	geometry_msgs::TransformStamped tf_msg;
    ros::NodeHandle node;
    ros::Rate rate(10);
	ros::Publisher pub = node.advertise<geometry_msgs::Pose>("tag_pose", 1000);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (ros::ok())
    {
        try
        {
            listener.waitForTransform("/base_link", "/tag_0", ros::Time(0), ros::Duration(9.0));
            listener.lookupTransform("/base_link", "/tag_0", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        transformStampedTFToMsg(transform,tf_msg);
		
        msg.position.x=tf_msg.transform.translation.x;
        msg.position.y=tf_msg.transform.translation.y;
        msg.position.z=tf_msg.transform.translation.z;
        msg.orientation.x=tf_msg.transform.rotation.x;
        msg.orientation.y=tf_msg.transform.rotation.y;
        msg.orientation.z=tf_msg.transform.rotation.z;
        msg.orientation.w=tf_msg.transform.rotation.w;
        rate.sleep();
		pub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}
