#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <apriltag_ros/AprilTagDetectionArray.h>



//subscribes to the "tag_detections" topic to look up which tag is visible. Determines which "tag->baselink" transform to pass to lookupTransform.
class subscriber {
   public: 
    void start_publisher() {
        pub = nh.advertise<geometry_msgs::Pose>("tag_pose", 1000);
    }

    void publish(geometry_msgs::Pose msg_) {
        pub.publish(msg_);
    }
    void start_subscriber() {
      sub = nh.subscribe("/tag_detections", 1000, &subscriber::Callback, this);
    }

    void Callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& DATA) {

        if(DATA->detections.empty()){
            x=1;//if there are no detections, then search for tag_1
        }else
        {
            x=DATA->detections.at(0).id.at(0);
        }
        
         
    } 

    bool which_tag_available(){ 
      return (bool)x;
    }
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    apriltag_ros::AprilTagDetectionArray DATA;
    int x=1;
	};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_tag_frame");
	subscriber sub;
    geometry_msgs::Pose msg;
	geometry_msgs::TransformStamped location_msg;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    
    sub.start_subscriber();
    sub.start_publisher();

    while (ros::ok())
    {
        try
        {
            if(sub.which_tag_available()){
                listener.lookupTransform("/base_link", "/tag_1", ros::Time(0), transform);
            }
            else
            {
                listener.lookupTransform("/base_link", "/tag_0", ros::Time(0), transform);
            }
            
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        

        transformStampedTFToMsg(transform,location_msg);
                
        msg.position.x=location_msg.transform.translation.x;
        msg.position.y=location_msg.transform.translation.y;
        msg.position.z=location_msg.transform.translation.z;
        msg.orientation.x=location_msg.transform.rotation.x;
        msg.orientation.y=location_msg.transform.rotation.y;
        msg.orientation.z=location_msg.transform.rotation.z;
        msg.orientation.w=location_msg.transform.rotation.w;
        
		sub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}