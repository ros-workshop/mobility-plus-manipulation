#!/usr/bin/env python

import rospy
import rospkg
import os
import csv

# calls spawn_model to instantiate objects in Gazebo
# https://github.com/ros-simulation/gazebo_ros_pkgs/blob/indigo-devel/gazebo_ros/scripts/spawn_model 

if __name__ == '__main__':
    try:
        rospack = rospkg.RosPack()
        csv_path = rospack.get_path('mobility-plus-manipulation')+'/locations.csv'
        print("Opening CSV file " + csv_path)
        locations = []
        with open(csv_path, 'rb') as csvfile:
            point_reader = csv.reader(csvfile, delimiter=',')
            for row in point_reader:
                locations.append([float(row[0]), float(row[1])])
                print("Location %d: "%len(locations) + ', '.join(row))

        cmds = []
        for i in range(len(locations)):
            x,y = locations[i]

            cmds.append("rosrun gazebo_ros spawn_model -sdf " + 
                        "-file `rospack find apriltags_gazebo`/models/STAND/model.sdf "  +  
                        "-model stand_model_%d " % i +
                        "-x %f -y %f -z 0.2 " % ( x, y ))
 
            cmds.append("rosrun gazebo_ros spawn_model -urdf " + 
                        "-file `rospack find apriltags_gazebo`/models/new_apriltag/model.urdf "  +  
                        "-model apriltag_%d " % i +
                        "-x %f -y %f -z 0.51 " % ( x, y ))

        for cmd in cmds:
            print cmd      
            os.system( cmd )



    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
