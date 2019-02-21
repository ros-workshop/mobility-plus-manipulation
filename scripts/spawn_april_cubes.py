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
        csv_path = rospack.get_path('mobility')+'/locations.csv'
        print("Opening CSV file " + csv_path)
        locations = []
        with open(csv_path, 'rb') as csvfile:
            point_reader = csv.reader(csvfile, delimiter=',')
            for row in point_reader:
                locations.append([float(row[0]), float(row[1])])
                print("Location %d: "%len(locations) + ', '.join(row))
        
        
        april_cube_path = rospack.get_path('apriltags_gazebo')+'/models/apriltag/model.sdf'

        cmds = []
        for i in range(len(locations)):
            x,y = locations[i]

            april_cube_model = "april_cube_%d" % i 
            cmds.append("rosservice call gazebo/delete_model '{model_name: %s}'" % april_cube_model )
            cmds.append("rosrun gazebo_ros spawn_model -sdf " + 
                        "-file '%s' " % april_cube_path +  
                        "-model %s "  % april_cube_model + 
                        "-x %f -y %f -z 0.49 " % ( x, y ))

            #hydrant_model = "hydrant_%d" % i 
            #cmds.append("rosservice call gazebo/delete_model '{model_name: %s}'" % hydrant_model)
            #cmds.append("rosrun gazebo_ros spawn_model -sdf " + 
                        #"-database fire_hydrant "  +  
                        #"-model %s " % hydrant_model +
                        #"-x %f -y %f -z -0.3 " % ( x, y ))

        for cmd in cmds:
            print cmd      
            os.system( cmd ) 


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
