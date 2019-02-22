#!/usr/bin/env python

import roslib
import rospy
import rospkg
import actionlib
import csv
import time
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class MoveBaseSquare():
    def __init__(self, locations):
        rospy.init_node('nav_test', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        # Create a list to hold the waypoint poses
        waypoints = list()
        
        q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
        q = Quaternion(*q_angle)
        
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        for location in locations:
            waypoints.append(Pose(Point(location[0], location[1], 0.0), q))

        
        # Initialize the visualization markers for RViz
        self.init_markers()
        
        # Set a visualization marker at each waypoint        
        for waypoint in waypoints:           
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)
            
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        # Initialize a counter to track waypoints
        i = 0
        
        # Cycle through the four waypoints
        while i < len(locations) and not rospy.is_shutdown():
            rospy.loginfo("Starting goal [%s/%s] going to (%s, %s)" % (i+1, len(locations), locations[i][0], locations[i][1]))

            # Update the marker display
            self.marker_pub.publish(self.markers)
            
            # Intialize the waypoint goal
            goal = MoveBaseGoal()
            
            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'map'
            
            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Set the goal pose to the i-th waypoint
            goal.target_pose.pose = waypoints[i]
            
            # Start the robot moving toward the goal
            succeeded = self.move(goal)
            if succeeded :
                self.grab()

            i += 1
        
    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)
            
            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(90)) 
            
            succeeded = False
            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    succeeded = True
            return succeeded

    def grab(self):
        rospy.loginfo("Searching for cube")
        time.sleep(5)
        rospy.loginfo("Got cube!")
                    
    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.SPHERE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospack = rospkg.RosPack()
        path = rospack.get_path('mobility')+'/locations.csv'
        print("Searching for locations in: " + path)
        locations = []
        with open(path, 'rb') as csvfile:
            point_reader = csv.reader(csvfile, delimiter=',')
            for row in point_reader:
                print "Location loaded: " + ', '.join(row)
                locations.append([float(row[0]) - 1, float(row[1])])

        MoveBaseSquare(locations)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")