#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.pose = None
        self.waypoints = None
        self.traffic_wp_idx = None
        self.obstacle_wp_idx = None
        self.waypoint_pos = None
        self.waypoint_tree = None

        # Main execution loop
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.waypoints:
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        vehicle_pos = (self.pose.pose.position.x, self.pose.pose.position.y)
        idx = self.waypoint_tree.query(vehicle_pos, 1)[1]

        # Check if the index is in front or on the back of the current position
        closest_pos = np.array(self.waypoint_pos[idx])
        prev_pos = np.array(self.waypoint_pos[idx - 1])
        vehicle_pos = np.array(vehicle_pos)

        direction = np.dot(closest_pos - prev_pos, vehicle_pos - closest_pos)

        if direction > 0:
            idx += 1
            idx %= len(self.waypoint_pos)

        return idx

    def publish_waypoints(self, idx):
        lane = Lane()
        lane.header = self.waypoints.header
        lane.waypoints = self.waypoints.waypoints[idx:idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoint_pos:
            self.waypoint_pos = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in waypoints]
            self.waypoint_tree = KDTree(self.waypoint_pos)

    def traffic_cb(self, msg):
        self.traffic_wp_idx = msg.data
        pass

    def obstacle_cb(self, msg):
        self.obstacle_wp_idx = msg.data
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
