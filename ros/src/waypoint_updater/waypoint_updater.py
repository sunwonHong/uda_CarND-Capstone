#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool, Int32, Float32
from scipy.spatial import KDTree

import math

LOOKAHEAD_WPS = 50

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.present_vel = None
        self.sline_idx = -1
        self.gostop = 2
        self.change = True
        self.send_wp = None
        self.base_waypoints = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.pre_vel_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.highst_vel = rospy.get_param('/waypoint_loader/velocity') / 3.6

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                self.publish_waypoints()
            rate.sleep()
    
    def get_closest_waypoint_index(self):
        wp = self.base_waypoints.waypoints

        idx = -1
        dis_min = 100000000
        for i in range(len(wp)):
            dis = math.sqrt((self.pose.x - wp[i].pose.pose.position.x)**2 + (self.pose.y - wp[i].pose.pose.position.y)**2)
            if dis < dis_min:
                dis_min = dis
                idx = i
        
        if self.pose.x - self.base_waypoints.waypoints[idx].pose.pose.position.x > 0:
            idx = (idx + 1) % len(self.base_waypoints.waypoints)
            
        return idx

    def vel_state(self, wp_idx, inde_flag):
        lane = Lane()

        if inde_flag == 1:
            vel_final = self.present_vel
            for i in range(LOOKAHEAD_WPS):
                if vel_final < self.highst_vel:
                    vel_final = math.sqrt(self.present_vel + 1 * (math.sqrt((self.pose.x - self.base_waypoints.waypoints[(wp_idx + i) % len(self.base_waypoints.waypoints)].pose.pose.position.x)**2 + (self.pose.y - self.base_waypoints.waypoints[(wp_idx + i) % len(self.base_waypoints.waypoints)].pose.pose.position.y)**2)))
                    wp_temp = self.base_waypoints.waypoints[(wp_idx + i) % len(self.base_waypoints.waypoints)]
                    wp_temp.twist.twist.linear.x = vel_final
                    lane.waypoints.append(wp_temp)

        elif inde_flag == 2:
            vel_final = self.present_vel
            for i in range(LOOKAHEAD_WPS):
                if vel_final > 0.0:
                    vel_final = self.present_vel - (self.present_vel ** 2.0 / (2.0 * (math.sqrt((self.pose.x - self.base_waypoints.waypoints[self.sline_idx].pose.pose.position.x) ** 2 + (self.pose.y - self.base_waypoints.waypoints[self.sline_idx].pose.pose.position.y) ** 2) - 0.5))) / 10
                else:
                    vel_final = 0

                wp_temp = self.base_waypoints.waypoints[(wp_idx + i) % len(self.base_waypoints.waypoints)]
                wp_temp.twist.twist.linear.x = vel_final
                lane.waypoints.append(wp_temp)

        elif inde_flag == 3:
            temp = -5000
            for i in range(0, len(self.send_wp)):
                if self.send_wp[i].pose.pose.position == self.base_waypoints.waypoints[wp_idx].pose.pose.position:
                    temp = i
            if temp == -5000:
                temp = len(self.send_wp)

            for i in range(temp, len(self.send_wp)):
                wp_temp = self.base_waypoints.waypoints[(wp_idx + i - temp) % len(self.base_waypoints.waypoints)]
                wp_temp.twist.twist.linear.x = self.send_wp[i].twist.twist.linear.x
                lane.waypoints.append(wp_temp)

            for i in range(len(lane.waypoints), LOOKAHEAD_WPS):
                wp_temp = self.base_waypoints.waypoints[(wp_idx + i) % len(self.base_waypoints.waypoints)]
                wp_temp.twist.twist.linear.x = self.highst_vel
                lane.waypoints.append(wp_temp)

        elif inde_flag == 4:
            temp = -5000
            for i in range(0, len(self.send_wp)):
                if self.send_wp[i].pose.pose.position == self.base_waypoints.waypoints[wp_idx].pose.pose.position:
                    temp = i
            if temp == -5000:
                temp = len(self.send_wp)

            for i in range(temp, len(self.send_wp)):
                wp_temp = self.base_waypoints.waypoints[(wp_idx + i - temp) % len(self.base_waypoints.waypoints)]
                wp_temp.twist.twist.linear.x = self.send_wp[i].twist.twist.linear.x
                lane.waypoints.append(wp_temp)

            for i in range(len(lane.waypoints), LOOKAHEAD_WPS):
                wp_temp = self.base_waypoints.waypoints[(wp_idx + i) % len(self.base_waypoints.waypoints)]
                wp_temp.twist.twist.linear.x = 0
                lane.waypoints.append(wp_temp)

        return lane


    def publish_waypoints(self):

        lane = Lane()
        close_wp_idx = self.get_closest_waypoint_index()

        if self.gostop == 2 and self.sline_idx == -1 :
            self.gostop = 1
            self.change = True

        elif self.gostop == 1 and self.sline_idx != -1:
            brake_distance = math.sqrt((self.pose.x - self.base_waypoints.waypoints[self.sline_idx].pose.pose.position.x) ** 2 + (self.pose.y - self.base_waypoints.waypoints[self.sline_idx].pose.pose.position.y) ** 2) - 0.5
            if brake_distance < 50:
                self.gostop = 2
                self.change = True

        if self.gostop == 1 and self.change:
            lane = self.vel_state(close_wp_idx, 1)
        elif self.gostop == 2 and self.change:
            lane = self.vel_state(close_wp_idx, 2)
        elif self.gostop == 1 and not self.change:
            lane = self.vel_state(close_wp_idx, 3)
        elif self.gostop == 2 and not self.change:
            lane = self.vel_state(close_wp_idx, 4)

        self.change = False
        self.send_wp = lane.waypoints
        lane.header = self.base_waypoints.header
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg.pose.position

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        self.sline_idx = msg.data
        if self.sline_idx != -1 and len(self.base_waypoints.waypoints) != 0 :
            self.sline_idx = (self.sline_idx - 5 + len(self.base_waypoints.waypoints)) % len(self.base_waypoints.waypoints)

    def obstacle_cb(self, msg):
        pass
    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
        
    def pre_vel_cb(self, msg):
        self.present_vel = msg.twist.linear.x


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')