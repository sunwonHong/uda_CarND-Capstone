#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml
import math

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoint_tree = None
        self.waypoints_2d = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.tlclassifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= 3:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):

        #TODO implement
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):

        if (not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        return self.tlclassifier.get_classification(cv_image)

    def process_traffic_lights(self):

        sline_idx = None
        closest_light = None
        line_idx = None

        stop_line_positions = self.config['stop_line_positions']

        if self.pose:
            car_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                d = temp_wp_idx - car_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_idx = temp_wp_idx

            if line_idx != -1:
                if line_idx > car_idx:
                    dis = math.sqrt((self.waypoints.waypoints[car_idx].pose.pose.position.x - self.waypoints.waypoints[line_idx].pose.pose.position.x) ** 2 + (self.waypoints.waypoints[car_idx].pose.pose.position.y - self.waypoints.waypoints[line_idx].pose.pose.position.y) ** 2)
                    if dis < 300:
                        sline_idx = self.get_closest_waypoint(self.waypoints.waypoints[line_idx].pose.pose.position.x, self.waypoints.waypoints[line_idx].pose.pose.position.y)

        if closest_light:
            state = self.get_light_state(closest_light)
            return sline_idx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')