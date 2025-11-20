#!/usr/bin/env python3
import math
import rospy
import rospkg

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix, NavSatStatus


class SimFakeGPS(object):
    def __init__(self):
        # Parameters
        self.robot_model_name = rospy.get_param("~robot_model_name", "amiga_model")

        # Reference "farm" location (lat/lon in degrees)
        # You can change these to your real farm later
        self.lat0 = math.radians(rospy.get_param("~lat0_deg", 40.1149))    # UIUC-ish
        self.lon0 = math.radians(rospy.get_param("~lon0_deg", -88.2249))
        self.alt0 = rospy.get_param("~alt0", 220.0)  # meters

        # WGS84-equivalent radius for small deltas
        self.R = 6378137.0  # meters

        # Publisher
        self.pub = rospy.Publisher("/sim/gps/fix", NavSatFix, queue_size=10)

        # Subscriber
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1
        )

        rospy.loginfo(
            "SimFakeGPS: publishing /sim/gps/fix for %s around lat0=%.6f, lon0=%.6f",
            self.robot_model_name,
            math.degrees(self.lat0),
            math.degrees(self.lon0),
        )

    def model_states_cb(self, msg):
        # Find robot index
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        x = pose.position.x  # meters (Gazebo world, East)
        y = pose.position.y  # meters (Gazebo world, North)
        z = pose.position.z

        # Convert ENU (x,y) to lat/lon using small-angle approximation
        d_lat = y / self.R
        d_lon = x / (self.R * math.cos(self.lat0))

        lat = self.lat0 + d_lat
        lon = self.lon0 + d_lon
        alt = self.alt0 + z  # just add world z for fun

        msg_out = NavSatFix()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.header.frame_id = "gps_link"

        msg_out.status.status = NavSatStatus.STATUS_FIX
        msg_out.status.service = NavSatStatus.SERVICE_GPS

        msg_out.latitude = math.degrees(lat)
        msg_out.longitude = math.degrees(lon)
        msg_out.altitude = alt

        # Fake covariance (medium-ish accuracy)
        msg_out.position_covariance = [4.0, 0, 0,
                                       0, 4.0, 0,
                                       0, 0, 16.0]
        msg_out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        try:
            self.pub.publish(msg_out)
        except rospy.ROSException:
            pass


if __name__ == "__main__":
    rospy.init_node("sim_fake_gps")
    node = SimFakeGPS()
    rospy.spin()
