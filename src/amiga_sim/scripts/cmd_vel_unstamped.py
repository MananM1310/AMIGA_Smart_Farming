#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped, Twist


class CmdVelUnstamped(object):
    def __init__(self):
        in_topic = rospy.get_param("~in_topic", "/amiga/cmd_vel")
        out_topic = rospy.get_param("~out_topic", "/amiga/cmd_vel_unstamped")
        mirror_topic = rospy.get_param("~mirror_topic", "/cmd_vel")

        self.pub = rospy.Publisher(out_topic, Twist, queue_size=10)
        self.mirror_pub = rospy.Publisher(mirror_topic, Twist, queue_size=10)
        rospy.Subscriber(in_topic, TwistStamped, self.cb, queue_size=10)

        rospy.loginfo("cmd_vel_unstamped: %s -> %s and %s", in_topic, out_topic, mirror_topic)

    def cb(self, msg):
        tw = Twist()
        tw.linear = msg.twist.linear
        tw.angular = msg.twist.angular
        self.pub.publish(tw)
        self.mirror_pub.publish(tw)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_unstamped")
    CmdVelUnstamped()
    rospy.spin()
