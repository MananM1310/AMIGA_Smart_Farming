#!/usr/bin/env python3
import os
import json
import math

import rospy
import rospkg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class SimWeedOracle(object):
    def __init__(self):
        # Find the weed_positions.json we just made in gen_weeds.py
        rp = rospkg.RosPack()
        amiga_sim_path = rp.get_path("amiga_sim")
        json_path = os.path.join(
            amiga_sim_path, "models", "farm_weeds", "weed_positions.json"
        )

        if not os.path.exists(json_path):
            rospy.logerr("SimWeedOracle: weed_positions.json not found at %s", json_path)
            raise RuntimeError("weed_positions.json missing")

        with open(json_path, "r") as f:
            self.weeds = json.load(f)

        rospy.loginfo("SimWeedOracle: loaded %d weeds from %s",
                      len(self.weeds), json_path)

        # Parameters
        self.row_spacing = rospy.get_param("~row_spacing", 1.18)
        self.robot_model_name = rospy.get_param("~robot_model_name", "amiga_model")
        # Small epsilon so we don't consider weeds exactly at the robot as "ahead"
        self.ahead_eps = rospy.get_param("~ahead_eps", 0.1)
        self.treat_distance = rospy.get_param("~treat_distance", 0.3)

        # Publisher: nearest weed on current row, ahead of the robot
        self.pub = rospy.Publisher("/sim/nearest_weed", PoseStamped, queue_size=10)

        # Subscriber: robot pose from Gazebo
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1
        )

    def compute_row_index(self, y):
        """
        R2: use exact row index based on y and row_spacing.
        In gen_weeds.py: y = (i + 1) * row_spacing * sign
        => i = (|y| / row_spacing) - 1
        """
        return int(round((abs(y) / self.row_spacing) - 1.0))

    def model_states_cb(self, msg):
        # Find the robot in the ModelStates message
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            # Robot not in this message
            return

        pose = msg.pose[idx]
        x_r = pose.position.x
        y_r = pose.position.y

        # If robot exactly on y=0, row index formula is ambiguous; skip
        if abs(y_r) < 1e-6:
            return

        # Robot's row index (R2)
        row_idx_robot = self.compute_row_index(y_r)
        y_sign_robot = 1 if y_r >= 0.0 else -1
        x_sign_robot = 1 if x_r >= 0.0 else -1

        best_idx = None     # index in self.weeds
        best = None         # (x_w, y_w)
        best_dist = None

        for idx, w in enumerate(self.weeds):
            x_w = w["x"]
            y_w = w["y"]

            # (Optional) keep only same quadrant so we don't jump across the field
            if x_w * x_r < 0 or y_w * y_r < 0:
                continue

            dx = x_w - x_r
            dy = y_w - y_r
            d = math.sqrt(dx*dx + dy*dy)

            if best is None or d < best_dist:
                best_idx = idx
                best = (x_w, y_w)
                best_dist = d

        if best is None:
            # No weed near us in this quadrant
            return

        # If we're close enough, consider this weed "treated" and drop it
        if best_dist <= self.treat_distance:
            try:
                done_weed = self.weeds.pop(best_idx)
                rospy.loginfo(
                    "SimWeedOracle: marking weed %s as done (d=%.2f)",
                    done_weed.get("name", "?"),
                    best_dist,
                )
            except Exception as e:
                rospy.logwarn("SimWeedOracle: failed to pop weed idx %s: %s",
                              best_idx, e)
            # Don't publish this weed anymore
            return

        if rospy.is_shutdown():
            return

        # Publish nearest remaining weed
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "world"
        ps.pose.position.x = best[0]
        ps.pose.position.y = best[1]
        ps.pose.position.z = 0.0

        try:
            self.pub.publish(ps)
        except rospy.ROSException:
            pass


if __name__ == "__main__":
    rospy.init_node("sim_weed_oracle")
    try:
        node = SimWeedOracle()
        rospy.spin()
    except Exception as e:
        rospy.logerr("SimWeedOracle crashed: %s", e)

