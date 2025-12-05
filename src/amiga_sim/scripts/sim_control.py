#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import math

class SimController:
    def __init__(self):
        rospy.init_node('twist_to_wheels')

        # Robot physical parameters
        wheel_radius = 0.216   # meters
        wheel_base = 0.81     # meters (distance between left and right wheels)

        self.L = wheel_base
        self.R = wheel_radius
        self.model_name = 'amiga_model'

        # Publishers mapped to correct wheel sides
        self.pubs = {
            "bl": rospy.Publisher('/bl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "fl": rospy.Publisher('/fl_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "br": rospy.Publisher('/br_wheel_joint_velocity_controller/command', Float64, queue_size=1),
            "fr": rospy.Publisher('/fr_wheel_joint_velocity_controller/command', Float64, queue_size=1),
        }

        # Publisher for simulated GPS data
        self.gps_pub = rospy.Publisher('/gps/pvt', NavSatFix, queue_size=1)

        # GPS origin (flat Earth approximation)
        # GPS simulation origin
        self.lat0 = 40.0     # degrees north (e.g., Nebraska or Kansas)
        self.lon0 = -93.0    # central meridian of Zone 15N
        self.alt0 = 300.0    # elevation in meters (reasonable ground level)
        self.earth_radius = 6378137.0  # meters

        # Storage for latest command velocities
        self.commanded_velocities = {
            'bl_wheel_joint': 0.0,
            'fl_wheel_joint': 0.0,
            'br_wheel_joint': 0.0,
            'fr_wheel_joint': 0.0,
        }

        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0
        self.boosted_linear_x = 0.0
        self.boosted_angular_z = 0.0

        self.latest_gazebo_linear = 0.0
        self.latest_gazebo_angular = 0.0

        self.gazebo_linear_history = []
        self.target_linear_history = []
        self.gazebo_angular_history = []
        self.target_angular_history = []

        # For timeout watchdog
        self.last_cmd_time = rospy.Time.now()
        self.cmd_timeout = rospy.Duration(0.5)  # seconds
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.1), self.watchdog_callback)

        # Subscribers
        rospy.Subscriber('/amiga/cmd_vel', TwistStamped, self.cmd_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        rospy.on_shutdown(self.plot_linear_velocity_history)
        rospy.on_shutdown(self.plot_angular_velocity_history)

        rospy.spin()

    def model_states_callback(self, msg):
        if self.model_name in msg.name:
            index = msg.name.index(self.model_name)
            twist = msg.twist[index]
            pose = msg.pose[index]

            self.latest_gazebo_linear = (twist.linear.x ** 2 + twist.linear.y ** 2) ** 0.5
            self.latest_gazebo_angular = twist.angular.z

            # Simulated GPS publishing
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            d_lat = (x / self.earth_radius) * (180.0 / math.pi)
            d_lon = (y / (self.earth_radius * math.cos(self.lat0 * math.pi / 180.0))) * (180.0 / math.pi)

            gps_msg = NavSatFix()
            gps_msg.header.stamp = rospy.Time.now()
            gps_msg.header.frame_id = "gps_link"
            gps_msg.latitude = self.lat0 + d_lat
            gps_msg.longitude = self.lon0 + d_lon
            gps_msg.altitude = self.alt0 + z
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.gps_pub.publish(gps_msg)

    def boost_linear_power(self, x):
        return x

    def boost_angular_power(self, x):
        # Linear boost to overcome friction
        return x * 4.0

    def cmd_callback(self, msg):
        self.last_cmd_time = rospy.Time.now()

        # Use linear.x directly to preserve sign (important for backward motion)
        linear_vel = msg.twist.linear.x
        angular_vel = msg.twist.angular.z

        self.latest_linear_x = linear_vel
        self.latest_angular_z = angular_vel
        self.boosted_linear_x = self.boost_linear_power(linear_vel)
        self.boosted_angular_z = self.boost_angular_power(angular_vel)

        v_left = (self.boosted_linear_x - self.boosted_angular_z * self.L / 2.0) / self.R
        v_right = (self.boosted_linear_x + self.boosted_angular_z * self.L / 2.0) / self.R

        self.commanded_velocities['bl_wheel_joint'] = v_left
        self.commanded_velocities['fl_wheel_joint'] = v_left
        self.commanded_velocities['br_wheel_joint'] = v_right
        self.commanded_velocities['fr_wheel_joint'] = v_right

        self.pubs['bl'].publish(v_left)
        self.pubs['fl'].publish(v_left)
        # Invert right wheels because of URDF axis orientation
        self.pubs['br'].publish(-v_right)
        self.pubs['fr'].publish(-v_right)

    def watchdog_callback(self, event):
        if rospy.Time.now() - self.last_cmd_time > self.cmd_timeout:
            for joint in self.commanded_velocities:
                self.commanded_velocities[joint] = 0.0
            for pub in self.pubs.values():
                pub.publish(0.0)

    def joint_state_callback(self, msg):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        output_lines = []

        output_lines.append(f"Subscribed linear.x: {self.latest_linear_x:.3f}, Subscribed angular.z: {self.latest_angular_z:.3f}\n")
        output_lines.append(f"Boosted linear.x: {self.boosted_linear_x:.3f}, Boosted angular.z: {self.boosted_angular_z:.3f}\n")

        actual_velocities = {}

        for joint, commanded in self.commanded_velocities.items():
            if joint in name_to_index:
                actual = msg.velocity[name_to_index[joint]]
                actual_velocities[joint] = actual
                error = abs(commanded - actual)
                output_lines.append(f"{joint} -> Commanded: {commanded:.3f}, Actual: {actual:.3f}, |Error|: {error:.3f}")

        if all(j in actual_velocities for j in ['fl_wheel_joint', 'fr_wheel_joint', 'bl_wheel_joint', 'br_wheel_joint']):
            left_avg = (actual_velocities['fl_wheel_joint'] + actual_velocities['bl_wheel_joint']) / 2.0
            right_avg = (actual_velocities['fr_wheel_joint'] + actual_velocities['br_wheel_joint']) / 2.0
            diff = left_avg - right_avg

            output_lines.append("")
            if diff > 0.01:
                output_lines.append("🔁 Actual motion: TURNING RIGHT")
            elif diff < -0.01:
                output_lines.append("🔄 Actual motion: TURNING LEFT")
            else:
                output_lines.append("⬆️  Actual motion: DRIVING STRAIGHT")

            fl_fr_diff = actual_velocities['fl_wheel_joint'] - actual_velocities['fr_wheel_joint']
            bl_br_diff = actual_velocities['bl_wheel_joint'] - actual_velocities['br_wheel_joint']

            output_lines.append(f"fl - fr = {fl_fr_diff:.3f}   (Front left vs right)")
            output_lines.append(f"bl - br = {bl_br_diff:.3f}   (Back left vs right)")

        output_lines.append("")

        epsilon = 1e-3

        lin_err = abs(self.latest_gazebo_linear - self.latest_linear_x)
        ang_err = abs(self.latest_gazebo_angular - self.latest_angular_z)

        lin_pct = 0.0 if abs(self.latest_linear_x) < epsilon and abs(self.latest_gazebo_linear) < epsilon else 100.0 * lin_err / (abs(self.latest_linear_x) + epsilon)
        ang_pct = 0.0 if abs(self.latest_angular_z) < epsilon and abs(self.latest_gazebo_angular) < epsilon else 100.0 * ang_err / (abs(self.latest_angular_z) + epsilon)

        output_lines.append(f"Gazebo actual linear.x: {self.latest_gazebo_linear:.3f} vs Target: {self.latest_linear_x:.3f}   | Error: {lin_err:.3f} ({lin_pct:.1f}%)")
        output_lines.append(f"Gazebo actual angular.z: {self.latest_gazebo_angular:.3f} vs Target: {self.latest_angular_z:.3f} vs Commanded: {self.boosted_angular_z:.3f}   | Error: {ang_err:.3f} ({ang_pct:.1f}%)")

        self.gazebo_linear_history.append(self.latest_gazebo_linear)
        self.target_linear_history.append(self.latest_linear_x)
        self.gazebo_angular_history.append(self.latest_gazebo_angular)
        self.target_angular_history.append(self.latest_angular_z)

        rospy.loginfo("\n" + "\n".join(output_lines) + "\n" + "-" * 50)

    def plot_linear_velocity_history(self):
        if not self.gazebo_linear_history or not self.target_linear_history:
            return
        script_dir = os.path.dirname(os.path.realpath(__file__))
        parent_dir = os.path.dirname(script_dir)
        data_dir = os.path.join(parent_dir, 'data')
        plt.figure()
        plt.plot(self.gazebo_linear_history, label='Actual Linear Velocity (m/s)')
        plt.plot(self.target_linear_history, label='Target Linear Velocity (m/s)', linestyle='--')
        plt.xlabel('Time Step')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Actual vs Target Linear Velocity')
        plt.legend()
        plt.grid(True)
        plt.savefig(data_dir + '/linear_velocity_comparison.png')
        plt.close()
        print("Figure (linear velocity) has been saved")

    def plot_angular_velocity_history(self):
        if not self.gazebo_angular_history or not self.target_angular_history:
            return
        script_dir = os.path.dirname(os.path.realpath(__file__))
        parent_dir = os.path.dirname(script_dir)
        data_dir = os.path.join(parent_dir, 'data')
        plt.figure()
        plt.plot(self.gazebo_angular_history, label='Actual Angular Velocity (rad/s)')
        plt.plot(self.target_angular_history, label='Target Angular Velocity (rad/s)', linestyle=':')
        plt.xlabel('Time Step')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.title('Actual vs Target Angular Velocity')
        plt.legend()
        plt.grid(True)
        plt.savefig(data_dir + '/angular_velocity_comparison.png')
        plt.close()
        print("Figure (angular velocity) has been saved")

if __name__ == '__main__':
    SimController()
