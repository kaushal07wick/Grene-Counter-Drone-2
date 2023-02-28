#!/usr/bin python3

import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry

class DroneInterceptor:
    
    def __init__(self):
        # Initialize ROS node and subscribers/publishers
        rospy.init_node('drone_interceptor')
        self.sub_odom = rospy.Subscriber('/drone/odom', Odometry, self.odom_callback)
        self.sub_target = rospy.Subscriber('/drone/target', PoseStamped, self.target_callback)
        self.pub_cmd = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=10)

        # Initialize PN parameters
        self.k = 1.0  # Proportional gain
        self.cmd_vel = Twist()
        self.last_time = rospy.Time.now()
        self.last_target_pos = None
        self.current_pos = Vector3(0, 0, 0)
        self.current_vel = Vector3(0, 0, 0)
        self.current_yaw = 0

    def odom_callback(self, msg):
        # Calculate time delta
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Update drone state
        self.current_pos = msg.pose.pose.position
        self.current_vel = msg.twist.twist.linear
        self.current_yaw = self.get_yaw(msg.pose.pose.orientation)

        # Calculate LOS rate and update command velocity
        if self.last_target_pos is not None:
            target_pos = self.last_target_pos
            target_vel = (target_pos - self.current_pos) / dt
            los_vector = target_pos - self.current_pos
            los_rate = self.get_los_rate(los_vector, target_vel, self.current_vel)
            self.cmd_vel.linear.x = self.k * los_rate[0]
            self.cmd_vel.linear.y = self.k * los_rate[1]
            self.cmd_vel.linear.z = self.k * los_rate[2]

        # Publish command velocity
        self.pub_cmd.publish(self.cmd_vel)

    def target_callback(self, msg):
        # Update last target position
        self.last_target_pos = msg.pose.position

    def get_yaw(self, quat):
        # Convert quaternion to Euler angles and return yaw angle
        euler = [0, 0, 0]
        euler[0], euler[1], euler[2] = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def get_los_rate(self, los_vector, target_vel, drone_vel):
        # Transform LOS vector to drone frame
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        rot_matrix = [[cos_yaw, -sin_yaw, 0],
                      [sin_yaw, cos_yaw, 0],
                      [0, 0, 1]]
        los_vector_drone = [rot_matrix[0][0] * los_vector.x + rot_matrix[0][1] * los_vector.y + rot_matrix[0][2] * los_vector.z,
                            rot_matrix[1][0] * los_vector.x + rot_matrix[1][1] * los_vector.y + rot_matrix[1][2] * los_vector.z,
                            rot_matrix[2][0] * los_vector.x + rot_matrix[2][1] * los_vector.y + rot_matrix[2][2] * los_vector.z]

        # Calculate relative velocity between target and drone
        relative_vel = [target_vel.x - drone_vel.x                       target_vel.y - drone_vel.y,
                       target_vel.z - drone_vel.z]

        # Calculate LOS rate
        los_rate = [los_vector_drone[1] * relative_vel[2] - los_vector_drone[2] * relative_vel[1],
                    los_vector_drone[2] * relative_vel[0] - los_vector_drone[0] * relative_vel[2],
                    los_vector_drone[0] * relative_vel[1] - los_vector_drone[1] * relative_vel[0]]
        return los_rate

if __name__ == '__main__':
    try:
        interceptor = DroneInterceptor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass