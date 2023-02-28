import rospy
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from math import atan2, sqrt

class Interceptor:
    def __init__(self):
        # Initialize node
        rospy.init_node('interceptor')

        # Drone's state information
        self.drone_state = State()

        # Enemy drone's pose information
        self.enemy_pose = PoseStamped()

        # Velocity command to be sent to the drone
        self.cmd_vel = Twist()

        # Set the drone's altitude to 10m
        self.target_altitude = 10

        # Subscribe to the necessary topics
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/enemy_drone/pose', PoseStamped, self.enemy_pose_cb)

        # Publish velocity commands to the drone
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

        # Connect to the necessary services
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Set the loop rate
        self.rate = rospy.Rate(20)

    def state_cb(self, msg):
        # Callback function to update drone's state information
        self.drone_state = msg

    def enemy_pose_cb(self, msg):
        # Callback function to update enemy drone's pose information
        self.enemy_pose = msg

    def run(self):
        # Set the initial mode and arm the drone
        self.set_mode_service(custom_mode="OFFBOARD")
        self.arm_service(True)

        while not rospy.is_shutdown():
            # Calculate the distance and bearing to the enemy drone
            dx = self.enemy_pose.pose.position.x - self.drone_state.pose.position.x
            dy = self.enemy_pose.pose.position.y - self.drone_state.pose.position.y
            dz = self.enemy_pose.pose.position.z - self.drone_state.pose.position.z
            distance = sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            bearing = atan2(dy, dx)

            # Calculate the velocity required to intercept the enemy drone
            self.cmd_vel.linear.x = 2 * distance * sin(bearing)
            self.cmd_vel.linear.y = -2 * distance * cos(bearing)
            self.cmd_vel.linear.z = self.target_altitude - self.drone_state.pose.position.z

            # Publish the velocity command to the drone
            self.vel_pub.publish(self.cmd_vel)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        interceptor = Interceptor()
        interceptor.run()
    except rospy.ROSInterruptException:
        pass
