import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time 
import numpy as np 

class MinimalPublisher(Node):
    def __init__(self, delta_t):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.vel_sub
        self.odom_sub
        self.i = 0
        self.desired_data=[]   #to store the desired poses calculated from the simulation
        self.set_q_init = None
        self.r = 0.04 # Wheel radius
        self.L = 0.08 # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.t = np.arange(0, 10, self.Ts) # Simulation time

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        pass    
    
    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]

    def inter_direction_diff_drive(self, duration=5, r_distance=1.3, refPose=np.array([3,6,0]), k_p=0.5, k_w=0.7):
        q = self.set_q_init # np.array([4 ,0.5, np.pi/6]) # Initial pose
        time_utilized = 0.0
        while rclpy.ok():
            if(duration < time_utilized):

                print("End of simulation at pose: ",q)               
                self.send_vel(0.0, 0.0)
                break
            wL = 20 # Left wheel velocity
            wR = 18 # Right wheel velocity
            v = self.r/2*(wR+wL) # Robot velocity
            w = self.r/self.L*(wR-wL) # Robot angular velocity
            dq = np.array([v*np.cos(q[2]+self.Ts*w/2), v*np.sin(q[2]+self.Ts*w/2), w])
            q = q + self.Ts*dq # Integration
            q[2] = self.wrap_to_pi(q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            time.sleep(self.Ts)
            time_utilized  =  time_utilized + self.Ts
            self.desired_data.append(q)     #append the last desired state

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

SPIN_QUEUE = []
PERIOD = 0.01

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(delta_t=0.03)
    SPIN_QUEUE.append(minimal_publisher)
    while rclpy.ok():
        try:
            if(minimal_publisher.set_q_init is not None):
                    print(f"Init value is set: {minimal_publisher.set_q_init}")
                    break
            # for node in SPIN_QUEUE:
            #     rclpy.spin_once(node, timeout_sec=(PERIOD / len(SPIN_QUEUE)))
        except Exception as e:
            print(f"something went wrong in the ROS Loop: {e}")
        rclpy.spin_once(minimal_publisher)

    minimal_publisher.inter_direction_diff_drive(5)
    desired_data = np.array(minimal_publisher.desired_data)
    
    plt.figure("Desired Trajectroy")
    plt.title("Desired Trajectory")
    plt.plot(desired_data[:,0],desired_data[:,1])
 
    plt.title("Desired trajectory")
    plt.xlabel("Timestep")
    plt.ylabel("Distance [meter]")
    plt.legend()

    plt.figure("Desired x and y")
    plt.plot(desired_data[:,0],'r',label = 'x')
    plt.plot(desired_data[:,1],'g',label = 'y')
    plt.title("Desired trajectory")
    plt.xlabel("Timestep")
    plt.ylabel("Distance [meter]")
    plt.legend()

    plt.figure("Desired phi")
    plt.plot(desired_data[:,2],'b',label = '${\phi}$')
    plt.title("Desired orientation")
    plt.xlabel("Timestep")
    plt.ylabel("Angle [radian]")

    plt.show()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty