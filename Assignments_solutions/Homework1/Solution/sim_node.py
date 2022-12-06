import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time 
import matplotlib.pyplot as plt
import numpy as np 

class MinimalPublisher1(Node):
    def __init__(self, delta_t):
        super().__init__('minimal_publisher1')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.vel_sub
        self.odom_sub
        self.i = 0
        self.time = 0.0
        self.actual_data = []
        self.set_q_init = None
        self.r = 0.3 # Wheel radius
        self.L = 1.25 # Axle length
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

    

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        self.actual_data.append(self.set_q_init)
        self.time += 0.03
        time.sleep(0.03)   #sleep for delta_t to synchronize with desired data
        if (self.time > 5): #exit the loop after certain simulation time
            self.plot()


    def plot(self):
        actual_data = np.array(self.actual_data)

        plt.figure("Actual Trajectroy")
        plt.plot(actual_data[:,0],actual_data[:,1])

        plt.figure("Actual x and y")
        plt.plot(actual_data[:,0],'r',label = 'x')
        plt.plot(actual_data[:,1],'g',label = 'y')
        #plt.plot(actual_data[:,2],'b',label = '${\phi}$')
        plt.title("Actual Data")
        plt.xlabel("Timestep")
        plt.ylabel("Distance [meter]")
        plt.legend()

        plt.figure("Actual phi")
        plt.plot(actual_data[:,2],'b',label = '${\phi}$')
        plt.title("Actual orientation")
        plt.xlabel("Timestep")
        plt.ylabel("Angle [radian]")
        plt.show()

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

SPIN_QUEUE = []
PERIOD = 0.01

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher1 = MinimalPublisher1(delta_t=0.03)
    SPIN_QUEUE.append(minimal_publisher1)
    while rclpy.ok():
        try:
            if(minimal_publisher1.set_q_init is not None):
                    print(f"Init value is set: {minimal_publisher1.set_q_init}")
                    break
            # for node in SPIN_QUEUE:
            #     rclpy.spin_once(node, timeout_sec=(PERIOD / len(SPIN_QUEUE)))
        except Exception as e:
            print(f"something went wrong in the ROS Loop: {e}")
        rclpy.spin_once(minimal_publisher1)
    
 
    rclpy.spin(minimal_publisher1)
    
    minimal_publisher1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty