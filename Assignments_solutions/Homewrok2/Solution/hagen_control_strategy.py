import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time 
import matplotlib.pyplot as plt
import numpy as np 

class ControlStrategy(Node):
    def __init__(self, delta_t,):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.vel_sub
        self.odom_sub
        self.i = 0 #index of current segment
        self.set_q_init = None
        self.q = None 
        self.r = 0.3 # Wheel radius
        self.L = 1.25 # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.t = np.arange(0, 10, self.Ts) # Simulation time
        self.end_controller = False
        self.total_q_anal = []
        self.total_q_sims = []
        self.timer = self.create_timer(self.Ts, self.timer_callback)

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

    def stop_vehicle(self, ):
        self.send_vel(0.0, 0.0)    
    
    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]

    def reference_path_follower_diff_drive_init(self, duration=50, 
            control_points=np.array([[3,0], [6,4], [3,4], [3,1], [0,3]]), k_p=0.4, 
            k_theta=3):
        self.duration = duration
        self.control_points  = control_points
        self.k_p = k_p 
        self.k_theta = k_theta
        self.k_r = 0.2
        self.dmin = 0.7
        self.time_utilized = 0.0
        self.t_i0 = [0,0]        #t_i is intial position
        self.t_i1 = self.control_points[self.i]  #t_i+1 is first item of control points

    def reference_path_follower_diff_drive(self):
         if(self.q is not None):
            #stop when simulation time ends
            """  if(self.duration < self.time_utilized):
                self.stop_vehicle()
                print("End of simulation")
                self.end_controller = True """
            
           
            delta_x = self.t_i1[0] - self.t_i0[0]
            delta_y = self.t_i1[1] - self.t_i0[1]
            #vector aligned with current segment
            v = np.array([ 
                [delta_x],
                [delta_y]
            ])
            #vector orthogonal to current segment
            v_n = np.array([ 
                [delta_y],
                [-delta_x]
            ])
            # calculate r vector from T_i to robot position
            r_x = self.q[0]-self.t_i0[0]
            r_y = self.q[1]-self.t_i0[1]
            r = np.array([ 
                [r_x],
                [r_y]
            ])
            #calculate the projection of r into v
            u  = np.dot(v.transpose(),r) / np.dot(v.transpose(),v)
             
            #stop when reach end point of last segment
            if(u >1 and self.i == len(self.control_points )-1):
                self.stop_vehicle()
                print("Current pose is: ", self.q[0],", ", self.q[1])
                print("Reach to final control point")
                self.end_controller = True    
            elif (u >1):
                print("Current pose is: ", self.q[0],", ", self.q[1])
                print("change path segment from path",self.i, " to path",self.i+1)
                self.t_i0 = self.control_points[self.i]       
                self.t_i1 = self.control_points[self.i+1]
                self.i += 1  #increment current segment
            #calculate the projection of r into v_n
            d  = np.dot(v_n.transpose(),r) / np.dot(v_n.transpose(),v_n)
            #distance from robot to t_i+1
            self.D = np.sqrt((self.q[0]-self.t_i1[0])**2 + (self.q[1]-self.t_i1[1])**2)
    
            phi_lin = np.arctan2(delta_y,delta_x)        
            phi_rot = np.arctan(self.k_r * d)
            phi_ref = self.wrap_to_pi( phi_lin + phi_rot)
            e_phi = self.wrap_to_pi( float(phi_ref -  self.q[2]))
            v = self.k_p * np.cos(e_phi)
            w = self.k_theta*e_phi
            #print("Distance to the end of current segment: ", self.D)
            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2)
                            , v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q + self.Ts*dq # Integration
            self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts 
            self.total_q_anal.append(self.q)

    def timer_callback(self, ):
        if self.end_controller is False:
            #self.inter_direction_diff_drive()
            #self.inter_point_diff_drive()
            self.reference_path_follower_diff_drive()
        else:
            self.send_vel(0.0, 0.0)
            self.destroy_timer(self.timer)
            return 

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if(self.set_q_init is None):
            self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.set_q_init
        self.total_q_sims.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]))

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ControlStrategy(delta_t=0.03)
    #minimal_publisher.inter_direction_diff_drive_init()
    #minimal_publisher.inter_point_diff_drive_init()
    minimal_publisher.reference_path_follower_diff_drive_init()

    while minimal_publisher.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(minimal_publisher)
        except KeyboardInterrupt:
            break

    minimal_publisher.total_q_anal = np.array(minimal_publisher.total_q_anal)
    minimal_publisher.total_q_sims = np.array(minimal_publisher.total_q_sims)
    plt.plot(minimal_publisher.control_points[:,0],minimal_publisher.control_points[:,1],'r--',label = "Reference path segments")
    plt.plot(minimal_publisher.total_q_anal[:,0],minimal_publisher.total_q_anal[:,1],'b-',label = "Analytical path")
    plt.plot(minimal_publisher.total_q_sims[:,0],minimal_publisher.total_q_sims[:,1],'g-',label = "Simulation path")
    plt.title("Trajectroy for reference path following")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()
    plt.show() 
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
