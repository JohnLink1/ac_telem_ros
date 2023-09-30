import rclpy, time
from rclpy.node import Node
import math
import struct, socket, ctypes
import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String, Header
from deep_orange_msgs.msg import Autonomy, TireReport, PtReport
from nav_msgs.msg import Odometry
from raptor_dbw_msgs.msg import WheelSpeedReport
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, TwistWithCovariance, Vector3, PointStamped
import tf_transformations

from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped


class ACTelemPub(Node):

    def __init__(self):
        super().__init__('ac_telem_pub')
        
        self.declare_parameter('ego1ip', '172.23.182.164')
        self.declare_parameter('ego1port', 10000)
        self.declare_parameter('ego2ip', '127.0.0.1')
        self.declare_parameter('ego2port', 9996)
        
        UDP1_IP = self.get_parameter('ego1ip').get_parameter_value().string_value
        UDP1_PORT = self.get_parameter('ego1port').get_parameter_value().integer_value
        UDP2_IP = self.get_parameter('ego2ip').get_parameter_value().string_value
        UDP2_PORT = self.get_parameter('ego2port').get_parameter_value().integer_value
        
        qos = rclpy.qos.QoSProfile(
                history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
                )
        
        self.publisher_ = self.create_publisher(Autonomy, 'telemetry/autonomy', qos)
        self.odom_pub = self.create_publisher(Odometry, 'novatel_top/dyntf_odom', 1)
        self.wheel_pub = self.create_publisher(WheelSpeedReport, '/raptor_dbw_interface/wheel_speed_report', 1)
        self.tire_pub = self.create_publisher(TireReport, '/telemetry/tire_report', qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/telemetry/vehicle_viz', qos)
        self.accel_pub = self.create_publisher(PointStamped, '/telemetry/vehicle_accel', qos)
        self.pt_pub = self.create_publisher(PtReport, '/raptor_dbw_interface/pt_report', 1)
        self.tf                  = TransformStamped()
        self.tf_pub              = TransformBroadcaster(self)

        
        timer_period = 1/1000  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        timer2_period = 1/100  # seconds
        self.timer = self.create_timer(timer2_period, self.pub_data)
        timer3_period = 1/5  # seconds
        self.timer = self.create_timer(timer3_period, self.pub_telem)

        self.UDP1_ADR = (UDP1_IP, UDP1_PORT)
        self.UDP2_ADR = (UDP2_IP, UDP2_PORT)
        
        self.magic_yaw_offset = 90
        
        self.telem_message = '<lfffffflffffffffffffffffffffff'
        self.telem_buffer = struct.calcsize(self.telem_message)
        socket.setdefaulttimeout(2)
        self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.telem_buffer)
        self.UDP_socket.settimeout(1)
        self.UDP_socket.bind(self.UDP1_ADR)
        self.connected = False
        self.odom = Odometry()
        self.msg = Autonomy()
        self.twist = TwistWithCovariance()
        self.wheel = WheelSpeedReport()
        self.tires = TireReport()
        self.vehicle_pose = PoseStamped()
        self.vehicle_accel = PointStamped()
        self.engineInfo = PtReport()
        self.timeer = time.time()
        
    def pub_data(self):
        if self.connected:
            self.wheel_pub.publish(self.wheel)
            self.odom_pub.publish(self.odom)
            self.pt_pub.publish(self.engineInfo)
            
    def pub_telem(self):
        if self.connected:
            self.publisher_.publish(self.msg)
            self.tire_pub.publish(self.tires)
            self.pose_pub.publish(self.vehicle_pose)
            self.accel_pub.publish(self.vehicle_accel)
        

    def timer_callback(self):
        try:
            self.timeer = time.time()
            data = self.parse_data(self.send_get_data())
            
            self.engineInfo.engine_rpm = data["rpm"]
            self.engineInfo.engine_coolant_temperature = 50.0
            
            self.msg.accelerator_cmd = data["throttle"] * 100
            self.msg.brake_cmd = data["brake"] * 55150
            if data["gear"] < 1:
                self.msg.gear_cmd = 1
                self.engineInfo.current_gear = 1
            else:
                self.msg.gear_cmd = data["gear"]
                self.engineInfo.current_gear = data["gear"]
            self.msg.steering_cmd = data["steering_angle"]*-1
            self.msg.current_velocity = data["mph"]
            
            self.odom.child_frame_id = 'rear_axle_middle_ground'
            header = Header()
            header.frame_id = 'center_of_gravity'
            self.odom.header = header
            pose = Pose()
            
            x, y = self.transform(data["x"], data["y"] * -1, 0)
            pose.position.x = x
            pose.position.y = y
            pose.position.z = data["z"]
            data["yaw"] = ((data["yaw"]  * -1 ) + math.pi/2)
            if (data["yaw"] > math.pi):
                data["yaw"] = data["yaw"] - (math.pi * 2)
            if (data["yaw"] < -math.pi):
                data["yaw"] = data["yaw"] + (math.pi * 2)
            #self.get_logger().info(str(data["yaw"]))
            
            q = tf_transformations.quaternion_from_euler(data["roll"], data["pitch"], data["yaw"])

            orientation = R.from_quat(q).as_matrix()
            rotation_matrix = R.from_euler('z',180.0,degrees=True).as_matrix()
            resultant_matrix = np.matmul(orientation,rotation_matrix)
            q = (R.from_matrix(resultant_matrix)).as_quat()
            
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            self.vehicle_pose.pose = pose
            
            pos_cov = PoseWithCovariance()
            pos_cov.pose = pose
            self.odom.pose = pos_cov
            
            self.twist.twist.linear.x = data["vel_y"]
            self.twist.twist.linear.y = -data["vel_x"]
            self.twist.twist.linear.z = -data["vel_z"]
            
            
            self.twist.twist.angular.x = data["angular_vel_y"]
            self.twist.twist.angular.y = -data["angular_vel_x"]
            self.twist.twist.angular.z = -data["angular_vel_z"]
            self.odom.twist = self.twist
            
            
            self.tf.header.stamp = self.get_clock().now().to_msg()
            self.tf.header.frame_id='map'
            self.tf.child_frame_id='base_link'
            self.tf.transform = Transform(translation = Vector3(x = self.odom.pose.pose.position.x, y = self.odom.pose.pose.position.y, z=self.odom.pose.pose.position.z), rotation = self.odom.pose.pose.orientation)
            self.tf_pub.sendTransform(self.tf)
            
            # Radians/sec to kmph
            rad = 0.3127 * 3.6
            self.wheel.front_left = data["fl_wheel_speed"] * 0.3 * 3.6
            self.wheel.front_right = data["fr_wheel_speed"] * 0.3 * 3.6
            self.wheel.rear_left = data["rl_wheel_speed"] * 0.3127 * 3.6
            self.wheel.rear_right = data["rr_wheel_speed"] * 0.3127 * 3.6
            
            self.tires.fl_tire_temperature = np.full(16, data["fl_wheel_temp"]).tolist()
            self.tires.fr_tire_temperature = np.full(16, data["fr_wheel_temp"]).tolist()
            self.tires.rl_tire_temperature = np.full(16, data["rl_wheel_temp"]).tolist()
            self.tires.rr_tire_temperature = np.full(16, data["rr_wheel_temp"]).tolist()
            
            self.vehicle_accel.point.x = data["y_accel"]
            self.vehicle_accel.point.y = -data["x_accel"]
            self.vehicle_accel.point.z = data["z_accel"]
            #self.get_logger().info(str(time.time() - self.timeer))
            
            self.connected = True
            
            
        except Exception as e:
            self.get_logger().info(str(e))
            self.connected = False
            self.get_logger().info('No Connection')
            
    def transform(self, x, y, deg):
        new_x = math.cos(deg * math.pi/180) * x + math.sin(deg * math.pi/180) * y
        new_y = -math.sin(deg * math.pi/180) * x + math.cos(deg * math.pi/180) * y
        return new_x, new_y

    def parse_data(self, telem_struct):
        data_map = {"lap_time" : telem_struct[0], "x" : telem_struct[1], "z" : telem_struct[2], "y" : telem_struct[3],
                "yaw" : telem_struct[4], "pitch" : telem_struct[5], "roll" : telem_struct[6], "gear" : telem_struct[7], 
                "angular_vel_x" : telem_struct[8], "angular_vel_z" : telem_struct[9], "angular_vel_y" : telem_struct[10], 
                "vel_x" : telem_struct[11], "vel_z" : telem_struct[12], "vel_y" : telem_struct[13], "mph" : telem_struct[14],
                "fl_wheel_speed" : telem_struct[15], "fr_wheel_speed" : telem_struct[16], "rl_wheel_speed" : telem_struct[17],
                "rr_wheel_speed" : telem_struct[18], "throttle" : telem_struct[19], "brake" : telem_struct[20], "steering_angle" : telem_struct[21], 
                "rpm" : telem_struct[22], "fl_wheel_temp" : telem_struct[23], "fr_wheel_temp" : telem_struct[24], "rl_wheel_temp" : telem_struct[25],
                "rr_wheel_temp" : telem_struct[26], "x_accel" : telem_struct[27], "z_accel" : telem_struct[28], "y_accel" : telem_struct[29]
                }
        return data_map

    def send_get_data(self):
            data, addr = self.UDP_socket.recvfrom(self.telem_buffer)
            return struct.unpack(self.telem_message, data)



def main(args=None):
    rclpy.init(args=args)

    ac_telem_pub = ACTelemPub()

    rclpy.spin(ac_telem_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ac_telem_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()