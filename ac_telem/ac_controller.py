import rclpy, time
from rclpy.node import Node
import struct, socket, ctypes

from std_msgs.msg import String, Float32, UInt8
from deep_orange_msgs.msg import JoystickCommand
from deep_orange_msgs.msg import PtReport

class ACController(Node):

    def __init__(self):
        super().__init__('ac_controller')

        self.declare_parameter('ego1ip', '192.168.0.51')
        self.declare_parameter('ego1port', 10000)
        
        UDP1_IP = self.get_parameter('ego1ip').get_parameter_value().string_value
        UDP1_PORT = self.get_parameter('ego1port').get_parameter_value().integer_value
        
        qos = rclpy.qos.QoSProfile(
                history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
                )
        
        self.accel = self.create_subscription(Float32, 'joystick/accelerator_cmd', self.accel_callback, qos_profile=qos)
        self.brake = self.create_subscription(Float32, 'joystick/brake_cmd', self.brake_callback, qos_profile=qos)
        self.steer = self.create_subscription(Float32, 'joystick/steering_cmd', self.steer_callback, qos_profile=qos)
        self.gear_sub = self.create_subscription(UInt8, 'joystick/gear_cmd', self.gear_callback, qos_profile=qos)
        self.cur_gear_sub = self.create_subscription(PtReport, 'raptor_dbw_interface/pt_report', self.pt_callback, qos_profile=1)
        
        timer_period = 1/100  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.shift_safety = False
        self.UDP1_ADR = (UDP1_IP, UDP1_PORT)

        self.control_format = 'fffi'
        control_size = struct.calcsize(self.control_format)
        socket.setdefaulttimeout(2)
        self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, control_size)
        self.throttle = 0.0
        self.brake = 0.0
        self.steering_angle = 0.0
        self.gear = 0
        self.shift = 0
        self.cur_gear = 0
        
    def accel_callback(self, data):
        self.throttle = data.data / 100 * 5
        
    def brake_callback(self, data):
        self.brake = data.data / 8000001.0
    
    def steer_callback(self, data):
        self.steering_angle = data.data
        
    def gear_callback(self, data):
        if(self.gear < data.data and not self.shift_safety):
            self.shift = 1
            self.shift_safety = True
        if(self.gear > data.data and not self.shift_safety):
            self.shift = -1
            self.shift_safety = True
        if(self.cur_gear == data.data):
            self.shift_safety = False
            self.gear = data.data

    def pt_callback(self, data):
        self.cur_gear = data.current_gear

    def timer_callback(self):
        self.send_commands(self.UDP1_ADR, self.throttle, self.brake, self.steering_angle, self.shift)
        self.shift = 0
            
    def send_commands(self, UDP_ADR, steer, throttle, brake, shift):
        try:
            command = struct.pack('fffi', steer, throttle, brake, shift)
            self.UDP_socket.sendto(command, UDP_ADR)
        except:
            self.get_logger().info('No Connection')



def main(args=None):
    rclpy.init(args=args)

    ac_controller = ACController()

    rclpy.spin(ac_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ac_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()