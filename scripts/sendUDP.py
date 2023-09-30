import socket
import struct
import ctypes
import time

UDP_IP = "172.23.182.164"
UDP_PORT = 10000
UDP_ADR = (UDP_IP, UDP_PORT)

telem_buffer = struct.calcsize('<lfffffflfffffffffffffff')

start_handshake = struct.pack('iii', 0, 0, 0)
get_data_handshake = struct.pack('iii', 0, 0, 1)
socket.setdefaulttimeout(1)
UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_socket.bind((UDP_IP, UDP_PORT))
#UDP_socket.sendto(start_handshake, UDP_ADR)

#data, addr = UDP_socket.recvfrom(208)
#initial_info = struct.unpack('cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccciicccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc', data)

#UDP_socket.sendto(get_data_handshake, UDP_ADR)

data, addr = UDP_socket.recvfrom(telem_buffer)
telem_struct = struct.unpack('<lfffffflfffffffffffffff', data)

data_map = {"lap_time" : telem_struct[0], "x" : telem_struct[1], "z" : telem_struct[2], "y" : telem_struct[3],
                "yaw" : telem_struct[4], "pitch" : telem_struct[5], "roll" : telem_struct[6], "gear" : telem_struct[7], 
                "angular_vel_x" : telem_struct[8], "angular_vel_z" : telem_struct[9], "angular_vel_x" : telem_struct[10], 
                "vel_x" : telem_struct[11], "vel_z" : telem_struct[12], "vel_y" : telem_struct[13], "mph" : telem_struct[14],
                "fl_wheel_speed" : telem_struct[14], "fr_wheel_speed" : telem_struct[15], "rl_wheel_speed" : telem_struct[16],
                "rr_wheel_speed" : telem_struct[17], "throttle" : telem_struct[18], "brake" : telem_struct[19], "steering_angle" : telem_struct[20]
                }

print(telem_struct)