import time, socket, struct, vgamepad as vg
import configparser


def run():
    global cg, gamepad, control_buff, UDP_address_port, UDP_socket, control_format
    UDP_address = '127.0.0.1' 
    UDP_port = 10001

    config = configparser.ConfigParser()
    try:
        config.read("resource/config.ini")
        UDP_address = config.get("ros_telem_config", "host_ip_address")
        UDP_port = int(config.get("ros_telem_config", "host_port"))
    except Exception as e:
        print(e)

    UDP_address_port = (UDP_address, UDP_port)

    control_format = 'fffi'
    control_buff = struct.calcsize(control_format)
    gamepad = vg.VX360Gamepad()

    socket.setdefaulttimeout(1)
    UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_socket.bind((UDP_address, UDP_port))
    UDP_socket.settimeout(2)
    UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, control_buff)

    
    try:
        while True:
            udp_socket_client()
    except Exception as e:
        if e == KeyboardInterrupt:
            UDP_socket.close()
            exit

def send_ass_control(control):
    gamepad.left_trigger_float(value_float = control["brake"])
    gamepad.right_trigger_float(value_float = control["throttle"])
    gamepad.left_joystick_float(x_value_float = control["steer"] * -1, y_value_float = 0.0)
    
    gamepad.update()

    if control["shift"] == 1:
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
        gamepad.update()
        time.sleep(0.1)
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
        gamepad.update()

    elif control["shift"] == -1:
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
        gamepad.update()
        time.sleep(0.1)
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
        gamepad.update()

    #time.sleep(0.0001)
       

def udp_socket_client():
    try:
        data, addr = UDP_socket.recvfrom(control_buff)
        control_struct = struct.unpack(control_format, data)
        control_map = {"throttle": control_struct[0], 'brake': control_struct[1], 'steer': (control_struct[2]/24), 'shift': control_struct[3]}
        #control_map['shift'] = 0
        send_ass_control(control_map)
        #print("Throttle:", control_map["throttle"], "   Steer:", control_map["steer"])
    except Exception as e:
        if e == KeyboardInterrupt:
            UDP_socket.close()
            exit
        time.sleep(0.2)
        #print("No Connection")

if __name__ == '__main__':
    run()