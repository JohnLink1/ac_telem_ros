from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    hostIPaddress = "192.168.0.200"
    carIPaddress = "192.168.0.51"
    oppIPaddress = "192.168.0.50"
    
    ac_listener = Node(
        package="ac_telem",
        executable="ac_bridge",
        remappings=[],
        parameters=[
            {"ego1ip" : '192.168.0.200'}, 
            {"ego1port" : 10000}
        ]
    )
    ac_controller = Node(
        package="ac_telem",
        executable="ac_controller",
        # remappings=[
        #     ('joystick/accelerator_cmd', 'throttle'),
        #     ('joystick/brake_cmd', 'brake'),
        #     ('joystick/steering_cmd', 'steering_angle'),
        #     ('joystick/gear_cmd', 'current_gear')
        # ],
        parameters=[
            {"ego1ip" : carIPaddress}, 
            {"ego1port" : 10001}
        ]
        
    )

    opp_ac_listener = Node(
        package="ac_telem",
        executable="ac_bridge",
        remappings=[],
        namespace="opp",
        parameters=[
            {"ego1ip" : hostIPaddress}, 
            {"ego1port" : 10002}
        ]
    )
    opp_ac_controller = Node(
        package="ac_telem",
        executable="ac_controller",
        namespace="opp",
        # remappings=[
        #     ('joystick/accelerator_cmd', 'throttle'),
        #     ('joystick/brake_cmd', 'brake'),
        #     ('joystick/steering_cmd', 'steering_angle'),
        #     ('joystick/gear_cmd', 'current_gear')
        # ],
        parameters=[
            {"ego1ip" : oppIPaddress}, 
            {"ego1port" : 10003}
        ]
        
    )

    ld.add_action(ac_listener)
    ld.add_action(ac_controller)
    #ld.add_action(opp_ac_listener)
    #ld.add_action(opp_ac_controller)
    return ld