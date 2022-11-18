# ========================================================================================================
# Assetto Corsa Telemetry To ROS2
# - written by Latch Dimitrov - Dec 2017
# - updated by John Link - Nov 2022
# - contact: latch(DOT)dimitrov(DOT)00(AT)gmail(DOT)com, john(DOT)link33(AT)gmail(DOT)com
# ========================================================================================================

import os
import sys
import platform
if platform.architecture()[0] == "64bit":
    sys.path.insert(0, "apps/python/ros_telem/stdlib64")
else:
    sys.path.insert(0, "apps/python/ros_telem/stdlib")
os.environ['PATH'] = os.environ['PATH'] + ";."
import ac
import acsys
import configparser
import locale
import re
import socket
import struct
import subprocess
import sim_info
import time


locale.setlocale(locale.LC_ALL, '')

class Cacti:

    is_using_local_acti=False
    trig_socket=0
    udp2_socket=0
    udp2_dataflow_ip='172.23.182.164'
    SOCKET_TIMEOUT=3.0
    ACTI_TRIG_PORT=10000
    ACTI_UDP2_PORT=10000
    TRIG_CONNECT_REQ=5500
    TRIG_DISCONNECT_REQ=5501
    TRIG_KILL_REQ=5502
    TRIG_FSPLICE_REQ=5503
    TRIG_IS_SUBPROC_MSG=5510
    TRIG_ACK=5550
    TRIG_NACK=5551
    TRIG_STATUS_ERROR=-1
    TRIG_STATUS_REFUSED=0
    TRIG_STATUS_ACCEPTED=1

    is_settings_window_visible=False
    init_complete=False
    ac_status_verified=False

    # AC_STATUS
    AC_OFF=0
    AC_REPLAY=1
    AC_LIVE=2
    AC_PAUSE=3

    # AC_SESSION_TYPE
    AC_UNKNOWN=-1
    AC_PRACTICE=0
    AC_QUALIFY=1
    AC_RACE=2
    AC_HOTLAP=3
    AC_TIME_ATTACK=4
    AC_DRIFT=5
    AC_DRAG=6

# The acti object
acti = Cacti()




def acMain(ac_version):
    global acti
    appWindow = ac.newApp("ROS_telem")
    ac.setSize(appWindow, 200, 200)
    try:
        acti.udp2_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        acti.udp2_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        acti.udp2_socket.settimeout(acti.SOCKET_TIMEOUT)
    except Exception as e:
        ac.console("Error binding to selected port")
    acti.init_complete = True
    return "ROS_telem"

def acUpdate(deltaT):
    global acti
    
    try:
        sim_info_obj = sim_info.SimInfo()
        ACTI_CarInfo = ""

        PackingString = "<"; PackingList = []
        # XXXXX "Python" Channels XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
        PackingString += "l"; PackingList.append(ac.getCarState(0, acsys.CS.LapTime))

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.WorldPosition)[0])
        
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.WorldPosition)[1])
        
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.WorldPosition)[2])
              
        PackingString += "f"; PackingList.append(float(sim_info_obj.physics.heading))

        PackingString += "f"; PackingList.append(float(sim_info_obj.physics.pitch))

        PackingString += "f"; PackingList.append(float(sim_info_obj.physics.roll))

        PackingString += "l"; PackingList.append(ac.getCarState(0, acsys.CS.Gear)-1)

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalAngularVelocity)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalAngularVelocity)[1])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalAngularVelocity)[2])

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalVelocity)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalVelocity)[1])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalVelocity)[2])
        
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.SpeedMPH))
        
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.WheelAngularSpeed)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.WheelAngularSpeed)[1])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.WheelAngularSpeed)[2])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.WheelAngularSpeed)[3])
        
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.Gas))
        
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.Brake))
        
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.Steer))
        
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.RPM))
        
        
        '''

        PackingString += "l"; PackingList.append(ac.getCarState(0, acsys.CS.LapInvalidated))

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.SuspensionTravel)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.SuspensionTravel)[1])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.SuspensionTravel)[2])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.SuspensionTravel)[3])

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.RideHeight)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.RideHeight)[1])

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.Caster))

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.ToeInDeg, 0))
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.ToeInDeg, 1))
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.ToeInDeg, 2))
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.ToeInDeg, 3))

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalAngularVelocity)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalAngularVelocity)[1])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalAngularVelocity)[2])

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalVelocity)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalVelocity)[1])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.LocalVelocity)[2])

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.CurrentTyresCoreTemp)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.CurrentTyresCoreTemp)[1])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.CurrentTyresCoreTemp)[2])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.CurrentTyresCoreTemp)[3])

        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.DynamicPressure)[0])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.DynamicPressure)[1])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.DynamicPressure)[2])
        PackingString += "f"; PackingList.append(ac.getCarState(0, acsys.CS.DynamicPressure)[3])

        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreWear))[0])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreWear))[1])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreWear))[2])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreWear))[3])

        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.brakeTemp))[0])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.brakeTemp))[1])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.brakeTemp))[2])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.brakeTemp))[3])

        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempI))[0])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempI))[1])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempI))[2])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempI))[3])

        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempM))[0])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempM))[1])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempM))[2])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempM))[3])

        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempO))[0])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempO))[1])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempO))[2])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.tyreTempO))[3])

        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.carDamage))[0])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.carDamage))[1])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.carDamage))[2])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.carDamage))[3])
        PackingString += "f"; PackingList.append(list(map(float, sim_info_obj.physics.carDamage))[4])
        '''
        # XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
        #ac.console(PackingString)
        sim_info_obj.close()
        ACTI_CarInfo = struct.pack(PackingString, *PackingList)
        acti.udp2_socket.sendto(ACTI_CarInfo, (acti.udp2_dataflow_ip, acti.ACTI_UDP2_PORT))

    except Exception as e:
        ac.console("Error sending packet")
        ac.console(str(e))

def acShutdown(*args):
    global acti
    acti.udp2_socket.close()

