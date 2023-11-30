import asyncio
import json
import zmq
from zmq.asyncio import Context
import time
from dynamixel_sdk import *
import numpy as np
import threading

import os, sys

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(sys.stdin.fileno())
    def getch():
        return sys.stdin.read(1)

ctx = Context.instance()

# Control table address
ADDR_MX_TORQUE_ENABLE     = 24       # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION     = 30
ADDR_GOAL_VELOCITY        = 32
ADDR_MX_PRESENT_POSITION  = 36
ADDR_MX_PRESENT_SPEED     = 38

# Protocol version
PROTOCOL_VERSION    = 1.0       # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE    = 57600     # Dynamixel default baudrate : 57600
DEVICENAME  = '/dev/ttyUSB0'    # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE       = 1 # Value for enabling the torque
TORQUE_DISABLE      = 0 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023      # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20# Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE] # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result1, dxl_error1 = packetHandler.write1ByteTxRx(portHandler, 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result2, dxl_error2 = packetHandler.write1ByteTxRx(portHandler, 2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result3, dxl_error3 = packetHandler.write1ByteTxRx(portHandler, 3, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result4, dxl_error4 = packetHandler.write1ByteTxRx(portHandler, 4, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result5, dxl_error5 = packetHandler.write1ByteTxRx(portHandler, 5, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result6, dxl_error6 = packetHandler.write1ByteTxRx(portHandler, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result7, dxl_error7 = packetHandler.write1ByteTxRx(portHandler, 7, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result8, dxl_error8 = packetHandler.write1ByteTxRx(portHandler, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

mode = 0 # 0:stop, 1:forward, 2:back, 3:left, 4:right

GoalVelocities = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear
GoalAngles = np.array([0,0,0,0], float)
vel_servo = np.array([0,0,0,0], int)

async def receive():
    global mode
    print("Receiving")
    socket = ctx.socket(zmq.ROUTER)
    socket.bind("tcp://*:5556")

    while True:
        req = await socket.recv_multipart()
        data = json.loads(req[1].decode())
        controls = data['controls']
        print(f"Robot received {controls}")

        if('start' not in controls and 'stop' not in controls):
            if(controls['forward'] == 1):
                print("moving_forward")
                mode = 1
            elif(controls['back'] == 1):
                print("moving_back")
                mode = 2
            elif(controls['riht'] == 1):
                print("moving_right")
                mode = 3
            elif(controls['left'] == 1):
                print("moving_left")
                mode = 4


def communication():
    asyncio.run(
        asyncio.wait(
            [
                receive(),
            ]
        )
    )

def robot_control():
    global mode
    moving = False
    moving_cnt = 0
    rotation_vel = 350
    straight_vel = 1000
    angle = 68.32
    C = 1023
    cnt_lim = 20

    while True:

        if(moving == False and mode != 0):
            moving = True
            if(mode == 1):
                print("moving_forward")
                vel_servo[0] = C + straight_vel 
                vel_servo[1] = straight_vel 
                vel_servo[2] = straight_vel 
                vel_servo[3] = C + straight_vel 

                GoalAngles[:] = 0
            elif(mode == 2):
                print("moving_back")
                vel_servo[0] = straight_vel 
                vel_servo[1] = C + straight_vel 
                vel_servo[2] = C + straight_vel 
                vel_servo[3] = straight_vel 

                GoalAngles[:] = 0
            elif(mode == 3):
                print("moving_right")
                GoalAngles[0] = -angle
                GoalAngles[1] = angle
                GoalAngles[2] = -angle
                GoalAngles[3] = angle
            
                vel_servo[0] = rotation_vel
                vel_servo[1] = rotation_vel
                vel_servo[2] = rotation_vel
                vel_servo[3] = rotation_vel
            elif(mode == 4):
                print("moving_left")
                GoalAngles[0] = -angle
                GoalAngles[1] = angle
                GoalAngles[2] = -angle
                GoalAngles[3] = angle
            
                vel_servo[0] = C + rotation_vel
                vel_servo[1] = C + rotation_vel
                vel_servo[2] = C + rotation_vel
                vel_servo[3] = C + rotation_vel
            else:
                GoalAngles[:] = 0
                vel_servo[:] = 0
                print("stoping the robot")        

        if(moving):
            moving_cnt += 1
            if(moving_cnt == cnt_lim):
                moving = False
                mode = 0
                moving_cnt = 0
                GoalAngles[:] = 0
                vel_servo[:] = 0
        
        dxl_comm_result1, dxl_error1 = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_GOAL_POSITION, int(512*(GoalAngles[0]/150 +1)))
        dxl_comm_result2, dxl_error2 = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_GOAL_POSITION, int(512*(GoalAngles[1]/150 +1)))
        dxl_comm_result3, dxl_error3 = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_GOAL_POSITION, int(512*(GoalAngles[2]/150 +1)))
        dxl_comm_result4, dxl_error4 = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_GOAL_POSITION, int(512*(GoalAngles[3]/150 +1)))
        dxl_comm_result5, dxl_error5 = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_VELOCITY, vel_servo[0])
        dxl_comm_result6, dxl_error6 = packetHandler.write2ByteTxRx(portHandler, 6, ADDR_GOAL_VELOCITY, vel_servo[1])
        dxl_comm_result7, dxl_error7 = packetHandler.write2ByteTxRx(portHandler, 7, ADDR_GOAL_VELOCITY, vel_servo[2])
        dxl_comm_result8, dxl_error8 = packetHandler.write2ByteTxRx(portHandler, 8, ADDR_GOAL_VELOCITY, vel_servo[3])

        time.sleep(0.025)

thread1 = threading.Thread(target = communication)
thread2 = threading.Thread(target = robot_control)

#thread1.daemon = True # die when the main thread dies
#thread2.daemon = True 

thread1.start() #serial recive thread
thread2.start() #joystic animation thread
