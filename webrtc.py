import asyncio
import json
from zmq.asyncio import Context
import zmq
import numpy as np
import serial
import struct
import time

MOTOR_CONTROLLER_MACHINE_ID = 183
USER_MACHINE_ID             = 184
ID                          =  1
PID_PNT_VEL_CMD             = 207
PID_PNT_MAIN_TOTAL_DATA_NUM =  24 
PID_MAIN_DATA               = 193
ENABLE                      =  1  
RETURN_PNT_MAIN_DATA        = 2    

LEFT                        =   0
RIGHT                       =   1
VELOCITY_CONSTANT_VALUE     =   9.5492743

def byte2int(low_byte, high_byte):
    return low_byte | high_byte << 8

def byte2long(data1, data2, data3, data4):
    return data1 | data2 << 8 | data3 << 16 | data4 << 24

def int2byte(input_int):
    low_byte = input_int & 0xff
    high_byte = (input_int >> 8) & 0xff
    return low_byte, high_byte

ctx = Context.instance()
 
async def receive():
    print("Receiving")
    socket = ctx.socket(zmq.ROUTER)
    socket.bind("tcp://*:5556")
    req = await socket.recv_multipart()
    print(f"Robot received {req[1]}")
    return req[1]

async def send():
    print("Sending")

    ser = serial.Serial(
        port = '/dev/ttyUSB0',
        baudrate = 115200,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS
    )

    if ser.isOpen():
        ser.close()
    ser.open()

    telemetry = json.loads(await receive())
    goal_rpm_speed = await process_telemetry(telemetry)

    nLeftRPM = goal_rpm_speed[0]
    nRightRPM = goal_rpm_speed[1]

    put_pnt_vel_cmd(ser, nLeftRPM, nRightRPM)

    ser.close()

async def process_telemetry(telemetry):
    if telemetry["controls"]["w"] == 1:
        linear = 2000
        angular = 2000
    elif telemetry["controls"]["s"] == 1:
        linear = -2000
        angular = -2000
    elif telemetry["controls"]["a"] == 1:
        linear = 2000
        angular = 1500
    elif telemetry["controls"]["d"] == 1:
        linear = 1500
        angular = 2000
    else:
        linear = angular = 0

    RobotSpeedToRPMSpeed(linear, angular)
    return goal_rpm_speed
    
goal_rpm_speed = np.zeros(2, dtype = np.int16)

def RobotSpeedToRPMSpeed(linear, angular):
    wheel_velocity_cmd = np.zeros(2)

    wheel_radius = 0.085
    wheel_separation = 0.68
    reduction = 1
    nMaxRPM = 200
    VELOCITY_CONSTANT_VALUE = 1

    wheel_velocity_cmd[0] = linear + (angular * wheel_separation / 2)
    wheel_velocity_cmd[1] = linear - (angular * wheel_separation / 2)

    wheel_velocity_cmd[0] = np.clip(wheel_velocity_cmd[0] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -nMaxRPM, nMaxRPM)
    wheel_velocity_cmd[1] = np.clip(wheel_velocity_cmd[1] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -nMaxRPM, nMaxRPM)

    # For 400T
    # goal_rpm_speed[0] = np.int16(wheel_velocity_cmd[0])
    # goal_rpm_speed[1] = np.int16(-wheel_velocity_cmd[1])  # Flip the sign to change direction

    # For 200T
    goal_rpm_speed[0] = np.int16(wheel_velocity_cmd[0])
    goal_rpm_speed[1] = np.int16(wheel_velocity_cmd[1])

def put_pnt_vel_cmd(ser, left_rpm, right_rpm):

    byDataNum = 7

    packet = bytearray()
    packet.append(MOTOR_CONTROLLER_MACHINE_ID)
    packet.append(USER_MACHINE_ID)
    packet.append(ID)
    packet.append(PID_PNT_VEL_CMD)
    packet.append(byDataNum)
    packet.append(ENABLE)
    packet.extend(int2byte(left_rpm))
    packet.append(ENABLE)
    packet.extend(int2byte(right_rpm))
    packet.append(RETURN_PNT_MAIN_DATA)

    checksum = sum(packet) & 0xFF
    packet.append(~checksum + 1 & 0xFF)

    for byte in packet:
        ser.write(struct.pack('B', byte))
        time.sleep(1)

async def main():    
    while True:
        await send()

if __name__ == "__main__":
    asyncio.run(main())
