import asyncio
import json
from zmq.asyncio import Context
import zmq
import numpy as np
import serial
import struct

MOTOR_CONTROLLER_MACHINE_ID = 183
USER_MACHINE_ID             = 184
ID                          = 1
PID_PNT_VEL_CMD             = 207
PID_PNT_MAIN_TOTAL_DATA_NUM = 24 
PID_MAIN_DATA               = 193
ENABLE                      = 1  
RETURN_PNT_MAIN_DATA        = 2
MAX_PACKET_SIZE             = 255

LEFT                        = 0
RIGHT                       = 1
VELOCITY_CONSTANT_VALUE     = 9.5492743

def byte2int(low_byte, high_byte):
    return low_byte | high_byte << 8

def byte2long(data1, data2, data3, data4):
    return data1 | data2 << 8 | data3 << 16 | data4 << 24

class IByte:
    def __init__(self, nIn):
        self.byLow = nIn & 0xff
        self.byHigh = (nIn >> 8) & 0xff

def int2byte(nIn):
    return IByte(nIn)

ctx = Context.instance()
socket = None

async def receive(socket):
    try:
        req = await socket.recv_multipart()
    except zmq.ZMQError:
        print("Connection lost.")
        raise ConnectionError("Connection lost.")
    print(f"Robot received {req[1]}")
    return req[1]

async def send(socket):
    print("Sending")

    ser = serial.Serial(
        port = '/dev/ttyUSB0',
        baudrate = 19200,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS
    )

    if ser.isOpen():
        ser.close()
    ser.open()

    try:
        telemetry = json.loads(await receive(socket))
        goal_rpm_speed = await process_telemetry(telemetry)

        nLeftRPM = goal_rpm_speed[0]
        nRightRPM = goal_rpm_speed[1]

        put_pnt_vel_cmd(ser, nLeftRPM, nRightRPM)
    except ConnectionError:
        print("Stopping motor due to lost Connection")
        ser.close()
        
    ser.close()

async def process_telemetry(telemetry):
    controls = telemetry.get("controls", {})
    w = controls.get("forward", 0)
    s = controls.get("Backward", 0)
    a = controls.get("Left", 0)
    d = controls.get("Right", 0)
    print(w,s,a,d)


    if w == 1 and a == 0 and d == 0:
        linear = 5.5
        angular = 0
    elif s == 1 and a == 0 and d == 0:
        linear = -5.5
        angular = 0
    elif a == 1 and w == 0 and s == 0:
        linear = 0
        angular = -5.5
    elif d == 1 and w == 0 and s == 0:
        linear = 0
        angular = 5.5
    elif w == 1 and a == 1:
        linear = 2.5
        angular = -3.0
    elif w == 1 and d == 1:
        linear = 2.5
        angular = 3.0
    elif s == 1 and a == 1:
        linear = -2.5
        angular = -3.0
    elif s == 1 and d == 1:
        linear = -2.5
        angular = 3.0
    else:
        linear = 0
        angular = 0

    print("linear:", linear, "angular:", angular)
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
    byD = bytearray(MAX_PACKET_SIZE)
    byDataNum = 7

    byD[0] = MOTOR_CONTROLLER_MACHINE_ID
    byD[1] = USER_MACHINE_ID
    byD[2] = ID
    byD[3] = PID_PNT_VEL_CMD
    byD[4] = byDataNum
    byD[5] = ENABLE
    left = int2byte(left_rpm)
    byD[6], byD[7] = left.byLow, left.byHigh
    byD[8] = ENABLE
    right = int2byte(right_rpm)
    byD[9], byD[10] = right.byLow, right.byHigh
    byD[11] = RETURN_PNT_MAIN_DATA
    byChkSum = sum(byD[0:12]) & 0xFF
    byD[12] = (byChkSum ^ 0xFF) + 1 & 0xFF

    for byte in byD:
        ser.write(struct.pack('B', byte))

async def main():
    global socket
    socket = ctx.socket(zmq.ROUTER)
    socket.bind("tcp://*:5556")

    print("Receiving")

    while True:
        await send(socket)

if __name__ == "__main__":
    asyncio.run(main())

