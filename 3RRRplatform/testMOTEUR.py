import os, sys
import time

from dynamixel_sdk import *  # import the SDK functions / definitions

# Control table addresses for an MX series (Protocol 1.0)
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Other settings
DXL_ID = 1                # ID of your servo
BAUDRATE = 1000000         # Communication speed
DEVICENAME = "COM7" 
PROTOCOL_VERSION = 1.0     # MX-28 uses Protocol 1.0 in many examples

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_POSITION = 4095        # for MX series (0–4095)
MIN_POSITION = 0

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort() is False:
    print("Failed to open the port")
    sys.exit(1)

# Set baud rate
if portHandler.setBaudRate(BAUDRATE) is False:
    print("Failed to set baud rate")
    sys.exit(1)

# Enable torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Torque enabled")

# Move to goal position
goal_pos = 2048  # midpoint
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
    portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_pos)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Optional: loop and read current position until it reaches the goal
while True:
    dxl_present_pos, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        break
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        break

    print("Present pos: %d" % dxl_present_pos)
    if abs(goal_pos - dxl_present_pos) < 10:
        break

    time.sleep(0.1)

# Disable torque
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

# Close port
portHandler.closePort()


