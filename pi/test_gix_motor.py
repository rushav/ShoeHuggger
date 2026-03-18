#!/usr/bin/env python3
"""
GIX Motor Test Utility
=======================
Simple test script for verifying DYNAMIXEL motor communication via OpenCR.
Tests wheel motors (ID 1, 2) and wing motor (ID 3) individually.

Usage:
    python3 test_gix_motor.py

Requires:
    pip3 install dynamixel-sdk
"""

import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler

# Configuration
DEVICE_PORT      = '/dev/ttyACM0'
BAUDRATE         = 1000000
PROTOCOL_VERSION = 2.0

# Motor IDs
MOTOR_LEFT  = 1
MOTOR_RIGHT = 2
MOTOR_WING  = 3

# Control table addresses
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_VELOCITY    = 104

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0


def main():
    port = PortHandler(DEVICE_PORT)
    pkt = PacketHandler(PROTOCOL_VERSION)

    if not port.openPort():
        print(f'ERROR: Cannot open {DEVICE_PORT}')
        sys.exit(1)
    if not port.setBaudRate(BAUDRATE):
        print(f'ERROR: Cannot set baudrate {BAUDRATE}')
        sys.exit(1)

    print(f'Connected to {DEVICE_PORT} at {BAUDRATE} baud')
    print()

    # Ping all motors
    for motor_id, name in [(MOTOR_LEFT, 'Left Wheel'),
                            (MOTOR_RIGHT, 'Right Wheel'),
                            (MOTOR_WING, 'Wing')]:
        model, result, error = pkt.ping(port, motor_id)
        if result == 0:
            print(f'  Motor {motor_id} ({name}): OK (model: {model})')
        else:
            print(f'  Motor {motor_id} ({name}): NOT FOUND')

    print()
    input('Press Enter to test wing motor (ID 3)...')

    # Enable wing motor torque
    pkt.write1ByteTxRx(port, MOTOR_WING, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    # Move wing to a few positions
    positions = [1500, 2048, 2500, 2048]
    for pos in positions:
        print(f'  Moving wing to position {pos}...')
        pkt.write4ByteTxRx(port, MOTOR_WING, ADDR_GOAL_POSITION, pos)
        time.sleep(1.0)

        curr_pos, _, _ = pkt.read4ByteTxRx(port, MOTOR_WING, ADDR_PRESENT_POSITION)
        print(f'  Current position: {curr_pos}')

    # Disable torque
    pkt.write1ByteTxRx(port, MOTOR_WING, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    print()
    print('Wing motor test complete.')

    port.closePort()


if __name__ == '__main__':
    main()
