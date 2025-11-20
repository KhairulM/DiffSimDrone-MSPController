#!/usr/bin/env python3

from pymultiwii import MultiWii
import time
import struct

serial_port = '/dev/ttyACM0'

try:
    board = MultiWii(serial_port)
    print(f"Connected to flight controller on {serial_port}")

    # Wait a moment for connection to stabilize
    time.sleep(1)

    # Test attitude reading with debugging
    print("Testing attitude data reading...")

    try:
        # Send attitude request
        print("Sending attitude request...")
        board.sendCMD(0, MultiWii.ATTITUDE, [], '')

        # Read response manually with debugging
        print("Reading response header...")
        while True:
            header = board.ser.read().decode('utf-8')
            if header == '$':
                header = header + board.ser.read(2).decode('utf-8')
                print(f"Header: {header}")
                break

        datalength = struct.unpack('<b', board.ser.read())[0]
        print(f"Data length: {datalength}")

        code = struct.unpack('<b', board.ser.read())[0]
        print(f"Response code: {code}")

        data = board.ser.read(datalength)
        print(f"Raw data: {data}")
        print(f"Data bytes: {[hex(b) for b in data]}")

        if datalength >= 6:  # Attitude should be 6 bytes (3 shorts)
            temp = struct.unpack('<hhh', data[:6])
            print(f"Unpacked data: {temp}")
            print(f"Roll: {temp[0]/10.0}°")
            print(f"Pitch: {temp[1]/10.0}°")
            print(f"Heading: {temp[2]}°")
        else:
            print(f"Insufficient data length: {datalength}")

        board.ser.flushInput()
        board.ser.flushOutput()

    except Exception as e:
        print(f"Error in manual attitude reading: {e}")
        import traceback
        traceback.print_exc()

    # Now test the library method
    print("\nTesting library getData method...")
    try:
        attitude = board.getData(MultiWii.ATTITUDE)
        print(f"Attitude data: {attitude}")
    except Exception as e:
        print(f"Error with getData: {e}")
        import traceback
        traceback.print_exc()

except Exception as error:
    print(f"Connection error: {error}")
