from pymultiwii import MultiWii
import time
import struct

serial_port = '/dev/ttyACM0'  # Replace with your serial port

try:
    board = MultiWii(serial_port)
    print(f"Connected to flight controller on {serial_port}")

    # Get API version from flight controller
    print("Requesting API version...")
    try:
        # Send API_VERSION command (command 1, no data)
        board.sendCMD(0, MultiWii.API_VERSION, [], '')

        # Read response
        while True:
            header = board.ser.read().decode('utf-8')
            if header == '$':
                header = header + board.ser.read(2).decode('utf-8')
                break

        datalength = struct.unpack('<b', board.ser.read())[0]
        code = struct.unpack('<b', board.ser.read())
        data = board.ser.read(datalength)

        if datalength >= 3:
            # API version format: major, minor, patch
            version_data = struct.unpack('<BBB', data[:3])
            print(
                f"API Version: {version_data[0]}.{version_data[1]}.{version_data[2]}")
        else:
            print(f"API Version response: {data}")

        board.ser.flushInput()
        board.ser.flushOutput()

    except Exception as e:
        print(f"Could not get API version: {e}")

    # Custom arm function that includes the missing data format parameter
    def arm_drone():
        timer = 0
        start = time.time()
        print("Sending arm commands...")
        while timer < 0.5:
            # Roll, Pitch, Yaw high, Throttle low
            data = [1500, 1500, 2000, 1000]
            board.sendCMD(8, MultiWii.SET_RAW_RC, data, '4H')
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start = time.time()

    # Arm the drone
    arm_drone()
    print("Arming sequence completed. Motors should be armed now.")

    # Optional: Check if armed by reading some data
    # try:
    #     print("Reading attitude data...")
    #     attitude = board.getData(MultiWii.ATTITUDE)
    #     if attitude:
    #         print(
    #             f"Flight controller responding - Roll: {attitude['angx']:.2f}°, Pitch: {attitude['angy']:.2f}°")
    #     else:
    #         print("No attitude data received")
    # except Exception as e:
    #     print(f"Could not read attitude data: {e}")
    #     print("But arming commands were sent successfully.")

except Exception as error:
    print(f"Error: {error}")
    print("Make sure the flight controller is connected and the port is correct.")
