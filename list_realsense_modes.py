"""List all supported resolutions and framerates for RealSense D435."""

import pyrealsense2 as rs


def list_supported_modes():
    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("No RealSense devices found!")
        return

    for device in devices:
        print(f"\nDevice: {device.get_info(rs.camera_info.name)}")
        print(f"Serial: {device.get_info(rs.camera_info.serial_number)}")
        print(
            f"Firmware: {device.get_info(rs.camera_info.firmware_version)}\n")

        for sensor in device.query_sensors():
            print(f"Sensor: {sensor.get_info(rs.camera_info.name)}")

            # Group by stream type
            stream_profiles = {}
            for profile in sensor.get_stream_profiles():
                if profile.is_video_stream_profile():
                    vp = profile.as_video_stream_profile()
                    stream_type = str(profile.stream_type()).split('.')[-1]

                    if stream_type not in stream_profiles:
                        stream_profiles[stream_type] = []

                    stream_profiles[stream_type].append({
                        'width': vp.width(),
                        'height': vp.height(),
                        'fps': profile.fps(),
                        'format': str(profile.format()).split('.')[-1]
                    })

            # Print organized by stream type
            for stream_type, profiles in sorted(stream_profiles.items()):
                print(f"\n  {stream_type} stream:")

                # Group by resolution
                res_dict = {}
                for p in profiles:
                    res_key = f"{p['width']}x{p['height']}"
                    if res_key not in res_dict:
                        res_dict[res_key] = []
                    res_dict[res_key].append(f"{p['fps']}fps ({p['format']})")

                for res, fps_list in sorted(res_dict.items()):
                    print(f"    {res}: {', '.join(sorted(set(fps_list)))}")

            print()


if __name__ == "__main__":
    try:
        list_supported_modes()
    except Exception as e:
        print(f"Error: {e}")
        print("\nMake sure:")
        print("1. RealSense D435 is connected")
        print("2. You have permission to access USB devices")
        print("3. pyrealsense2 is installed: pip install pyrealsense2")
