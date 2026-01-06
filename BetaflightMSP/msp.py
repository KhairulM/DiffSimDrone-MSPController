import struct
import serial


class MSP:
    def __init__(self, serial_port):
        self.serial_port = serial_port

    def connect(self):
        self.serial_in = serial.Serial(

        )

    def get_checksum(msp_command_id, payload):
        checksum = 0
        length = len(payload)

        for byte in bytes([length, msp_command_id]) + payload:
            checksum ^= byte

        checksum &= 0xFF
        return checksum

    def send_msp_command(serial_port, msp_command_id, data):
        payload = bytearray()
        for value in data:
            payload += struct.pack('<1H', value)

        header = b'$M<'
        length = len(payload)
        checksum = get_checksum(msp_command_id, payload)

        msp_package = header + \
            bytes([length, msp_command_id]) + payload + bytes([checksum])
        serial_port.write(msp_package)

    def send_msp_request(serial_port, msp_command_id):
        header = b'$M<'
        length = 0
        checksum = get_checksum(msp_command_id, bytes([]))

        msp_package = header + \
            struct.pack('<BB', length, msp_command_id) + bytes([checksum])
        serial_port.write(msp_package)

    def read_msp_response(serial_port):
        response = serial_port.readline()
        if response.startswith(b'$M>'):
            length = response[3]
            msp_command_id = response[4]
            payload = response[5:5 + length]
            return msp_command_id, payload
        else:
            raise ValueError("Invalid MSP response")
