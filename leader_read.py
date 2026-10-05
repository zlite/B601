"""Read-only FashionStar leader adapter. No torque, origin or mode writes."""
import argparse
import json
import math
import os
import fcntl
from pathlib import Path
import select
import struct
import sys
import termios
import time

ROOT = Path(__file__).resolve().parent
LEADER_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
NAMES = ['Base', 'Shoulder', 'Elbow', 'Wrist bend', 'Wrist yaw', 'Wrist roll', 'Gripper']
# Initial preview convention from upstream LeRobot; verify physically in the UI.
PREVIEW_SIGNS = [-1, -1, 1, 1, 1, -1]


def fresh_angles(monitors):
    values = []
    for servo_id in range(7):
        value = monitors.get(servo_id)
        if value is None or not value.reliable:
            raise RuntimeError(f'Leader servo {servo_id}: no fresh reply')
        angle = float(value.angle_deg)
        if not math.isfinite(angle):
            raise RuntimeError(f'Leader servo {servo_id}: invalid angle')
        values.append(angle)
    return values


class SdkLeaderReader:
    def __init__(self, port=LEADER_PORT):
        self.port = port

    def __enter__(self):
        sys.path.insert(0, str(ROOT / 'vendor/leader_sdk'))
        from motorbridge_smart_servo import FashionStarServo
        self.bus = FashionStarServo(self.port, baudrate=1_000_000)
        return self

    def read(self):
        return fresh_angles(self.bus.sync_monitor(list(range(7))))

    def __exit__(self, *_):
        self.bus.close()


def monitor_request(servo_id):
    if servo_id not in range(7):
        raise ValueError('Expected leader servo ID 0..6')
    # FashionStar QUERY_SERVO_MONITOR (22). No general command-writing API.
    data = bytes([0x12, 0x4c, 22, 1, servo_id])
    return data + bytes([sum(data) & 255])


def parse_monitor(buffer, servo_id):
    """Consume a full, checked reply; return None while it is incomplete."""
    start = buffer.find(b'\x05\x1c')
    if start < 0:
        if len(buffer) > 1:
            del buffer[:-1]
        return None
    del buffer[:start]
    if len(buffer) < 4:
        return None
    length = buffer[3]
    if len(buffer) < length+5:
        return None
    packet = bytes(buffer[:length+5])
    del buffer[:length+5]
    if (sum(packet[:-1]) & 255) != packet[-1]:
        raise RuntimeError('Leader reply checksum mismatch')
    if packet[2] != 22 or length not in (14,16) or packet[4] != servo_id:
        raise RuntimeError('Unexpected leader reply ID, command, or length')
    return struct.unpack_from('<i',packet,14)[0] / 10.0


class LeaderReader:
    """Linux read-only monitor transactions, with no cached-angle fallback.

    SDK 0.0.4 conflates a near-zero filter state with missing replies. Here each
    sample requires a fresh checksummed monitor response after its own query.
    Protocol reference: motorbridge/motorbridge-smart-servo,
    smart_servo_vendors/fashionstar/src/protocol.rs (QUERY_SERVO_MONITOR).
    Intended for observation only, not a motion-control feedback source.
    """
    def __init__(self, port=LEADER_PORT):
        self.port = port
        self.fd = None
        self.previous = None

    def __enter__(self):
        self.fd = os.open(self.port,os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd,fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd,termios.TIOCEXCL)
            settings = termios.tcgetattr(self.fd)
            settings[0] = settings[1] = settings[3] = 0
            settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            settings[4] = settings[5] = termios.B1000000
            settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd,termios.TCSANOW,settings)
            termios.tcflush(self.fd,termios.TCIOFLUSH)
            return self
        except BaseException:
            self.__exit__()
            raise

    def read(self):
        values=[]
        for servo_id in range(7):
            termios.tcflush(self.fd,termios.TCIFLUSH)
            request=monitor_request(servo_id)
            if os.write(self.fd,request) != len(request):
                raise RuntimeError('Incomplete leader query')
            deadline=time.monotonic()+.25
            buffer=bytearray()
            while True:
                remaining=deadline-time.monotonic()
                if remaining <= 0 or not select.select([self.fd],[],[],remaining)[0]:
                    raise RuntimeError(f'Leader servo {servo_id}: no fresh reply')
                chunk=os.read(self.fd,256)
                if not chunk:
                    raise RuntimeError('Leader serial connection closed')
                buffer.extend(chunk)
                value=parse_monitor(buffer,servo_id)
                if value is not None:
                    values.append(value)
                    break
        if self.previous is not None and max(abs(a-b) for a,b in zip(values,self.previous)) > 20:
            raise RuntimeError('Leader angle jumped over 20 degrees between reads; re-reference required')
        self.previous=values
        return values

    def __exit__(self, *_):
        if self.fd is not None:
            try:
                fcntl.ioctl(self.fd,termios.TIOCNXCL)
            finally:
                os.close(self.fd)
                self.fd=None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=LEADER_PORT)
    parser.add_argument('--sdk', action='store_true', help='Diagnostic: use optional SDK instead of fresh monitor transactions')
    args = parser.parse_args()
    with (SdkLeaderReader if args.sdk else LeaderReader)(args.port) as reader:
        print(json.dumps(dict(zip(NAMES, reader.read())), indent=2))
