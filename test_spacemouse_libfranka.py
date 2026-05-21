"""
SpaceMouse -> franka_server TCP client
Uses official Franka Robotics axis mapping from franka_spacemouse repo.
No buttons  = XY translation (Columbia style)
Left button = rotation only
Right button = unlock Z
"""

import time
import socket
import json
import numpy as np
import pyspacemouse
from multiprocessing.managers import SharedMemoryManager

SERVER_IP   = '192.168.18.1'
SERVER_PORT = 4242

MAX_POS_SPEED = 0.15   # m/s
MAX_ROT_SPEED = 0.15   # rad/s
OPERATOR_FRONT = True  # True if sitting in front of robot

def main():
    print(f'Connecting to franka_server at {SERVER_IP}:{SERVER_PORT}...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print('Connected!')

    device = pyspacemouse.open()
    if not device:
        raise RuntimeError('SpaceMouse not found')
    print('SpaceMouse ready')
    print('No buttons   = XY translation')
    print('Left button  = rotation only')
    print('Right button = unlock Z')
    print('Ctrl+C to stop')

    def send_vel(vx, vy, vz, wx, wy, wz):
        msg = json.dumps({
            'vx': float(vx), 'vy': float(vy), 'vz': float(vz),
            'wx': float(wx), 'wy': float(wy), 'wz': float(wz),
            't': time.time()
        }) + '\n'
        sock.sendall(msg.encode())

    try:
        while True:
            t_start = time.monotonic()

            state = device.read()

            # official Franka axis mapping from franka_spacemouse repo
            vx = -float(state.y) * MAX_POS_SPEED
            vy =  float(state.x) * MAX_POS_SPEED
            vz =  float(state.z) * MAX_POS_SPEED
            wx = -float(state.roll)  * MAX_ROT_SPEED
            wy = -float(state.pitch) * MAX_ROT_SPEED
            wz = -float(state.yaw)   * MAX_ROT_SPEED

            # flip if operator is not in front
            if not OPERATOR_FRONT:
                vx *= -1
                vy *= -1
                wx *= -1
                wy *= -1

            buttons = state.buttons
            btn_left  = bool(buttons[0]) if len(buttons) > 0 else False
            btn_right = bool(buttons[1]) if len(buttons) > 1 else False

            # Columbia-style button mapping
            if not btn_left:
                # no left button = translation mode, zero rotation
                wx = wy = wz = 0.0
            else:
                # left button = rotation mode, zero translation
                vx = vy = vz = 0.0

            if not btn_right:
                # no right button = 2D mode, zero Z
                vz = 0.0

            send_vel(vx, vy, vz, wx, wy, wz)

            if any(abs(v) > 0.001 for v in [vx, vy, vz, wx, wy, wz]):
                print(
                    f'lin: [{vx:+.3f} {vy:+.3f} {vz:+.3f}]  '
                    f'ang: [{wx:+.3f} {wy:+.3f} {wz:+.3f}]  '
                    f'btn: [{btn_left} {btn_right}]'
                )

            elapsed = time.monotonic() - t_start
            sleep_time = 0.01 - elapsed  # 100Hz
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        try:
            send_vel(0, 0, 0, 0, 0, 0)
        except:
            pass
        device.close()
        sock.close()
        print('Stopped')

if __name__ == '__main__':
    main()
