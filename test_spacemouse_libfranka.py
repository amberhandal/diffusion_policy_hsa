import time
import socket
import json
import numpy as np
from multiprocessing.managers import SharedMemoryManager
from diffusion_policy.real_world.spacemouse_shared_memory import Spacemouse

SERVER_IP   = '192.168.18.1'
SERVER_PORT = 4242

LINEAR_SCALE  = 0.10
ANGULAR_SCALE = 0.20
DEADZONE      = 0.05

def main():
    print(f'Connecting to franka_server at {SERVER_IP}:{SERVER_PORT}...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print('Connected!')

    def send_vel(vx, vy, vz, wx, wy, wz):
        msg = json.dumps({
            'vx': float(vx), 'vy': float(vy), 'vz': float(vz),
            'wx': float(wx), 'wy': float(wy), 'wz': float(wz),
            't': time.time()
        }) + '\n'
        sock.sendall(msg.encode())

    with SharedMemoryManager() as shm_manager:
        with Spacemouse(shm_manager=shm_manager) as sm:
            try:
                while True:
                    motion  = sm.get_motion_state_transformed()
                    buttons = sm.get_button_state()

                    dpos = motion[:3].copy()
                    drot = motion[3:].copy()

                    if buttons[0]:
                        dpos[:] = 0
                    else:
                        drot[:] = 0

                    if not buttons[1]:
                        dpos[2] = 0

                    dpos[np.abs(dpos) < DEADZONE] = 0
                    drot[np.abs(drot) < DEADZONE] = 0

                    vel_lin = dpos * LINEAR_SCALE
                    vel_ang = drot * ANGULAR_SCALE

                    send_vel(
                        vel_lin[0], vel_lin[1], vel_lin[2],
                        vel_ang[0], vel_ang[1], vel_ang[2]
                    )

                    if np.any(np.abs(vel_lin) > 0.001) or \
                       np.any(np.abs(vel_ang) > 0.001):
                        print(
                            f'lin: [{vel_lin[0]:+.3f} {vel_lin[1]:+.3f} {vel_lin[2]:+.3f}]  '
                            f'ang: [{vel_ang[0]:+.3f} {vel_ang[1]:+.3f} {vel_ang[2]:+.3f}]  '
                            f'btn: {buttons}'
                        )

                    time.sleep(0.05)

            except KeyboardInterrupt:
            finally:
                send_vel(0, 0, 0, 0, 0, 0)

    sock.close()

if __name__ == '__main__':
    main()
