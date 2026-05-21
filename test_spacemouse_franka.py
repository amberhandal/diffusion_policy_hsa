"""
Minimal test: SpaceMouse controls Franka arm via ROS2 cartesian velocity.

Press Ctrl+C to stop.
"""

import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from diffusion_policy.real_world.spacemouse_shared_memory import Spacemouse
from multiprocessing.managers import SharedMemoryManager

# tuning parameters
LINEAR_SCALE  = 0.15   # m/s per unit spacemouse input
ANGULAR_SCALE = 0.3    # rad/s per unit spacemouse input
DEADZONE      = 0.05   # ignore inputs below this magnitude

def main():
    # init ROS2
    rclpy.init()
    node = rclpy.create_node('spacemouse_franka_test')

    # publisher to cartesian velocity controller
    pub = node.create_publisher(
        TwistStamped,
        '/cartesian_velocity_controller/cmd_vel',
        10
    )

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('ROS2 initialized')
    print('Starting SpaceMouse...')

    with SharedMemoryManager() as shm_manager:
        with Spacemouse(shm_manager=shm_manager) as sm:
            print('SpaceMouse ready')
            print('Move the puck to move the arm')
            print('Left button  = rotation mode (translation disabled)')
            print('Right button = enable Z axis (disabled by default)')
            print('Ctrl+C to stop')

            try:
                while True:
                    # get spacemouse state in robot frame
                    motion = sm.get_motion_state_transformed()
                    buttons = sm.get_button_state()

                    dpos = motion[:3].copy()
                    drot = motion[3:].copy()

                    # left button = rotation only mode
                    if buttons[0]:
                        dpos[:] = 0
                    else:
                        drot[:] = 0

                    # right button unlocks Z
                    if not buttons[1]:
                        dpos[2] = 0

                    # apply deadzone
                    dpos[np.abs(dpos) < DEADZONE] = 0
                    drot[np.abs(drot) < DEADZONE] = 0

                    # scale
                    vel_lin = dpos * LINEAR_SCALE
                    vel_ang = drot * ANGULAR_SCALE

                    # publish
                    cmd = TwistStamped()
                    cmd.header.stamp = node.get_clock().now().to_msg()
                    cmd.header.frame_id = 'fer_link0'
                    cmd.twist.linear.x  = float(vel_lin[0])
                    cmd.twist.linear.y  = float(vel_lin[1])
                    cmd.twist.linear.z  = float(vel_lin[2])
                    cmd.twist.angular.x = float(vel_ang[0])
                    cmd.twist.angular.y = float(vel_ang[1])
                    cmd.twist.angular.z = float(vel_ang[2])
                    pub.publish(cmd)

                    # print non-zero motion for feedback
                    if np.any(np.abs(vel_lin) > 0.001) or np.any(np.abs(vel_ang) > 0.001):
                        print(f'lin: [{vel_lin[0]:+.3f} {vel_lin[1]:+.3f} {vel_lin[2]:+.3f}]  '
                              f'ang: [{vel_ang[0]:+.3f} {vel_ang[1]:+.3f} {vel_ang[2]:+.3f}]  '
                              f'buttons: {buttons}')

                    time.sleep(0.05)  # 20Hz

            except KeyboardInterrupt:
                print('\nStopping...')

            finally:
                # publish zero velocity to stop arm
                stop = TwistStamped()
                stop.header.stamp = node.get_clock().now().to_msg()
                pub.publish(stop)
                print('Zero velocity sent, arm stopped')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
