#!/usr/bin/env python3

import argparse
import sys

import geometry_msgs.msg
import rclpy

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


MSG = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}


def get_key(settings):
    if sys.platform == "win32":
        return msvcrt.getwch()
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return f"currently:\tspeed {speed}\tturn {turn} "


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--turn", type=float, default=1.0)
    parser.add_argument("--topic", default="/diff_drive_controller/cmd_vel_unstamped")
    return parser.parse_args()


def main():
    args = parse_args()
    settings = save_terminal_settings()

    rclpy.init()
    node = rclpy.create_node("amr_teleop_keyboard")
    pub = node.create_publisher(geometry_msgs.msg.Twist, args.topic, 10)

    speed = args.speed
    turn = args.turn
    x = y = z = th = 0.0
    status = 0

    try:
        print(MSG)
        print(vels(speed, turn))
        while True:
            key = get_key(settings)
            if key in MOVE_BINDINGS:
                x, y, z, th = MOVE_BINDINGS[key]
            elif key in SPEED_BINDINGS:
                speed *= SPEED_BINDINGS[key][0]
                turn *= SPEED_BINDINGS[key][1]
                print(vels(speed, turn))
                if status == 14:
                    print(MSG)
                status = (status + 1) % 15
            else:
                x = y = z = th = 0.0
                if key == "\x03":
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as exc:
        print(exc)
    finally:
        twist = geometry_msgs.msg.Twist()
        pub.publish(twist)
        restore_terminal_settings(settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
