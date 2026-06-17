#!/usr/bin/env python3

import sys
import rclpy
from omniblue_programing.omniblue_action import omniblue_cmd


def main():
    rclpy.init()

    if len(sys.argv) != 5:
        print("Usage:")
        print("go_to map x y yaw")
        print("go_to relative x y yaw")
        print("go_to base x y yaw")
        print("go_to global x y yaw")
        return

    mode = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    yaw = float(sys.argv[4])

    nav = omniblue_cmd()

    if mode == "map":
        nav.move_map(x, y, yaw)

    elif mode == "relative":
        nav.move_relative(x, y, yaw)

    elif mode == "base":
        nav.move_base(x, y, yaw)

    elif mode == "global":
        nav.move_global(x, y, yaw)

    else:
        print("Unknown mode:", mode)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
