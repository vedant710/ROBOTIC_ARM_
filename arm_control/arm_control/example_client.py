#!/usr/bin/env python3
"""Demo: joint-space move -> switch -> TCP waypoints -> show that wrong-mode commands are ignored."""
from arm_control.client import ArmClient

BOX_GREEN = (0.50, -0.10)     # box positions in the arm base frame (table top = z 0)
BOX_RED = (0.40, 0.15)
DOWN = (1.0, 0.0, 0.0, 0.0)   # tool pointing down


def main():
    arm = ArmClient()

    print("mode ->", arm.set_mode("joint"))
    print("go_home:", arm.go_home())
    print("joint move:", arm.move_joints([-2.0, -1.0, -1.3, 1.7, -1.0, 1.6, 2.6]))

    print("mode ->", arm.set_mode("tcp"))
    print("start pose:", arm.move_tcp([0.35, 0.0, 0.30], DOWN))
    print("over green box:", arm.move_tcp([BOX_GREEN[0], BOX_GREEN[1], 0.18], DOWN))
    print("path:", arm.move_tcp_path([
        ([BOX_GREEN[0], BOX_GREEN[1], 0.10], DOWN),
        ([BOX_GREEN[0], BOX_GREEN[1], 0.18], DOWN),
        ([BOX_RED[0], BOX_RED[1], 0.18], DOWN),
    ]))

    # wrong-mode command: ignored
    print("joint cmd while in tcp mode ->", arm.move_joints([0.0] * 7))

    arm.close()


if __name__ == "__main__":
    main()
