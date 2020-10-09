import numpy as np
import time
import argparse
from ur_control import Robot
from transform3d import Transform
from transform3d import Transform
from gripper import get_gripper

r_a = RTDEControlInterface("192.168.1.129")
r_a.ctrl.moveL(r_a.base_t_tcp() @ Transform(rotvec=(0, 0, 0)))

grasp_start_widths = {
    'big_round_peg': 25,
    'small_round_peg': 17,
    'big_gear': 40,
    'small_gear': 26,
    'bnc': 24,
    'belt': 30,
    'small_square_peg': 14,
    'big_square_peg': 22,
    'ethernet_cable_head': 22,
}

def disassembly_place_belt(self, obj: Object, fingers: Tool):
    pick_frame = obj.find_frame("pick")
    if fingers is self._fingers_1:
        tool_frame = fingers.find_frame("grasp")
    elif fingers is self._fingers_6:
        tool_frame = fingers.find_frame("grasp_inner")
    else:
        raise Exception("Expected use of fingers 1 or 6.")


    taskboard_frame = self._taskboard.find_frame(obj.get_name())

    gripper = get_gripper()
    gripper.open()

    q_safe = (2.8662071228027344, -1.7563158474364222, -1.9528794288635254,
              -1.0198443692973633, -4.752078358327047, -2.1280840078936976)

    r.ctrl.moveJ(q_safe)
    pulley_frame = [0, 0, 0, 0, 0, 0]

    big_pulley = self._taskboard.find_frame("big_pulley")
    small_pulley = self._taskboard.find_frame("small_pulley")

    #align Z (rotation) , go to belt and pick the belt () ??

    pick = PickObject(self._robot_a_control, self._robot_a_receive, self._gripper_a, fingers, tool_frame, obj,
                      pick_frame, 0.01, 0.002, self._persistence)

    # Pick
    gripper.move(grasp_start_widths[obj_name], 255, 255)
    pick.pick(0.2)

    # Small pulley, position x y z , rotation Z
    r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))

    # go to home
    r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))

    # go to kit
    r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))

    # Release
    gripper.move(0, 255, 255)


    release_width = pick.get_release_width()
    self._gripper_a.release(release_width, 255, 255)
    self._robot_a_control.move_home(self._speed_a, self._acc_a)
