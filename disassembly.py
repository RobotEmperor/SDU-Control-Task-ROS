import numpy as np
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from transform3d import Transform

r_a = RTDEControlInterface("192.168.1.129")
r_a.ctrl.moveL(r_a.base_t_tcp() @ Transform(rotvec=(0, 0, 0)))

pulley_frame = [0, 0, 0, 0, 0, 0]

big_pulley = self._taskboard.find_frame("big_pulley")
small_pulley = self._taskboard.find_frame("small_pulley")




def disassembly_place_belt(self, obj: Object, fingers: Tool):
    pick_frame = obj.find_frame("pick")
    if fingers is self._fingers_1:
        tool_frame = fingers.find_frame("grasp")
    elif fingers is self._fingers_6:
        tool_frame = fingers.find_frame("grasp_inner")
    else:
        raise Exception("Expected use of fingers 1 or 6.")

    pick = PickObject(self._robot_a_control, self._robot_a_receive, self._gripper_a, fingers, tool_frame, obj,
                      pick_frame, 0.01, 0.002, self._persistence)

    taskboard_frame = self._taskboard.find_frame(obj.get_name())


    self._robot_a_control.move_home(self._speed_a, self._acc_a)
    self.check_frame(obj, self._kitlayout.find_frame(obj.get_name()))
    pick.pick(0.2)

    release_width = pick.get_release_width()
    self._gripper_a.release(release_width, 255, 255)
    self._robot_a_control.move_home(self._speed_a, self._acc_a)
