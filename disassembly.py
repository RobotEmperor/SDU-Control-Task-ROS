import numpy as np
import time
import argparse
from ur_control import Robot
from transform3d import Transform
from transform3d import Transform
from gripper import get_gripper

import time
import math

from i40.algorithms.speed_until_force import SpeedUntilForce
from i40.control.gripper_2f_controller import Gripper2FController
from i40.control.robot_controller import RobotController
from i40.control.robot_ft_controller import RobotFTController
from i40.kinematics.frame import Frame
from i40.kinematics.invariance_type import InvarianceType
from i40.kinematics.kinematics import Kinematics
from i40.kinematics.movable_frame import MovableFrame
from i40.loaders.cell_saver import CellSaver
from i40.math.pose_6d import Pose6D
from i40.math.rotation_3d import Rotation3D
from i40.math.transform_3d import Transform3D
from i40.math.vector_3d import Vector3D, dot, cross, normalize
from i40.models.object import Object
from i40.models.tool import Tool
from i40.persistence.persistencedriver import PersistenceDriver
from i40.sensor.robot_ft_sensor import RobotFTSensor
from i40.sensor.robot_sensor import RobotSensor

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

def disassembly_place_belt(self, controller: RobotController, sensor: RobotSensor, gripper: Gripper2FController, obj: Object):
    self._robot_control = controller
    self._robot_receive = sensor
    self._gripper = gripper

    self._robot_model = self._robot_control.get_model()
    self.robot_end = self._robot_model.get_end()

    self.robot_end.set_transform(Pose6D(_robot_receive.getActualTCPPose()))

    gripper = get_gripper()
    gripper.open()

    q_safe = (2.8662071228027344, -1.7563158474364222, -1.9528794288635254,
              -1.0198443692973633, -4.752078358327047, -2.1280840078936976)

    #self._robot_control.ctrl.moveJ(q_safe, 0.1, 0.1)
    self.desired_frame_0 = [0, -0.07, 0.0135, 0, 0, 0]
    self.desired_frame_1 = [0, 0, -0.02, 0, 0, 0]
    self.desired_frame_2 = [0, 0, 0, 0, 0, 0] # big pulley

    self.big_pulley = obj.find_frame("big_pulley")
    self.small_pulley = obj.find_frame("small_pulley")

    self.big_pulley.set_transform(Pose6D(self.big_pulley))
    self.small_pulley.set_transform(Pose6D(self.small_pulley))

    p = self.big_pulley.p() - self.small_pulley.p()

    align_angle_z_ = math.atan2(p[0], p[1]) # y axis align

    print("align_angle_z_: ", align_angle_z_)

    t_align_angle_z_ = Transform3D([0, 0, 0], Rotation3D(0, 0, align_angle_z_))

    #align
    t_small_pulley_p = Transform3D(self.small_pulley.p(), Rotation3D(0, 0, 0))
    t_big_pulley_p = Transform3D(self.big_pulley.p(), Rotation3D(0, 0, 0))
    t_base_end_approach = self.robot_end * t_align_angle_z_
    #self._robot_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

    #pick the belt
    t_desired_frame_0 = Transform3D(self.desired_frame_0[0:3], Rotation3D(0, 0, 0))
    self.robot_end.set_transform(Pose6D(_robot_receive.getActualTCPPose()))
    t_base_end_approach = self.robot_end * t_desired_frame_0
    # self._robot_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

    # move the small pulley center
    t_desired_frame_1 = Transform3D(self.desired_frame_1[0:3], Rotation3D(0, 0, 0))
    self.robot_end.set_transform(Pose6D(_robot_receive.getActualTCPPose()))
    t_base_end_approach = self.robot_end * t_small_pulley_p * t_desired_frame_1
    # self._robot_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

    # move the big pulley center 
    #t_desired_frame_2 = Transform3D(self.desired_frame_2[0:3], Rotation3D(0, 0, 0))
    #self.robot_end.set_transform(Pose6D(_robot_receive.getActualTCPPose()))
    #t_base_end_approach = self.robot_end * t_desired_frame_2
    # self._robot_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)





    t_desired_frame_2 = Transform3D(self.desired_frame_2[0:3], Rotation3D(0, 0, 0))


    #self.t_small_big = Kinematics.frame_t_frame(self.small_pulley, self.big_pulley, Transform3D)
    #self.t_tool_align = Kinematics.frame_t_frame(self.robot_end, self.t_small_big, Transform3D)
    #r_a.ctrl.moveL(r_a.base_t_tcp() @ Transform(rotvec=(0, 0, 0)))
    #align Z (rotation) , go to belt and pick the belt () ??
    # Pick
    #gripper.move(grasp_start_widths[obj_name], 255, 255)

    # Small pulley, position x y z , rotation Z
    #r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))

    # go to home
    #r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))

    # go to kit
    #r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))

    # Release
    #gripper.move(0, 255, 255)


    #release_width = pick.get_release_width()
    #self._gripper_a.release(release_width, 255, 255)
    #self._robot_a_control.move_home(self._speed_a, self._acc_a)

    #t_end_tool = Kinematics.frame_t_frame(robot_end, self._tool_frame, Transform3D)
    #t_world_robot = self._robot_model.world_t_base(Transform3D)

    #t_world_robot = self._robot_model.world_t_base(Transform3D)

    #t_world_approach = Transform3D(t_world_object.p() - z * approach_distance, t_world_object.r())
    #t_world_above = Transform3D(t_world_object.p() - z * 0.03, t_world_object.r())
    #t_world_pick = Transform3D(t_world_object.p(), t_world_object.r())


    #t_base_end_approach = t_world_robot.inverse() * t_world_approach * t_end_tool.inverse()

if __name__=='__main__':
    persistence = WRSPersistence.driver()
    cell = CellLoader.load(persistence)
    r_a_control = RTDEControlInterface("192.168.1.129")
    r_a_receive = RTDEReceiveInterface("192.168.1.129")

    grippers = GrippersTask(cell, "taskboard_plate", persistence)
    taskboard = cell.find_object("taskboard")

    disassembly_place_belt(self, r_a_control, r_a_receive, grippers, taskboard)


    #global data_a
    #global data_b

    #data_a = 0
    #data_b = 0
    #grippers.move_grippper_a(data_a)
    #grippers.move_grippper_b(data_b)
