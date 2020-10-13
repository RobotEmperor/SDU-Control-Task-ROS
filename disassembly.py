    def disassembly_place_belt(self, obj: Object):
        self._robot_a_control.move_home(self._speed_a, self._acc_a)
        self._robot_model = self._robot_a_control.get_model()
        self.robot_end = self._robot_model.get_end()

        self.robot_end.set_transform(Pose6D(self._robot_a_receive.getActualTCPPose()))

        print("self._robot_a_receive.getActualTCPPose() :: ", self._robot_a_receive.getActualTCPPose())
        print("self._robot_a_receive.getActualQ() :: ", self._robot_a_receive.getActualQ())

        z_tool_ = -0.2
        self.desired_frame_0 = [0.025, 0.07, -0.0135 + z_tool_, 0, 0, 0]
        self.desired_frame_1 = [0, 0, 0.03, 0, 0, 0]
        self.desired_frame_2 = [0, -0.07, -0.035, 0, 0, 0]
        self.desired_frame_3 = [0, 0.07, 0, 0, 0, 0]  # big pulley

        big_pulley = obj.find_frame("big_pulley")
        small_pulley = obj.find_frame("small_pulley")

        base_t_big_pulley_a = Kinematics.frame_t_frame( self._robot_model.get_base(), big_pulley, Transform3D)
        base_t_small_pulley_a = Kinematics.frame_t_frame(self._robot_model.get_base(), small_pulley, Transform3D)

        #self._robot_a_receive.getActualTCPPose()::  [-0.3700943255261627, 0.0001571824753479421, 0.49976943267043744,
        #                                             2.222054563670011, 2.21993176226127, 0.00100073145190608]
        #self._robot_a_receive.getActualQ()::  [3.629941463470459, -0.940093533401825, -2.305957317352295,
        #                                       -1.4669749078205605, -4.714073363934652, 0.4889397621154785]

        print("big_pulley :: ", base_t_big_pulley_a)
        print("small_pulley :: ", base_t_small_pulley_a)

        # pick the belt
        self._gripper_a.open()

        #Align angle
        base_t_big_pulley_a = Pose6D(base_t_big_pulley_a.p()[0],base_t_big_pulley_a.p()[1],base_t_big_pulley_a.p()[2],0.0,0.0,0.0).to_transform_3d()
        base_t_small_pulley_a = Pose6D(base_t_small_pulley_a.p()[0],base_t_small_pulley_a.p()[1],base_t_small_pulley_a.p()[2],0.0,0.0,0.0).to_transform_3d()
        t_base_robot_end = Pose6D(self._robot_a_receive.getActualTCPPose()).to_transform_3d()
        t_end_big_pulley_a = t_base_robot_end.inverse() * base_t_big_pulley_a
        t_end_small_pulley_a = t_base_robot_end.inverse() * base_t_small_pulley_a

        p = t_end_big_pulley_a.p() - t_end_small_pulley_a.p()

        align_angle_z_ = math.atan2(p[0], p[1])  # y axis align
        print("align_angle_z_: ", align_angle_z_)

        if align_angle_z_ >= 179*Pi/180:
            align_angle_z_ = 179*Pi/180
        if align_angle_z_ <= -179*Pi/180:
            align_angle_z_ = -179*Pi/180


        t_align_angle_z_ = Transform3D(Vector3D(0.0, 0.0, 0.0),
                                         EAA(normalize(Vector3D(0.0, 0.0, 1.0)), -align_angle_z_))

        t_base_robot_end = Pose6D(self._robot_a_receive.getActualTCPPose()).to_transform_3d()
        t_base_end_approach = t_base_robot_end * t_align_angle_z_
        self._robot_a_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

        #go to pick position
        t_base_robot_end = Pose6D(self._robot_a_receive.getActualTCPPose()).to_transform_3d()
        t_end_small_pulley = t_base_robot_end.inverse() * base_t_small_pulley_a
        t_end_small_pulley = Transform3D(t_end_small_pulley.p(), EAA(0.0, 0.0, 0.0))
        t_small_pulley_desired_frame_0 = Pose6D(self.desired_frame_0).to_transform_3d()

        t_base_end_approach = t_base_robot_end * t_end_small_pulley * t_small_pulley_desired_frame_0
        self._robot_a_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

        # down to pick position
        t_base_robot_end = Pose6D(self._robot_a_receive.getActualTCPPose()).to_transform_3d()
        t_end_desired_frame_1 = Pose6D(self.desired_frame_1).to_transform_3d()
        t_base_end_approach = t_base_robot_end * t_end_desired_frame_1
        self._robot_a_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

        # pick the belt
        self._gripper_a.close()

        # go to the z direction
        t_base_robot_end = Pose6D(self._robot_a_receive.getActualTCPPose()).to_transform_3d()
        t_end_desired_frame_2 = Pose6D(self.desired_frame_2).to_transform_3d()
        t_base_end_approach = t_base_robot_end * t_end_desired_frame_2
        self._robot_a_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

        # go to the initial position
        t_base_robot_end = Pose6D(self._robot_a_receive.getActualTCPPose()).to_transform_3d()
        t_end_desired_frame_3 = Pose6D(self.desired_frame_3).to_transform_3d()
        t_base_end_approach = t_base_robot_end * t_end_desired_frame_3
        self._robot_a_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

        self._robot_a_control.move_home(self._speed_a, self._acc_a)
