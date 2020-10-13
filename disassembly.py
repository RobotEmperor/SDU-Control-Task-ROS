    def disassembly_place_belt(self, obj: Object):
        self._robot_model = self._robot_a_control.get_model()
        self.robot_end = self._robot_model.get_end()

        self.robot_end.set_transform(Pose6D(self._robot_a_receive.getActualTCPPose()))
        # q_safe = (2.8662071228027344, -1.7563158474364222, -1.9528794288635254,
       #           -1.0198443692973633, -4.752078358327047, -2.1280840078936976)

        # self._robot_control.ctrl.moveJ(q_safe, 0.1, 0.1)
        z_tool_ = -0.2
        self.desired_frame_0 = [0.025, 0.07, -0.0135 + z_tool_, 0, 0, 0]
        self.desired_frame_1 = [0, 0, 0.03, 0, 0, 0]
        self.desired_frame_2 = [0, -0.07, -0.035, 0, 0, 0]
        self.desired_frame_3 = [0, 0.07, 0, 0, 0, 0]  # big pulley

        big_pulley = obj.find_frame("big_pulley")
        small_pulley = obj.find_frame("small_pulley")

        #base_t_big_pulley_a = Kinematics.frame_t_frame( self._robot_model.get_base(), big_pulley, Transform3D)
        #base_t_small_pulley_a = Kinematics.frame_t_frame(self._robot_model.get_base(), small_pulley, Transform3D)

        base_t_big_pulley_a = Transform3D(Vector3D(-0.642689028290174, 0.32559504347025003, 0.05515902491897915),
                                  Rotation3D(0.005288428814332287, 0.49500821320042104, -0.8688722008355444,
                                             -0.0027536414386920647, -0.8688738481716058, -0.49502591186956296,
                                             -0.9999822248317732, 0.005010471793320436, -0.0032319023663288995))

        base_t_small_pulley_a = Transform3D(Vector3D(-0.7112236259771654, 0.4397112211231814, 0.05398238613972542),
                                    Rotation3D(0.005289513396601397, -0.15890102762114197, -0.9872803474540413,
                                               -0.0027524267256131183, -0.9872927328736917, 0.1588882744632767,
                                               -0.9999822224395529, 0.001876975157668792, -0.00565966157205448))


        print("big_pulley :: ", base_t_big_pulley_a)
        print("small_pulley :: ", base_t_small_pulley_a)

        p = base_t_big_pulley_a.p() - base_t_small_pulley_a.p()

        align_angle_z_ = math.atan2(p[0], p[1])  # y axis align

        print("align_angle_z_: ", align_angle_z_)

        t_align_angle_z_ = Transform3D(Vector3D(0.0, 0.0, 0.0), EAA(normalize(Vector3D(0.0, 0.0, 1.0)), align_angle_z_ - Pi/2))

        #t_end_gripper = Kinematics.frame_t_frame(self._robot_a_model.get_end(), self._gripper_a_model.get_end(), Transform3D)
        #t_end_gripper = t_end_gripper*Pose6D(0.0, 0.0, 0.035, 0.0, 0.0, 0.0).to_transform_3d()
        #world_taskboard_base = self._taskboard.get_base().world_t_frame(Transform3D)

        t_base_robot_end = Pose6D(self._robot_a_receive.getActualTCPPose()).to_transform_3d()

        t_base_end_approach = t_base_robot_end * t_align_angle_z_
        self._robot_a_control.moveL(Pose6D(t_base_end_approach)[0:6], 0.1, 0.1)

        # go to pick position
        t_base_robot_end = Pose6D(self._robot_a_receive.getActualTCPPose()).to_transform_3d()
        #t_base_robot_tool = t_base_robot_end * t_end_tool
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
