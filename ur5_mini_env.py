import mujoco_py
import PyKDL as kdl
import numpy as np


class my_jk5_nftgripper_env(object):

    def __init__(self, init_pos, model_path):
        self.mjc_joint_num = 6
        self.kdl_joint_num = 8
        self.kdl_model = self.kdlCreateChain()
        self.mjc_model = mujoco_py.load_model_from_path(model_path)
        self.sim = mujoco_py.MjSim(model=self.mjc_model)
        self.viewer = mujoco_py.MjViewer(self.sim)

        self.PI = np.pi
        self.gravity = kdl.Vector(0, 0, -9.81)
        # 初始化下的状态，用于以后状态的set，并不能主动改变
        # kdl由于bug无法设置fix关节，因此设为旋转关节，但是切记不能修改关节值
        self.kdl_joint_pos_state = kdl.JntArray(self.kdl_joint_num)
        self.kdl_joint_vel_state = kdl.JntArray(self.kdl_joint_num)
        self.kdl_joint_acc_state = kdl.JntArray(self.kdl_joint_num)
        self.kdl_joint_pos_state[0] = 0
        self.kdl_joint_pos_state[7] = 0
        self.kdl_joint_vel_state[0] = 0
        self.kdl_joint_vel_state[7] = 0
        self.kdl_joint_acc_state[0] = 0
        self.kdl_joint_acc_state[7] = 0

        self.kdl_joint_pos_min = kdl.JntArray(self.kdl_joint_num)
        self.kdl_joint_pos_max = kdl.JntArray(self.kdl_joint_num)
        self.kdl_joint_pos_min[0] = -0.000001
        self.kdl_joint_pos_min[7] = -0.000001
        self.kdl_joint_pos_max[0] = 0.000001
        self.kdl_joint_pos_max[7] = 0.000001

        # self.kdl_joint_pos_min[0] = -self.PI
        # self.kdl_joint_pos_min[7] = -self.PI
        # self.kdl_joint_pos_max[0] = self.PI
        # self.kdl_joint_pos_max[7] = self.PI
        for joint_id in range(1, 7):
            self.kdl_joint_pos_min[joint_id] = -self.PI
            self.kdl_joint_pos_max[joint_id] = self.PI

        self.mjc_state = self.mjcGetAndUpdateStatus()
        self.kdl_bias = kdl.Vector(0, 0, 0)  # 0.959159

        self.joint_list = list(self.mjc_model.joint_names)
        self.actuator_list = list(self.mjc_model.actuator_names)

        for joint_id in range(self.mjc_joint_num):
            self.sim.data.set_joint_qpos(self.joint_list[joint_id], init_pos[joint_id])
            self.kdl_joint_pos_state[joint_id + 1] = init_pos[joint_id]
        self.sim.forward()
        # self.sim.set_state(self.mjc_state)

    def mjcStep(self):
        for i in range(self.mjc_joint_num):
            self.kdl_joint_pos_state[i + 1] = self.mjc_state.qpos[i]
            self.kdl_joint_vel_state[i + 1] = self.mjc_state.qvel[i]
            # self.kdl_joint_acc_state[i + 1] = self.mjc_state.qacc[i]
        self.sim.step()

    def mjcRender(self):
        self.viewer.render()

    def mjcGetAndUpdateStatus(self):
        self.mjc_state = self.sim.get_state()
        return self.mjc_state

    def mjcGetJointPos(self):
        return self.sim.data.qpos

    def mjcGetJointVel(self):
        return self.sim.data.qvel

    def mjcGetEndEffectorPosition(self):
        return self.sim.data.get_body_xpos('ee_link')

    def kdlCreateChain(self):
        # __init__(name, PyKDL.Joint, PyKDL.Joint, PyKDL.Joint, PyKDL.RigidBodyInertia)
        name = []
        jnts = []
        frms = []
        iner = []

        name.append("base_link")
        name.append("link1")
        name.append("link2")
        name.append("link3")
        name.append("link4")
        name.append("link5")
        name.append("link6")
        name.append("ee_link")

        # constructor for joint along x, y or z axis, at origin of reference frame
        jnts.append(kdl.Joint("base_joint", kdl.Joint.TransZ))
        jnts.append(kdl.Joint("joint1", kdl.Joint.RotZ))
        jnts.append(kdl.Joint("joint2", kdl.Joint.RotZ))
        jnts.append(kdl.Joint("joint3", kdl.Joint.RotZ))
        jnts.append(kdl.Joint("joint4", kdl.Joint.RotZ))
        jnts.append(kdl.Joint("joint5", kdl.Joint.RotZ))
        jnts.append(kdl.Joint("joint6", kdl.Joint.RotZ))
        jnts.append(kdl.Joint("ee_joint", kdl.Joint.TransZ))

        # Rotx() give the value of the appropriate rotation matrix back.
        frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.069)))
        frms.append(kdl.Frame(kdl.Rotation.RotX(np.pi / 2), kdl.Vector(0, 0, 0.073001)))
        frms.append(kdl.Frame(kdl.Rotation.RotZ(np.pi / 2), kdl.Vector(0, 0.425, 0)))
        frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0.395, 0, 0)))
        frms.append(kdl.Frame(kdl.Rotation.Quaternion(-0.5, 0.5, -0.5, 0.5), kdl.Vector(0, 0, 0.1135)))
        frms.append(kdl.Frame(kdl.Rotation.RotX(np.pi / 2), kdl.Vector(0, 0, 0.1015)))
        frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.094)))  # position of nft
        frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.1)))  # end of nft

        # RigidBodyInertia (double m, const Vector &h, const RotationalInertia &I, bool mhi)
        iner.append(kdl.RigidBodyInertia(0.46749, kdl.Vector(3.2145E-06, -0.0017896, 0.027865) - kdl.Vector(0, 0, 0.069)
                                         , kdl.RotationalInertia(0.000936, 0.00081679, 0.0014742,
                                                                 2.3327E-09, 3.5682E-08, 0.0014742)))
        iner.append(kdl.RigidBodyInertia(4.27, kdl.Rotation.RotX(-np.pi / 2) *
                                         (kdl.Vector(-3.8005e-05, -0.0024889, 0.054452) - kdl.Vector(0, 0, 0.073001)),
                                         kdl.RotationalInertia(0.0213197, 0.0213631, 0.00802026,
                                                               -4.6934E-06, 2.04095E-05, 0.000963684)))
        iner.append(kdl.RigidBodyInertia(10.1, kdl.Rotation.RotZ(-np.pi / 2) *
                                         (kdl.Vector(0, 0.21252, 0.12053) - kdl.Vector(0, 0.425, 0)),
                                         kdl.RotationalInertia(0.168792, 1.01715, 0.868214,
                                                               -5.9634E-05, 0.258717, -4.6566E-06)))
        iner.append(kdl.RigidBodyInertia(2.61, kdl.Vector(0.27205, 1.9634E-05, 0.023591) - kdl.Vector(0.395, 0, 0),
                                         kdl.RotationalInertia(0.00482636, 0.262358, 0.260655,
                                                               1.66902E-05, 0.0172355, -1.88915E-07)))
        iner.append(kdl.RigidBodyInertia(1.45, kdl.Rotation.RotZ(np.pi / 2) * kdl.Rotation.RotY(-np.pi / 2) *
                                         (kdl.Vector(3.0707E-05, -0.015152, 0.11185) - kdl.Vector(0, 0, 0.1135)),
                                         kdl.RotationalInertia(0.0201018, 0.0193341, 0.0019386,
                                                               -1.83795E-06, -4.52893E-06, 0.00250956)))
        iner.append(kdl.RigidBodyInertia(1.45, kdl.Rotation.RotX(-np.pi / 2) *
                                         (kdl.Vector(-3.0707E-05, 0.015152, 0.099848) - kdl.Vector(0, 0, 0.1015)),
                                         kdl.RotationalInertia(0.00177669, 0.0133856, 0.0019389,
                                                               -2.44514E-06, -6.63528E-09, -2.247838E-03)))
        iner.append(kdl.RigidBodyInertia(0.21, kdl.Vector(0, 0.00058691, 0.072051) - kdl.Vector(0, 0, 0.094),
                                         kdl.RotationalInertia(0.00128044, 0.0012085, 0.000194552,
                                                               0, 0, -8.2685393E-06)))
        iner.append(kdl.RigidBodyInertia(1.371, kdl.Vector(0, 0, 0.069871) - kdl.Vector(0, 0, 0.1),
                                         kdl.RotationalInertia(0.00800113, 0.00972386, 0.00176438)))  # nft

        # iner.append(kdl.RigidBodyInertia(0.46749, kdl.Vector(3.2145E-06, -0.0017896, 0.027865),
        #                                  kdl.RotationalInertia(0.000936, 0.00081679, 0.0014742,
        #                                                        2.3327E-09, 3.5682E-08, 0.0014742)))
        # iner.append(kdl.RigidBodyInertia(4.27, kdl.Vector(-3.8005e-05, -0.0024889, 0.054452),
        #                                  kdl.RotationalInertia(0.0213197, 0.0213631, 0.00802026,
        #                                                        -4.6934E-06, 2.04095E-05, 0.000963684)))
        # iner.append(kdl.RigidBodyInertia(10.1, kdl.Vector(0, 0.21252, 0.12053),
        #                                  kdl.RotationalInertia(0.168792, 1.01715, 0.868214,
        #                                                        -5.9634E-05, 0.258717, -4.6566E-06)))
        # iner.append(kdl.RigidBodyInertia(2.61, kdl.Vector(0.27205, 1.9634E-05, 0.023591),
        #                                  kdl.RotationalInertia(0.00482636, 0.262358, 0.260655,
        #                                                        1.66902E-05, 0.0172355, -1.88915E-07)))
        # iner.append(kdl.RigidBodyInertia(1.45, kdl.Vector(3.0707E-05, -0.015152, 0.11185),
        #                                  kdl.RotationalInertia(0.0201018, 0.0193341, 0.0019386,
        #                                                        -1.83795E-06, -4.52893E-06, 0.00250956)))
        # iner.append(kdl.RigidBodyInertia(1.45, kdl.Vector(-3.0707E-05, 0.015152, 0.099848),
        #                                  kdl.RotationalInertia(0.00177669, 0.0133856, 0.0019389,
        #                                                        -2.44514E-06, -6.63528E-09, -2.247838E-03)))
        # iner.append(kdl.RigidBodyInertia(0.21, kdl.Vector(0, 0.00058691, 0.072051),
        #                                  kdl.RotationalInertia(0.00128044, 0.0012085, 0.000194552,
        #                                                        0, 0, -8.2685393E-06)))
        # iner.append(kdl.RigidBodyInertia(0, kdl.Vector(0, 0, 0), kdl.RotationalInertia(0, 0, 0)))

        rbt = kdl.Chain()

        link = []
        for i in range(self.kdl_joint_num):
            # iner[i] = iner[i].RefPoint(frms[i].p)
            link.append(kdl.Segment(name[i], jnts[i], frms[i], iner[i]))
            rbt.addSegment(link[i])
        return rbt

    def kdlGetForwardKinematics(self, kdl_joint_pos):
        fksolver = kdl.ChainFkSolverPos_recursive(self.kdl_model)
        cart_pos_ori = kdl.Frame()
        fksolver.JntToCart(kdl_joint_pos, cart_pos_ori)
        cart_pos_ori.p = cart_pos_ori.p + self.kdl_bias
        return cart_pos_ori

    def kdlGetInverseKinematics(self, kdl_joint_init_pos, kdl_cart_pos_ori):
        # iksolver = kdl.ChainIkSolverPos_LMA(self.kdl_model, maxiter=1500)
        # joint_pos = kdl.JntArray(self.kdl_joint_num)
        # iksolver.CartToJnt(kdl_joint_init_pos, kdl_cart_pos_ori, joint_pos)  # - self.kdl_bias
        # return joint_pos

        fkpossolver = kdl.ChainFkSolverPos_recursive(self.kdl_model)
        ikvelsolver = kdl.ChainIkSolverVel_pinv(self.kdl_model)
        ik = kdl.ChainIkSolverPos_NR_JL(self.kdl_model, self.kdl_joint_pos_min, self.kdl_joint_pos_max,
                                        fkpossolver, ikvelsolver, maxiter=5000)
        joint_pos = kdl.JntArray(self.kdl_joint_num)
        ik.CartToJnt(kdl_joint_init_pos, kdl_cart_pos_ori, joint_pos)
        return joint_pos

    # def kdlGetForwardDynamics(self):
    #     fd = kdl.ChainFdSolver_RNE(self.kdl_model, )
    #     fd.CartToJnt
    #     return 0

    def kdlGetInverseDynamics(self, kdl_joint_pos, kdl_joint_vel, kdl_joint_acc, kdl_fext):
        idsolver = kdl.ChainIdSolver_RNE(self.kdl_model, self.gravity)
        torque = kdl.JntArray(self.kdl_joint_num)
        idsolver.CartToJnt(kdl_joint_pos, kdl_joint_vel, kdl_joint_acc, kdl_fext, torque)
        return torque

    def kdlCartSpaceToJointSpace(self, kdl_cart_pos_array, kdl_cart_ori_array):
        pos_num = len(kdl_cart_pos_array)
        # ori_num = len(ori_array)
        # if pos_num != ori_num:
        #     print('error path')

        joint_last_pos = kdl.JntArray(self.kdl_joint_num)
        joint_array = []

        for current_path_id in range(pos_num):
            set_v = kdl_cart_pos_array[current_path_id] - self.kdl_bias
            # set_r = ori_array[current_path_id]
            set_r = kdl_cart_ori_array
            set_p = kdl.Frame(set_r, set_v)
            joint_current_pos = (self.kdlGetInverseKinematics(joint_last_pos, set_p))
            joint_last_pos = joint_current_pos
            joint_array.append(joint_current_pos)

        return joint_array


def Trajectory_Generation(cart_int_pos, cart_end_pos, dot_num):
    trajectory = []
    for i in range(dot_num):
        trajectory.append(cart_int_pos + (cart_end_pos - cart_int_pos) / (dot_num - 1) * i)
    return trajectory
