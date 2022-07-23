import mujoco_py
import PyKDL as kdl
import numpy as np


class my_ur5_nftgripper_env(object):

    def __init__(self, init_pos, model_path):
        self.joint_num = 2
        self.kdl_model = self.kdlCreateChain()
        self.mjc_model = mujoco_py.load_model_from_path(model_path)
        self.sim = mujoco_py.MjSim(model=self.mjc_model)
        self.viewer = mujoco_py.MjViewer(self.sim)

        self.PI = np.pi
        # 初始化下的状态，用于以后状态的set，并不能主动改变
        # kdl由于bug无法设置fix关节，因此设为旋转关节，但是切记不能修改关节值
        self.kdl_joint_pos_state = kdl.JntArray(self.joint_num)
        self.kdl_joint_vel_state = kdl.JntArray(self.joint_num)
        self.kdl_joint_acc_state = kdl.JntArray(self.joint_num)
        # self.kdl_joint_pos_state[0] = 0
        # self.kdl_joint_pos_state[7] = 0
        # self.kdl_joint_vel_state[0] = 0
        # self.kdl_joint_vel_state[7] = 0
        # self.kdl_joint_acc_state[0] = 0
        # self.kdl_joint_acc_state[7] = 0
        self.mjc_state = self.mjcGetAndUpdateStatus()
        self.kdl_bias = kdl.Vector(0, 0, 0.959159)  # 0.959159

        self.joint_list = list(self.mjc_model.joint_names)
        self.actuator_list = list(self.mjc_model.actuator_names)

        for joint_id in range(self.joint_num):
            self.sim.data.set_joint_qpos(self.joint_list[joint_id], init_pos[joint_id])
        self.sim.forward()
        self.sim.set_state(self.mjcGetAndUpdateStatus())

    def mjcGetAndUpdateStatus(self):
        self.mjc_state = self.sim.get_state()
        return self.mjc_state

    def mjcGetJointPos(self):
        return self.sim.data.qpos

    def mjcGetJointVel(self):
        return self.sim.data.qvel

    # def mjcGetEndEffectorPosition(self):
    #     return self.sim.data.get_body_xpos('ee_link')

    def kdlCreateChain(self):
        # __init__(name, PyKDL.Joint, PyKDL.Joint, PyKDL.Joint, PyKDL.RigidBodyInertia)
        name = []
        jnts = []
        frms = []
        iner = []

        # name.append("base_link")
        name.append("shoulder_link")
        name.append("upper_arm_link")
        # name.append("forearm_link")
        # name.append("wrist_1_link")
        # name.append("wrist_2_link")
        # name.append("wrist_3_link")
        # name.append("ee_link")

        # constructor for joint along x, y or z axis, at origin of reference frame
        # jnts.append(kdl.Joint("base_world_joint", kdl.Joint.RotZ))
        jnts.append(kdl.Joint("shoulder_pan_joint", kdl.Joint.RotZ))
        jnts.append(kdl.Joint("shoulder_lift_joint", kdl.Joint.RotY))
        # jnts.append(kdl.Joint("elbow_joint", kdl.Joint.RotY))
        # jnts.append(kdl.Joint("wrist_1_joint", kdl.Joint.RotY))
        # jnts.append(kdl.Joint("wrist_2_joint", kdl.Joint.RotZ))
        # jnts.append(kdl.Joint("wrist_3_joint", kdl.Joint.RotY))
        # jnts.append(kdl.Joint("tool_joint", kdl.Joint.RotZ))

        # Rotx() give the value of the appropriate rotation matrix back.
        # frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.089159)))
        frms.append(kdl.Frame(kdl.Rotation.RotY(np.pi/2), kdl.Vector(0, 0.13585, 0)))
        frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, -0.1197, 0.425)))
        # frms.append(kdl.Frame(kdl.Rotation.RotY(np.pi/2), kdl.Vector(0, 0, 0.39225)))
        # frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0.093, 0)))
        # frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.09465)))
        # frms.append(kdl.Frame(kdl.Rotation.RotX(-np.pi/2), kdl.Vector(0, 0.0823, 0)))
        # frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0)))

        # RigidBodyInertia (double m, const Vector &h, const RotationalInertia &I, bool mhi)
        # iner.append(kdl.RigidBodyInertia(4, kdl.Vector(0, 0, 0),
        #                                  kdl.RotationalInertia(0.00443333156, 0.00443333156, 0.0072)))
        iner.append(kdl.RigidBodyInertia(3.7, kdl.Vector(0, 0, 0),
                                         kdl.RotationalInertia(0.010267495893, 0.010267495893, 0.00666)))
        iner.append(kdl.RigidBodyInertia(8.393, kdl.Vector(0, 0, 0.28),
                                         kdl.RotationalInertia(0.22689067591, 0.22689067591, 0.0151074)))  # 8.393 0.28
        # iner.append(kdl.RigidBodyInertia(2.275, kdl.Vector(0, 0, 0.25),
        #                                  kdl.RotationalInertia(0.049443313556, 0.049443313556, 0.004095)))
        # iner.append(kdl.RigidBodyInertia(1.219, kdl.Vector(0, 0, 0),
        #                                  kdl.RotationalInertia(0.111172755531, 0.111172755531, 0.21942)))
        # iner.append(kdl.RigidBodyInertia(1.219, kdl.Vector(0, 0, 0),
        #                                  kdl.RotationalInertia(0.111172755531, 0.111172755531, 0.21942)))
        # iner.append(kdl.RigidBodyInertia(0.1879, kdl.Vector(0, 0, 0),
        #                                  kdl.RotationalInertia(0.0171364731454, 0.0171364731454, 0.033822)))
        # iner.append(kdl.RigidBodyInertia(1.3, kdl.Vector(0, 0, 0.069871),
        #                                  kdl.RotationalInertia(0.00800113, 0.00972386, 0.00176438)))

        rbt = kdl.Chain()

        link = []
        for i in range(self.joint_num):
            iner[i] = iner[i].RefPoint(frms[i].p)
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
        iksolver = kdl.ChainIkSolverPos_LMA(self.kdl_model, maxiter=1500)
        joint_pos = kdl.JntArray(self.joint_num + 2)
        iksolver.CartToJnt(kdl_joint_init_pos, kdl_cart_pos_ori - self.kdl_bias, joint_pos)
        return joint_pos

    # def kdlGetForwardDynamics(self):
    #     fd = kdl.ChainFdSolver_RNE(self.kdl_model, )
    #     fd.CartToJnt
    #     return 0

    def kdlGetInverseDynamics(self, kdl_joint_pos, kdl_joint_vel, kdl_joint_acc, kdl_fext):
        gravity = kdl.Vector(0, 0, -9.81)
        idsolver = kdl.ChainIdSolver_RNE(self.kdl_model, gravity)
        torque = kdl.JntArray(self.joint_num)
        idsolver.CartToJnt(kdl_joint_pos, kdl_joint_vel, kdl_joint_acc, kdl_fext, torque) #
        return torque

    def kdlCartSpaceToJointSpace(self, kdl_cart_pos_array, kdl_cart_ori_array):
        pos_num = len(kdl_cart_pos_array)
        # ori_num = len(ori_array)
        # if pos_num != ori_num:
        #     print('error path')

        joint_last_pos = kdl.JntArray(self.joint_num + 2)
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

    def mjcTransTokdl_PosVelAcc(self):
        for i in range(self.joint_num):
            self.kdl_joint_pos_state[i] = self.mjc_state.qpos[i]
            self.kdl_joint_vel_state[i] = self.mjc_state.qvel[i]
            # self.kdl_joint_acc_state[i + 1] = self.mjc_state.qacc[i]
        return 0


def Trajectory_Generation(cart_int_pos, cart_end_pos, dot_num):
    trajectory = []
    for i in range(dot_num):
        trajectory.append(cart_int_pos + (cart_end_pos - cart_int_pos) / (dot_num - 1) * i)
    return trajectory
