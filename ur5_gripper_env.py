import mujoco_py
import PyKDL as kdl
import numpy as np


class my_ur5_gripper_env(object):

    def __init__(self, init_pos, model_path):
        self.joint_num = 6
        self.kdl_model = self.createChain()
        self.mjc_model = mujoco_py.load_model_from_path(model_path)
        self.sim = mujoco_py.MjSim(model=self.mjc_model)
        self.viewer = mujoco_py.MjViewer(self.sim)

        self.kdl_state = kdl.JntArray(self.joint_num)
        self.mjc_state = self.getStatus()
        self.kdl_bias = kdl.Vector(0, 0, 0.959159)

        self.body_list = list(self.mjc_model.body_names)
        self.joint_list = list(self.mjc_model.joint_names)
        self.actuator_list = list(self.mjc_model.actuator_names)

        for joint_id in range(self.joint_num):
            self.sim.data.set_joint_qpos(self.joint_list[joint_id], init_pos[joint_id])
        self.sim.forward()
        self.sim.set_state(self.getStatus())

    def createChain(self):
        jnts = []
        frms = []

        # __init__(self: PyKDL.Joint, arg0: PyKDL.Joint)
        # constructor for joint along x, y or z axis, at origin of reference frame
        jnts.append(kdl.Joint(kdl.Joint.RotZ))
        jnts.append(kdl.Joint(kdl.Joint.RotY))
        jnts.append(kdl.Joint(kdl.Joint.RotY))
        jnts.append(kdl.Joint(kdl.Joint.RotY))
        jnts.append(kdl.Joint(kdl.Joint.RotZ))
        jnts.append(kdl.Joint(kdl.Joint.RotY))

        # Rotx() give the value of the appropriate rotation matrix back.
        frm1 = kdl.Frame(kdl.Rotation.RotY(np.pi / 2), kdl.Vector(0, 0.13585, 0))
        frm2 = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, -0.1197, 0.425))
        frm3 = kdl.Frame(kdl.Rotation.RotY(np.pi / 2), kdl.Vector(0, 0, 0.39225))
        frm4 = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0.093, 0))
        frm5 = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.09465))
        frm6 = kdl.Frame(kdl.Rotation.RotZ(np.pi / 2), kdl.Vector(0, 0.0823, 0))

        frms.append(frm1)
        frms.append(frm2)
        frms.append(frm3)
        frms.append(frm4)
        frms.append(frm5)
        frms.append(frm6)

        rbt = kdl.Chain()

        link = []
        for i in range(self.joint_num):
            link.append(kdl.Segment(jnts[i], frms[i]))
            rbt.addSegment(link[i])
        return rbt

    def getStatus(self):
        return self.sim.get_state()

    def getJointPosition(self):
        return self.sim.data.qpos

    def getEndEffectorPosition(self):
        return self.sim.data.get_body_xpos('ee_link') - self.sim.data.get_body_xpos('base_link')

    def getForwardKinematics(self, joint_pos):
        fk = kdl.ChainFkSolverPos_recursive(self.kdl_model)
        cart_pos_ori = kdl.Frame()
        fk.JntToCart(joint_pos, cart_pos_ori)
        cart_pos_ori.p = cart_pos_ori.p + self.kdl_bias
        return cart_pos_ori

    def getInverseKinematics(self, joint_init_pos, cart_pos_ori):
        ik = kdl.ChainIkSolverPos_LMA(self.kdl_model, maxiter=1500)
        joint_pos = kdl.JntArray(self.joint_num)
        ik.CartToJnt(joint_init_pos, cart_pos_ori - self.kdl_bias, joint_pos)
        return joint_pos

    def CartSpaceToJointSpace(self, pos_array, ori_array):
        pos_num = len(pos_array)
        # ori_num = len(ori_array)
        # if pos_num != ori_num:
        #     print('error path')

        joint_last_pos = kdl.JntArray(self.joint_num)
        joint_array = []

        for current_path_id in range(pos_num):
            set_v = pos_array[current_path_id] - self.kdl_bias
            # set_r = ori_array[current_path_id]
            set_r = ori_array
            set_p = kdl.Frame(set_r, set_v)
            joint_current_pos = (self.getInverseKinematics(joint_last_pos, set_p))
            joint_last_pos = joint_current_pos
            joint_array.append(joint_current_pos)

        return joint_array


def Trajectory_Generation(int_pos, end_pos, dot_num):
    trajectory = []
    for i in range(dot_num):
        trajectory.append(int_pos + (end_pos - int_pos) / (dot_num - 1) * i)
    return trajectory
