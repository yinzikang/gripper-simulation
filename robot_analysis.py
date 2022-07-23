import ur5_gripper_env
import ur5_mini_env
import ur5_nftgripper_env
import mujoco_py
import PyKDL as kdl
import numpy
import matplotlib.pyplot as plt

model_path = "UR5_gripper/UR5nftgripper.xml"
init_pos = [0, 0]
env = ur5_nftgripper_env.my_ur5_nftgripper_env(init_pos, model_path)
# model_path = "UR5_gripper/UR5mini.xml"
# init_pos = [0, 0, 0, 0, 0, 0]
# env = ur5_mini_env.my_ur5_mini_env(init_pos, model_path)
# model_path = "UR5_gripper/UR5.xml"
# init_pos = [0, 0, 0, 0, 0, 0]
# env = ur5_gripper_env.my_ur5_gripper_env(init_pos, model_path)


# for UR5gripper,UR5palm,UR5
# for j in range(env.joint_num):
#     env.kdl_joint_pos_state[j + 1] = env.mjc_state.qpos[j]
# ee_pos = env.kdlgetForwardKinematics(env.kdl_joint_pos_state)
# for ur5nftgripper
# for j in range(env.joint_num):
#     env.kdl_state[j] = env.mjc_state.qpos[j]
# ee_pos = env.getForwardKinematics(env.kdl_state)

# print(ee_pos)
# print(env.sim.data.get_body_xpos('ee_link'))

# env.mjcGetAndUpdateStatus()
env.mjcTransTokdl_PosVelAcc()
pos = env.kdlGetForwardKinematics(env.kdl_joint_pos_state)
print(pos)


fext = kdl.Wrench(kdl.Vector(0, 0, 0), kdl.Vector(0, 0, 0))
fextlist = [fext] * 2
torque = env.kdlGetInverseDynamics(env.kdl_joint_pos_state, env.kdl_joint_vel_state, env.kdl_joint_acc_state, fextlist)
print(torque,'\n')

# pymjmodel中质量惯量属性
# print(env.mjc_model.body_ipos)
# print(env.mjc_model.body_inertia)
# print(env.mjc_model.body_iquat)
# print(env.mjc_model.body_mass)


# pymjmodel中驱动器属性
# print(env.mjc_model.nq)
# print(env.mjc_model.nu)
# print(env.mjc_model.nv)
# print(env.mjc_model.actuator_trntype)
# print(env.mjc_model.actuator_dyntype)
# print(env.mjc_model.actuator_gaintype)
# print(env.mjc_model.actuator_biastype)
# print(env.mjc_model.actuator_dynprm)
# print(env.mjc_model.actuator_gainprm)
# print(env.mjc_model.actuator_biasprm)
# print(env.mjc_model.actuator_ctrllimited)
# print(env.mjc_model.actuator_ctrlrange)
# print(env.mjc_model.actuator_forcerange)

# pymjmodel中关节属性
# print(env.mjc_model.jnt_limited)
# print(env.mjc_model.jnt_user)

# pymjmodel dof
# print(env.mjc_model.body_parentid)
# print(env.mjc_model.dof_bodyid)




# 驱动器bias模式
# env.mjc_model.actuator_biastype[0] = 1
# env.mjc_model.actuator_biasprm[0][0] = 2
# env.mjc_model.actuator_gainprm[0] = [2, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# env.sim.data.ctrl[0] = 1
# env.sim.step()

# 驱动器空间与关节空间的驱动器力的比较，减速比gear
# print(env.sim.data.actuator_force)
# print(env.sim.data.qfrc_actuator)

# env.viewer.render()

# print(env.mjc_model.qfrc_actuation)
# print(env.mjc_model.actuator_forcelimited)
# print(env.mjc_model.actuator_forcerange)
# print(env.mjc_model.actuator_gainprm)
# print(env.mjc_model.actuator_biasprm)


# pymjdata中驱动器属性
# print(env.sim.data.actuator_length)
# print(env.sim.data.actuator_velocity)
# print(env.sim.data.actuator_force)
# print(env.sim.data.qfrc_actuator)
# print(env.sim.data.qfrc_applied)
# print(env.sim.data.xfrc_applied)
# print(env.sim.data.qfrc_bias)
# print(env.sim.data.qfrc_constraint)

# print(env.sim.data.efc_force)

# for i in range(6):
#     env.sim.data.ctrl[i] = -env.sim.data.qfrc_bias[i]
# env.sim.data.ctrl[2] = -0.1

duration = 3
step = 0
pos = numpy.empty((duration, 3))
torch = numpy.empty((duration, 1))

while step < duration:
    force = env.sim.data.qfrc_bias - numpy.matmul(env.sim.data.efc_JT, env.sim.data.efc_force)
    for i in range(env.joint_num):
        env.sim.data.ctrl[i] = force[i] / env.mjc_model.actuator_gear[i, 0]
    # env.sim.data.ctrl[1] = - 0.7
    print(env.sim.data.qfrc_bias)  # c
    print(env.sim.data.actuator_force[0:2])
    print(env.sim.data.qfrc_actuator,'\n')
    env.sim.step()
    env.viewer.render()

    # torch[step, :] = env.sim.data.ctrl[1]
    # pos[step, :] = env.mjcGetEndEffectorPosition()
    step += 1

#
# # plt.plot(range(step), pos[:, 2])
# plt.plot(range(step), pos[:, 2])
# plt.show()
# plt.savefig('fig3.jpg')

while 1:
    env.viewer.render()

