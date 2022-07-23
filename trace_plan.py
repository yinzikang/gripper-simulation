import numpy as np
import ur5_gripper_env
import ur5_mini_env
import os
import PyKDL as kdl
import matplotlib.pyplot as plt
ax = plt.axes(projection='3d')

# model_path = "UR5_gripper/UR5mini.xml"
# init_pos = [0, 0, 0, 0, 0, 0]
# env = ur5_mini_env.my_ur5_mini_env(init_pos, model_path)
model_path = "UR5_gripper/UR5gripper.xml"
init_pos = [0, 0, 0, 0, 0, 0]
env = ur5_gripper_env.my_ur5_gripper_env(init_pos, model_path)

duration = 10000
step = 0
int_pos = kdl.Vector(0.5, 0.19145, 0) + env.kdl_bias
tar_pos = kdl.Vector(0, 0.19145, 0.5) + env.kdl_bias

int_ori = kdl.Rotation(0, 1, 0, 1, 0, 0, 0, 0, -1)  # gripper
# int_ori = kdl.Rotation(-1, 0, 0, 0, 0, 1, 0, 1, 0)  # mini

cart_path = ur5_gripper_env.Trajectory_Generation(int_pos, tar_pos, duration)
joint_path = env.CartSpaceToJointSpace(cart_path, int_ori)
ee_pos = np.empty(shape=(duration, 3))

while step < duration:

    for i in range(env.joint_num):
        env.mjc_state.qpos[i] = joint_path[step][i]
    env.sim.set_state(env.mjc_state)

    env.sim.step()
    env.viewer.render()

    ee_pos[step] = env.sim.data.get_body_xpos('ee_link')
    print(ee_pos[step])
    step += 1

    if os.getenv('TESTING') is not None:
        break

# ax.scatter3D(ee_pos[:, 0], ee_pos[:, 1], ee_pos[:, 2], cmap='Blues')
# ax.set_xlim(0, 1)
# ax.set_ylim(0, 1)
# ax.set_zlim(0, 1)
# plt.show()
