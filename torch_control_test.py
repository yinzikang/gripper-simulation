import numpy as np
import ur5_gripper_env
import os
import PyKDL as kdl
import matplotlib.pyplot as plt

model_path = "UR5_gripper/UR5gripper.xml"
init_pos = [-1, 0, 0, 0, 0, 0]
env = ur5_gripper_env.my_ur5_gripper_env(init_pos, model_path)

duration = 7000
step = 0

env.sim.data.ctrl[0] = 2
# pos = np.empty((duration, 3))

while step < duration:

    if env.sim.data.qpos[0] > env.mjc_model.mjPI:
        env.sim.data.ctrl[0] = -2
    if env.sim.data.qpos[0] < -1.5:
        env.sim.data.ctrl[0] = 2

    env.sim.step()
    env.viewer.render()

    # print(env.sim.data.qpos[0])
    # pos[step, :] = env.getEndEffectorPosition()
    # print(pos[step])
    step += 1

    if os.getenv('TESTING') is not None:
        break

# ax = plt.axes(projection='3d')
# ax.scatter3D(ee_pos[:, 0], ee_pos[:, 1], ee_pos[:, 2], cmap='Blues')
# ax.set_xlim(0, 1)
# ax.set_ylim(0, 1)
# ax.set_zlim(0, 1)
# plt.show()

# plt.plot(range(step), pos[:, 2])
# plt.show()
# plt.savefig('fig1.jpg')
