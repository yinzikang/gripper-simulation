import PyKDL as kdl
import numpy as np

joint_num = 3
PI = np.pi


def createChain():
    jnts = []
    frms = []

    # jnts.append(kdl.Joint(name="joint1", origin=kdl.Vector(0, 0, 0), axis=kdl.Vector(0, 0, 1), type=kdl.Joint.RotZ, inertia=0.01, damping=0.1))
    # jnts.append(kdl.Joint("joint1", kdl.Vector(0, 0, 0),kdl.Vector(0, 0, 1), 6))
    # jnts.append(kdl.Joint(name="joint1", origin=kdl.Vector(0, 0, 0), axis=kdl.Vector(0, 0, 1)))
    # jnts.append(kdl.Joint("joint1", inertia=0.01, damping=0.1))
    jnts.append(kdl.Joint("joint1", kdl.Joint.RotY))
    jnts.append(kdl.Joint("joint2", kdl.Joint.RotY))
    jnts.append(kdl.Joint("joint3", kdl.Joint.Fixed))

    print(kdl.Joint.getType(jnts[0]))

    frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 1)))
    frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 1)))
    frms.append(kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 1)))

    rbt = kdl.Chain()
    link = []
    for i in range(joint_num):
        link.append(kdl.Segment(jnts[i], frms[i]))
        rbt.addSegment(link[i])
    return rbt


def getForwardKinematics(robot, joint_pos):
    fk = kdl.ChainFkSolverPos_recursive(robot)
    cart_pos_ori = kdl.Frame()
    # cart_pos_ori.p = kdl.Vector(1,1,0)
    fk.JntToCart(joint_pos, cart_pos_ori)
    return cart_pos_ori


def getInverseKinematics(robot, joint_init_pos, cart_pos_ori):
    ik = kdl.ChainIkSolverPos_LMA(robot, maxiter=1500)
    joint_pos = kdl.JntArray(joint_num)
    ik.CartToJnt(joint_init_pos, cart_pos_ori, joint_pos)
    return joint_pos


kdl_robot = createChain()
kdl_state = kdl.JntArray(joint_num)
kdl_state[1] = PI/2
set_v = kdl.Vector(2, 0, 1)
set_r = kdl.Rotation(0, 0, 1, 0, 1, -1, 0, 0, 0)
set_p = kdl.Frame(set_r, set_v)

ee_pos = getForwardKinematics(kdl_robot, kdl_state)
print(ee_pos)
q_pos = getInverseKinematics(kdl_robot, kdl.JntArray(joint_num), set_p)
print(q_pos)


