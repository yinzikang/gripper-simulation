import mujoco_py
import PyKDL as kdl
import numpy as np


def createChain1():
    rbt = kdl.Chain()

    seg1 = kdl.Segment("l1",
                       kdl.Joint("j1", kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
                       kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 1)),
                       )
    seg1.setInertia(kdl.RigidBodyInertia(1, kdl.Vector(0, 0, 0.5),
                                            kdl.RotationalInertia(1, 1, 1)))
    seg2 = kdl.Segment("l2",
                       kdl.Joint("j2", kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
                       kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 1)),
                       kdl.RigidBodyInertia(1, kdl.Vector(0, 0, 0.5),
                                            kdl.RotationalInertia(1, 1, 1)))
    seg2.setInertia(kdl.RigidBodyInertia(1, kdl.Vector(0, 0, 0.5),
                                            kdl.RotationalInertia(1, 1, 1)))
    seg3 = kdl.Segment("l3",
                       kdl.Joint("j3", kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
                       kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 1)),
                       kdl.RigidBodyInertia(1, kdl.Vector(0, 0, 0.5),
                                            kdl.RotationalInertia(1, 1, 1)))
    rbt.addSegment(seg1)
    rbt.addSegment(seg2)
    # rbt.addSegment(seg3)

    return rbt

def createChain2():
    rbt = kdl.Chain()

    # seg1 = kdl.Segment("l1",
    #                    kdl.Joint("j1", kdl.Vector(0, 0, 1), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
    #                    kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 1)),
    #                    kdl.RigidBodyInertia(1, kdl.Vector(0, 0, 0),
    #                                         kdl.RotationalInertia(1, 1, 1)))
    seg1 = kdl.Segment("l1",
                       kdl.Joint("j1", kdl.Vector(0, 0, 1), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
                       kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 1)),
                       kdl.RigidBodyInertia(1, kdl.Vector(0, 0, 0.5),
                                            kdl.RotationalInertia(1, 1, 1)))
    seg2 = kdl.Segment("l2",
                       kdl.Joint("j2", kdl.Vector(0, 0, 1), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
                       kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 1)),
                       kdl.RigidBodyInertia(1, kdl.Vector(0, 0, 0.5),
                                            kdl.RotationalInertia(1, 1, 1)))
    # seg3 = kdl.Segment("l3",
    #                    kdl.Joint("j3", kdl.Vector(0, 0, 1), kdl.Vector(0, 1, 0), kdl.Joint.RotAxis),
    #                    kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 1)),
    #                    kdl.RigidBodyInertia(1, kdl.Vector(0, 0, 0.5),
    #                                         kdl.RotationalInertia(1, 1, 1)))
    rbt.addSegment(seg1)
    rbt.addSegment(seg2)
    # rbt.addSegment(seg3)

    return rbt

def createChain3():
    rbt = kdl.Chain()
    rbt.addSegment(kdl.Segment(
        "link1", kdl.Joint(kdl.Vector(0, 0, 0.069), kdl.Vector(0, 0, 1), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 0.069)),
        kdl.RigidBodyInertia(4.27, kdl.Vector(-0.000038005, -0.0024889, 0.054452),
                             kdl.RotationalInertia(0.0340068, 0.0340237, 0.00804672, -5.0973e-06, 2.9246e-05, 0.00154238))))

    rbt.addSegment(kdl.Segment(
        "link2", kdl.Joint(kdl.Vector(0, 0, 0.073), kdl.Vector(0, -1, 0), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 0, -1, 0, 1, 0), kdl.Vector(0, 0, 0.073)),
        kdl.RigidBodyInertia(10.1, kdl.Vector(0, 0.21252, 0.12053),
                             kdl.RotationalInertia(0.771684, 1.16388, 1.32438, -5.9634e-05, 0.258717, -0.258717))))
    return rbt


def createChain4():
    rbt = kdl.Chain()
    rbt.addSegment(kdl.Segment(
        "link1", kdl.Joint(kdl.Vector(0, 0, 0), kdl.Vector(0, 0, 1), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 0.069)),
        kdl.RigidBodyInertia(4.27, kdl.Vector(-0.000038005, -0.0024889, 0.054452),
                             kdl.RotationalInertia(0.0340068, 0.0340237, 0.00804672, -5.0973e-06, 2.9246e-05, 0.00154238))))

    rbt.addSegment(kdl.Segment(
        "link2", kdl.Joint(kdl.Vector(0, 0, 0), kdl.Vector(0, -1, 0), kdl.Joint.RotAxis),
        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 0, -1, 0, 1, 0), kdl.Vector(0, 0, 0.073)),
        kdl.RigidBodyInertia(10.1, kdl.Vector(0, 0.21252, 0.12053),
                             kdl.RotationalInertia(0.771684, 1.16388, 1.32438, -5.9634e-05, 0.258717, -0.258717))))
    rbt
    return rbt

def kdlGetForwardKinematics(robot, kdl_joint_pos):
    fksolver = kdl.ChainFkSolverPos_recursive(robot)
    cart_pos_ori = kdl.Frame()
    fksolver.JntToCart(kdl_joint_pos, cart_pos_ori)
    # cart_pos_ori.p = cart_pos_ori.p
    return cart_pos_ori


def kdlGetInverseDynamics(robot, kdl_joint_pos, kdl_joint_vel, kdl_joint_acc, kdl_fext, num):
    gravity = kdl.Vector(0, 0, -10)
    idsolver = kdl.ChainIdSolver_RNE(robot, gravity)
    torque = kdl.JntArray(num)
    idsolver.CartToJnt(kdl_joint_pos, kdl_joint_vel, kdl_joint_acc, kdl_fext, torque)
    return torque

def kdlJacob(robot, kdl_joint_pos, num):
    output = kdl.Jacobian(num)
    jac_solver = kdl.ChainJntToJacSolver(robot)
    jac_solver.JntToJac(kdl_joint_pos, output)
    ret = [[0] * num for i in range(6)]
    for i in range(6):
        for j in range(num):
            ret[i][j] = output.__getitem__((i, j))
    return ret

num = 2
kdl_joint_pos = kdl.JntArray(num)
kdl_joint_pos[0] = np.pi/2
# kdl_joint_pos[1] = 0
vel = kdl.JntArray(num)
wrench = kdl.Wrench().Zero()

kdlrobot = createChain1()
p = kdlGetForwardKinematics(kdlrobot, kdl_joint_pos)

t = kdlGetInverseDynamics(kdlrobot, kdl_joint_pos, vel, vel, [wrench]*num, num)
j = kdlJacob(kdlrobot, kdl_joint_pos, num)
print(p)
print(t)
print(j)

# c = kdl.RigidBodyInertia(4, kdl.Vector(0, 0, 0), kdl.RotationalInertia(1, 1, 1))
# print(c.RefPoint())
