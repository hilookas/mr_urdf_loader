
from urchin import URDF
import numpy as np
from mr_urdf_loader import loadURDF
import modern_robotics as mr

np.set_printoptions(precision=4, suppress=True)

robot = URDF.load("/home/rosdev/ros2_ws/src/astra_description/urdf/astra_description_rel.urdf")

M, Slist, Blist, Mlist, Glist, robot = loadURDF(
    "/home/rosdev/ros2_ws/src/astra_description/urdf/astra_description_rel.urdf", 
    eef_link_name="link_ree", 
    actuated_joint_names=['joint_r1', 'joint_r2', 'joint_r3', 'joint_r4', 'joint_r5', 'joint_r6']
)

joint_state = [0.2, 0.1, 0.1, 0, 0, 0]

fk = robot.link_fk(cfg={
    'joint_r1': joint_state[0],
    'joint_r2': joint_state[1],
    'joint_r3': joint_state[2],
    'joint_r4': joint_state[3],
    'joint_r5': joint_state[4],
    'joint_r6': joint_state[5]
})

print(fk[robot.link_map['link_ree']])

print(mr.FKinSpace(M, Slist, joint_state))

robot.show(cfg={
    'joint_r1': joint_state[0],
    'joint_r2': joint_state[1],
    'joint_r3': joint_state[2],
    'joint_r4': joint_state[3],
    'joint_r5': joint_state[4],
    'joint_r6': joint_state[5]
})