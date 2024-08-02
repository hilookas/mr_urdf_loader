import numpy as np

np.set_printoptions(precision=6, suppress=True)
from mr_urdf_loader import loadURDF
from modern_robotics import *

urdf_name = "ur5/ur5.urdf"
M, Slist, Blist, Mlist, Glist, actuated_joints_num = loadURDF(urdf_name)

thetalist = np.array(
    [np.pi / 3.0, np.pi / 3.0, np.pi / 3.0, np.pi / 3.0, np.pi / 3.0, np.pi / 3.0]
).T
dthetalist = np.array([0, 0, 0, 0, 0, 0]).T
ddthetalist = np.array([0, 0, 0, 0, 0, 0]).T
g = np.array([0, 0, -9.8]).T
Ftip = np.array([0, 0, 0, 0, 0, 0]).T


# print(M)
print(Mlist)
print(Glist)
print("FKinSpace\n", FKinSpace(M, Slist, thetalist))
print("FKinBody\n", FKinBody(M, Blist, thetalist))

print("JacobianSpace\n", JacobianSpace(Slist, thetalist))
print("JacobianBody\n", JacobianBody(Blist, thetalist))

print(
    "InverseDynamics\n",
    InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist),
)
