from mr_urdf_loader import loadURDF
from modern_robotics import *

urdf_name = "3DoF.urdf"
M, Slist, Blist, Mlist, Glist, robot = loadURDF(urdf_name)

thetalist = np.array([0, 0, np.pi / 2.0])
dthetalist = np.array([0, 0, 0.1])
ddthetalist = np.array([0, 0, 0])
g = np.array([0, 0, -9.8])
Ftip = [0, 0, 0, 0, 0, 0]


print("FKinSpace\n", FKinSpace(M, Slist, thetalist))
print("FKinBody\n", FKinBody(M, Blist, thetalist))

print("JacobianSpace\n", JacobianSpace(Slist, thetalist))
print("JacobianBody\n", JacobianBody(Blist, thetalist))

print(
    "InverseDynamics\n",
    InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist),
)
