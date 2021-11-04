from os import O_EXCL
import pybullet as p
import time
import pybullet_data
import math
import matplotlib.pyplot as plt
import numpy as np


## Setup Simulator GUI
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

## Setup Physics
p.setGravity(0,0,-10)

## Load Plane
planeId = p.loadURDF("plane.urdf")

## Load Robot
startPos = [0,0,0.0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("husky/husky.urdf",startPos, startOrientation)
# boxId = p.loadURDF("biped/biped2d_pybullet.urdf",startPos, startOrientation)

## Get Joints Info
number_of_joints = p.getNumJoints(boxId)
for joint_number in range(0, number_of_joints):
    info = p.getJointInfo(boxId, joint_number)
    print(info[0], ": ", info[1])
    print("Link name: ", info[12])

## variable init for stored value 
P_x = []
P_y = []
P_z = []
O_x = []
O_y = []
O_z = []
O_w = []

for i in range(300):
	## Read Sensors or Link Information
    (linkWorldPosition,
        linkWorldOrientation,
        localInertialFramePosition,
        localInertialFrameOrientation,
        worldLinkFramePosition,
        worldLinkFrameOrientation,
        worldLinkLinearVelocity,
        worldLinkAngularVelocity) = p.getLinkState(boxId,0, computeLinkVelocity=1, computeForwardKinematics=1)
    print("Position:", linkWorldPosition)
    print("Orientation:", linkWorldOrientation)

    ## Send Commands to Actuator
    p.setJointMotorControl2(boxId,2,p.VELOCITY_CONTROL,targetVelocity=10,force=1000)
    p.setJointMotorControl2(boxId,3,p.VELOCITY_CONTROL,targetVelocity=10,force=1000)
    p.setJointMotorControl2(boxId,4,p.VELOCITY_CONTROL,targetVelocity=10,force=1000)
    p.setJointMotorControl2(boxId,5,p.VELOCITY_CONTROL,targetVelocity=10,force=1000)
    
    ## Update Simulations
    p.stepSimulation()
    time.sleep(1./240.)

    #Store Position Value and Orientation Value 
    P_x.append(linkWorldPosition[0])
    P_y.append(linkWorldPosition[1])
    P_z.append(linkWorldPosition[2])
    O_x.append(linkWorldOrientation[0])
    O_y.append(linkWorldOrientation[1])
    O_z.append(linkWorldOrientation[2])
    O_w.append(linkWorldOrientation[3])

print("\n")

def R_matrix(q): ## Rot-Matrix from Quaternion
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    m00 = 2 * (x * x + y * y) - 1
    m01 = 2 * (y * z - x * w)
    m02 = 2 * (y * w + x * z)
    m10 = 2 * (y * z + x * w)
    m11 = 2 * (x * x + z * z) - 1
    m12 = 2 * (z * w - x * y)
    m20 = 2 * (y * w - x * z)
    m21 = 2 * (z * w + x * y)
    m22 = 2 * (x * x + w * w) - 1

    R_matrix = np.array([[m00, m01, m02],[m10, m11, m12],[m20, m21, m22]])
    print("Rotation Matrix : \n", R_matrix,"\n") 
    return R_matrix

def Euler(q): ## Quaternion to Euler source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    e0 = +2.0 * (w * x + y * z)
    e1 = +1.0 - 2.0 * (x * x + y * y)
    alfa_x = math.atan2(e0, e1)

    e2 = +2.0 * (w * y - z * x)
    e2 = +1.0 if e2 > +1.0 else e2
    e2 = -1.0 if e2 < -1.0 else e2
    beta_y = math.asin(e2)

    e3 = +2.0 * (w * z + x * y)
    e4 = +1.0 - 2.0 * (y * y + z * z)
    gammma_z = math.atan2(e3, e4)

    print("Euler : \n [", alfa_x, beta_y, gammma_z," ]\n")
    return alfa_x, beta_y, gammma_z

def RPY(q): ## Roll-Pitch-Yaw source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    print("Roll, Pitch, Yaw : \n [", roll_x, pitch_y, yaw_z," ]\n")
    return roll_x, pitch_y, yaw_z

def Angel_ax(q): ## Angle-axis from Quaternion
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    theta=2*math.acos(x)
    w_x=y/(math.sin(theta/2))
    w_y=z/(math.sin(theta/2))
    w_z=w/(math.sin(theta/2))
    print("Angle Axis : \n [", w_x, w_y, w_z," ]\n")
    return w_x, w_y, w_z

## Plot Transformation
Euler(linkWorldOrientation)
RPY(linkWorldOrientation)
Angel_ax(linkWorldOrientation)
R_matrix(linkWorldOrientation) 

# Plot Position and Orientation 
plt.figure(1)
plt.suptitle('POSITION in X')
plt.plot(P_x)
plt.figure(2)
plt.suptitle('POSISITON in Y')
plt.plot(P_y)
plt.figure(3)
plt.suptitle('POSISITON in Z')
plt.plot(P_z)

plt.figure(4)
plt.suptitle('ORIENTATION in X')
plt.plot(O_x)
plt.figure(5)
plt.suptitle('ORIENTATION in Z')
plt.plot(O_y)
plt.figure(6)
plt.suptitle('ORIENTATION in Y')
plt.plot(O_z)
plt.figure(7)
plt.suptitle('ORIENTATION in W')
plt.plot(O_w)
plt.show()