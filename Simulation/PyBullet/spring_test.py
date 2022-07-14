""" Our test for using a spring. We want to approximate a spring two external forces
applied to the two links the spring is supposed to be connected to. PyBullet allows
for deformable bodies too but for our purposes that is too complex.

"""
import pybullet as p
import time
import math
import pybullet_data

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Setup environment
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")


# Set parameters
startPos = [0, 0, 2.5]
startOrientation = p.getQuaternionFromEuler([0,0,0])

ballStartPos = [0, 5, 0.3]
ballStartOrientation = p.getQuaternionFromEuler([0,0,0])

spring_coeff = 10 # spring coefficient in [N/degrees]

counter = 0 # set counter to visualize sim parameters every k'th iter


# External force to debug
torqueId = p.addUserDebugParameter('torque', rangeMin=-2000, rangeMax=2000, startValue=0) # [Nm]

# Load objects
robotId = p.loadURDF("urdf_files/dodo_simple.urdf", startPos, startOrientation, useFixedBase=True)

numJoints = p.getNumJoints(robotId)
robotEndEffectorIndex = 2 # link counting starts from 0, base link doesn't count


# Set center of mass frame (loadURDF sets the pos and orientation of link frame)
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)


while 1:
    # Step simulation and pause
    p.stepSimulation()
    time.sleep(1/240)


    # Get current joint angle and calculate spring torque
    jointAngle = p.getJointState(bodyUniqueId=robotId, jointIndex=1)[0] # this returns a list, we need 1st element
    jointAngle = jointAngle * 180 / math.pi # convert rad to degrees
    torque = - jointAngle * spring_coeff

    # Set input, action and reaction torques
    torque_in = p.readUserDebugParameter(torqueId) # read input torque
    torque_ext = [torque_in, 0, 0] # create torque vector
    torque_act = [torque, 0, 0]
    torque_react = [-torque, 0, 0]
    

    # Apply torques on opposing links
    p.applyExternalTorque(
        objectUniqueId=robotId,
        linkIndex=0,
        torqueObj=torque_act,
        flags=p.WORLD_FRAME, # Joint axes are on the x-axis planar robot
    )

    p.applyExternalTorque(
    objectUniqueId=robotId,
    linkIndex=1,
    torqueObj=torque_react,
    flags=p.WORLD_FRAME,
    )

    p.applyExternalTorque(
    objectUniqueId=robotId,
    linkIndex=1,
    torqueObj=torque_ext,
    flags=p.WORLD_FRAME,
    )

    # Print stuff for debugging
    if counter % 100 == 0:
        #print(p.getLinkState(robotId, robotEndEffectorIndex)[0])
        #print(jointAngle)
        counter = 0 # reset counter

    counter += 1
     