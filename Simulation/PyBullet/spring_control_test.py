"""Our test file for seeing how position control works with the spring added.
This is for a single double pendulum with a spring between the two links.
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

maxForce = 1000 # max motor force
spring_coeff = 5 # spring coefficient in [N/degrees]

counter = 0 # set counter to visualize sim parameters every k'th iter

# Add sliders for debugging and giving desired point coords
x_target = p.addUserDebugParameter('x_d', rangeMin=-1.6, rangeMax=1.6, startValue=0)
y_target = p.addUserDebugParameter('y_d', rangeMin=-1.6, rangeMax=1.6, startValue=0)
z_target = p.addUserDebugParameter('z_d', rangeMin=0, rangeMax=3, startValue=0)


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

    targetPos = [
        p.readUserDebugParameter(x_target),
        p.readUserDebugParameter(y_target),
        p.readUserDebugParameter(z_target)
        ]
  
    # Calculate inverse kinematics
    jointPoses = p.calculateInverseKinematics(
        robotId,
        robotEndEffectorIndex,
        targetPos,
        maxNumIterations=200
        )

    # Get current joint angle and calculate spring torque
    jointAngle = p.getJointState(bodyUniqueId=robotId, jointIndex=1)[0] * 180 / math.pi # convert rad to degrees
    torque = -jointAngle * spring_coeff

    # Set action and reaction torques
    torque_act = [torque, 0, 0]
    torque_react = [-torque, 0, 0]



    # Set control
    for jointIdx in range(numJoints-1): # go through all joints except final fixed one
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=jointIdx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=jointPoses[jointIdx],
            force=maxForce
        )

    # Apply torques on opposing links
    p.applyExternalTorque(
        objectUniqueId=robotId,
        linkIndex=0,
        torqueObj=torque_react,
        flags=p.WORLD_FRAME, # Joint axes are on the x-axis planar robot
    )

    p.applyExternalTorque(
    objectUniqueId=robotId,
    linkIndex=1,
    torqueObj=torque_act,
    flags=p.WORLD_FRAME,
    )


    if counter % 250 == 0:
        print(p.getLinkState(robotId, robotEndEffectorIndex)[0])
        counter = 0 # reset counter

    counter += 1
     