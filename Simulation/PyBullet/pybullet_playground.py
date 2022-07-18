"""Our test file for seeing how position control works"""

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

maxForce = 0 # max motor force

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
    time.sleep(1/120)

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

    # Set control
    for jointIdx in range(numJoints-1): # go through all joints except final fixed one
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=jointIdx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=jointPoses[jointIdx],
            force=maxForce
        )

    if counter % 250 == 0:
        print(p.getLinkState(robotId, robotEndEffectorIndex)[0])
        counter = 0 # reset counter

    counter += 1
     