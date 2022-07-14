import pybullet as p
import time
import math
import pybullet_data


physicsClient = p.connect(p.GUI) # start client

# Setup search path and environment
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # additional search path for pybullet urdfs
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

# Add sliders for debugging and giving desired point coords
x_target = p.addUserDebugParameter('x_d', rangeMin=-3, rangeMax=3, startValue=0)
y_target = p.addUserDebugParameter('y_d', rangeMin=-3, rangeMax=3, startValue=0)
z_target = p.addUserDebugParameter('z_d', rangeMin=0, rangeMax=6, startValue=0)

# Set start position and orientation
startPos = [0, 0, 2.2]
startOrientation = p.getQuaternionFromEuler([0,0,0])


# Load robot and set robot parameters
robotId = p.loadURDF("urdf_files/dodo_simple.urdf", startPos, startOrientation, useFixedBase=True)

numJoints = p.getNumJoints(robotId)
robotEndEffectorIndex = numJoints-1 # link counting starts from 0, base link doesn't count


# Set Pos and Orn of center of mass frame
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)


while 1:
    # Step simulation
    p.stepSimulation()
    time.sleep(1/240)

    # Read target end effector position
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
        maxNumIterations=100,
        solver=0,
        )

    # Go through all joints except the final fixed one and set joint angles
    for jointIdx in range(numJoints-1):
       p.resetJointState(robotId, jointIdx, jointPoses[jointIdx])