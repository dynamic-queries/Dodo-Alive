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
body_height = p.addUserDebugParameter('z_body', rangeMin=-1, rangeMax=10, startValue=2)

# Set start position and orientation
startPos = [0, 0, 2.2]
startOrientation = p.getQuaternionFromEuler([0,0,0])


# Load robot and set robot parameters
robotId = p.loadURDF("urdf_files/dodo_simple_vertical_test_stand.urdf", startPos, startOrientation, useFixedBase=True)

numJoints = p.getNumJoints(robotId)
robotEndEffectorIndex = numJoints-1 # link counting starts from 0, base link doesn't count


# Set Pos and Orn of center of mass frame
# TODO: figure out if this makes sense, maybe something is messed up with urdf files?
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)


while 1:
    # Step simulation
    p.stepSimulation()
    time.sleep(1/240)

    # Read target body position
    targetPos = p.readUserDebugParameter(body_height)
  

    # Set joint state for body (link 0 since baselink in this sim is the vertical slider)
    p.resetJointState(robotId, 0, targetPos)