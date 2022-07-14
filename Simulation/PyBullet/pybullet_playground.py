import pybullet as p
import time
import pybullet_data


physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 4]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("urdf_files/dodo_simple_vertical_test_stand.urdf", startPos, startOrientation, useFixedBase=True)

# set the center of mass frame
p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# Start simulation
while 1:
    p.stepSimulation()
    time.sleep(1/240)
    # if i % 100 == 0:
    #     print('==================')
    #     print(p.getJointState(boxId, 0))
    #     print(p.getJointState(boxId, 1))



cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
p.disconnect()