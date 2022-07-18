"""
Our test case for the squat motion to determine whether the robot can lift it's own weight. 
Parameters aren't implemented yet.

Since we have equal leg lengths the geometry works out nicely and the angle from the body
to the first link is the same as the angle measured from the ground to the last link 
(with the toe fixed to the ground).

The height of the body turns out to be ==> h = 2*l*sin(alpha) where l is the link length
and alpha is the angle defined earlier. This allows us to specify the body height and calculate
joint angles. Who needs the inverse kinematics functionality?

TODO: figure out how to make the first joint not be a free joint the joints are all coupled
and I'm using a prismatic joint to prevent the body from rotating around. But we need to be able to know
the hip joint torques. If I remove the prismatic joint the body just falls down :(

NOTE: maybe my approach of calculating the angles as the simulation goes will work, we
need to figure out the joint torque at the hip to keep the body from pitching forwards.
"""






import pybullet as p
import time
import math
import pybullet_data


physicsClient = p.connect(p.GUI) # start client

# Setup search path and environment
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # additional search path for pybullet urdfs
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

# Set parameters
startPos = [0, 0, 3]
startOrientation = p.getQuaternionFromEuler([0,0,0])
maxForce = 2000 # max force that can be applied by motors
length = 1 # [m] length of each link 

# Add sliders for debugging and giving desired point coords
body_height = p.addUserDebugParameter('z_body', rangeMin=0, rangeMax=2*length, startValue=2*length) # these ranges are important otherwise arcsin fails
joint_0 = p.addUserDebugParameter('q0', rangeMin=-math.pi, rangeMax=math.pi, startValue=0) 
joint_1 = p.addUserDebugParameter('q1', rangeMin=-math.pi, rangeMax=math.pi, startValue=0) 
joint_2 = p.addUserDebugParameter('q2', rangeMin=-math.pi, rangeMax=math.pi, startValue=0) 
                                                                                            

# Load robot and set robot parameters
robotId = p.loadURDF("urdf_files/dodo_simple.urdf", startPos, startOrientation, useFixedBase=False)

numJoints = p.getNumJoints(robotId)
robotEndEffectorIndex = numJoints-1 # link counting starts from 0, base link doesn't count


# Set position and orientation of center of mass frame
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)

# Fix the toe of the robot to the ground
p.createConstraint(
    parentBodyUniqueId=robotId,
    parentLinkIndex=robotEndEffectorIndex,
    childBodyUniqueId=-1, # means it's the world frame
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0,0,0], # it's a fixed joint this doesn't matter
    parentFramePosition=[0,0,0], # position of joint frame relative to parent frame
    childFramePosition=[0,0,0], # origin of world frame
    )

# Make sure the body can only move up and down
#p.createConstraint(
#     parentBodyUniqueId=robotId,
#     parentLinkIndex=-1, # means it's the base link
#     childBodyUniqueId=-1,
#     childLinkIndex=-1,
#     jointType=p.JOINT_PRISMATIC,
#     jointAxis=[0,0,1], # joint axis is the z (vertical) axis
#     parentFramePosition=[0,0,0], # position of joint frame relative to parent frame
#     childFramePosition=[0,0,0], # origin of world frame
# )

# Final joint is supposed to be a free joint set control for it
p.setJointMotorControl2(
    bodyIndex=robotId,
    jointIndex=robotEndEffectorIndex,
    controlMode=p.POSITION_CONTROL,
    targetPosition=0,
    force=0 # zero force means it's a free joint
)

while 1:
    # Step simulation
    p.stepSimulation()
    time.sleep(1/240)


    
    # Read target body position
    targetPos = p.readUserDebugParameter(body_height)  # desired height of the body of the robot
  
    # Calculate required joint angles
    alpha = math.asin(targetPos/(2*length)) # angle from ground to last link
    beta = alpha # angle from body to first link, triangle is isoceles triangle so == alpha
    gamma = alpha + beta # angle between first and second link
    #print(alpha)

    jointPoses = [math.pi/2 - beta, math.pi/2 - gamma, alpha] # actual joint angles, notation is different
    #jointPoses = [p.readUserDebugParameter(joint_0), p.readUserDebugParameter(joint_1), p.readUserDebugParameter(joint_2)] 

    # Set control 
    for jointIdx in range(numJoints-1): # go through all joints except final toe joint (it's a free joint)
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=jointIdx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=jointPoses[jointIdx],
            force=maxForce
        )
    # p.setJointMotorControl2(
    # bodyIndex=robotId,
    # jointIndex=0,
    # controlMode=p.POSITION_CONTROL,
    # targetPosition=0,
    # force=0 # zero force means it's a free joint
    # )

    # p.setJointMotorControl2(
    # bodyIndex=robotId,
    # jointIndex=1,
    # controlMode=p.POSITION_CONTROL,
    # targetPosition=jointPoses[1], 
    # force=maxForce # zero force means it's a free joint
    # )
    
