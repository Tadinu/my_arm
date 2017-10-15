import pybullet as p
import numpy as np
import copy
from copy import deepcopy
import collections as col
import math
from math import degrees
from math import pi

class Kuka:

  def __init__(self, urdfRootPath='', timeStep=0.01, inverseKinematics=1):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    
    self.maxForce = 200.
    self.fingerAForce = 6
    self.fingerBForce = 5.5
    self.fingerTipForce = 6
    self.useInverseKinematics = inverseKinematics
    self.useSimulation = 1
    self.useNullSpace = 1
    self.useOrientation = 1
    self.kukaEndEffectorIndex = 6
    #lower limits for null space
    self.ll=[-.967,-2 ,-2.96,0.19,-2.96,-2.09,-3.05]
    #upper limits for null space
    self.ul=[.967,2 ,2.96,2.29,2.96,2.09,3.05]
    #joint ranges for null space
    self.jr=[5.8,4,5.8,4,5.8,4,6]
    #restposes for null space
    self.rp=[0,0,0,0.5*math.pi,0,-math.pi*0.5*0.66,0]
    #joint damping coefficents
    self.jd=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    self.reset()
    
  def reset(self):
    objects = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")
    self.kukaUid = objects[0]
    #for i in range (p.getNumJoints(self.kukaUid)):
    #  print(p.getJointInfo(self.kukaUid,i))
    p.resetBasePositionAndOrientation(self.kukaUid,[-0.100000,0.000000,0.070000],[0.000000,0.000000,0.000000,1.000000])
    self.jointPositions=[ 0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048, -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200 ]
    self.numJoints = p.getNumJoints(self.kukaUid)
    for jointIndex in range (self.numJoints):
      p.resetJointState(self.kukaUid,jointIndex,self.jointPositions[jointIndex])
      p.setJointMotorControl2(self.kukaUid,jointIndex,p.POSITION_CONTROL,targetPosition=self.jointPositions[jointIndex],force=self.maxForce)
    
    self.trayUid = p.loadURDF("tray/tray.urdf", 0.640000,0.075000,-0.190000,0.000000,0.000000,1.000000,0.000000)
    self.endEffectorPos = [0.537,0.0,0.5]
    self.endEffectorAngle = 0
    
    
    self.motorNames = []
    self.motorIndices = []
    
    for i in range (self.numJoints):
      jointInfo = p.getJointInfo(self.kukaUid,i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        #print("motorname")
        #print(jointInfo[1])
        self.motorNames.append(str(jointInfo[1]))
        self.motorIndices.append(i)

  def getActionDimension(self):
    if (self.useInverseKinematics):
      return len(self.motorIndices)
    return 6 #position x,y,z and roll/pitch/yaw euler angles of end effector

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    # Joint State
    for i in range (self.numJoints-7):
        jointState = self.getJointState(i)
        observation.append(np.array(jointState[0], dtype=np.float32)) # Pos
        observation.append(np.array(jointState[1], dtype=np.float32)) # Vel

    #print ('KUKA OBSERVATION ONLY', self.numJoints, observation)

    # Link State
    #state = p.getLinkState(self.kukaUid,self.kukaEndEffectorIndex)
    #pos = state[0]
    #orn = state[1]
    #euler = p.getEulerFromQuaternion(orn)
    #
    #observation.extend(list(pos))
    #observation.extend(list(euler))

    return observation

  ## Output: returns the position list of 3 floats and orientation as list of 4 floats in [x,y,z,w] order.
  def getBasePosOrient(self):
    return p.getBasePositionAndOrientation(self.kukaUid)

  def getDynamicsInfo(self):
    return p.getDynamicsInfo(self.kukaUid)

  def getNumConstraints(self):
    return p.getNumConstraints(self.kukaUid)

  def getConstraintUniqueId(self, serialIndex):
    return p.getConstraintUniqueId(self.kukaUid, serialIndex)

  def getConstraintInfo(self, constraintUniqueId):
    return p.getConstraintInfo(self.kukaUid, constraintUniqueId)

  ## Output :Tuple(jointIndex,
  ##               jointName,
  ##               jointType,
  ##               qIndex,
  ##               uIndex,
  ##               flags,
  ##               jointDamping,
  ##               jointFriction,
  ##               jointLowerLimit,
  ##               jointUpperLimit,
  ##               jointMaxForce,
  ##               jointMaxVelocity,
  ##               linkName)
  ##
  def getJointInfo(self, jointIndex):
    if(jointIndex > -1 and jointIndex < self.numJoints):
        return p.getJointInfo(self.kukaUid, jointIndex)

  ## Output: Joint min, max limit
  ##
  def getJointLimit(self, jointIndex):
    jointInfo  = self.getJointInfo(jointIndex)
    jointLimit = [jointInfo[8], jointInfo[9]]
    return jointLimit

  ## Output: Tuple (float jointPosition, float jointVelocity,
  ##                jointReactionForces(tuple 6 floats),
  ##                float appliedJointMotorTorque)
  def getJointState(self, jointIndex):
    return p.getJointState(self.kukaUid, jointIndex)

  def getJointStates(self, jointIndices):
    return p.getJointStates(self.kukaUid, jointIndices)

  def moveJointTo(self, jointIndex, jointPos):
    #p.setJointMotorControl2(bodyIndex=self.kukaUid,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[i],targetVelocity=0,force=self.maxForce,positionGain=0.03,velocityGain=1)
    p.setJointMotorControl2(self.kukaUid, jointIndex, p.POSITION_CONTROL, jointPos, 0, self.maxForce, 0.03, 1)

  def moveJoint(self, jointIndex, jointDeltaPos):
      curJointState = self.getJointState(0)
      curJointPos = curJointState[0]
      p.setJointMotorControl2(self.kukaUid, jointIndex, p.POSITION_CONTROL, curJointPos + jointDeltaPos, 0, self.maxForce, 0.03, 1)

  ## Ouput: Tuple (linkWorldPosition             (vec3, list of 3 floats)
  ##               linkWorldOrientation          (vec4, list of 4 floats)
  ##               localInertialFramePosition    (vec3, list of 3 floats)
  ##               localInertialFrameOrientation (vec4, list of 4 floats)
  ##               worldLinkFramePosition        (vec3, list of 3 floats)
  ##               worldLinkFrameOrientation     (vec4, list of 4 floats)
  ##               worldLinkLinearVelocity       (vec3, list of 3 floats)
  ##               worldLinkAngularVelocity      (vec3, list of 3 floats)
  ##              )
  def getLinkState(self, linkIndex):
    return p.getLinkState(self.kukaUid, linkIndex, 1) ## 1: Compute Link Velocity

  def detectCollision(self, objectId1, objectId2):
      # To compute closest points of objects within an arbitrary distance.
      # --> Collision within a given distance!
      closestPoints = p.getClosestPoints(objectId1, objectId2, 0.01)
      numPt = len(closestPoints)
      #if(numPt > 0):
      #  print ("CLOSEST POINTS : ", numPt, " - DISTANCE : ", closestPoints[0][8])
      return (numPt > 0)

  def detectCollisionWith(self, objectId):
      return self.detectCollision(self.kukaUid, objectId)

  def getShortestDistanceToObjects(self, objectIdList):
    dist = 0.01
    for i in range(len(objectIdList)):
      closestPoints = p.getClosestPoints(self.kukaUid, objectIdList[i], 0.01)
      for j in range(len(closestPoints)):
        #contactDistance = closestPoints[j][8]
        contactDistance = abs(closestPoints[j][5][2] - closestPoints[j][6][2])
        if ((contactDistance > 0) and (contactDistance < dist)): # contact distance(+ for separation, - for penetration)
          dist = contactDistance

    return dist

  def distanceFromEndTipToObj(self, objectId):
      pos, orn = p.getBasePositionAndOrientation(objectId)
      endTipPos = self.getEndEffectorPos()
      #print('END TIP POS', endTipPos)
      #linearVel, angularVel = p.getBaseVelocity()
      dist = math.sqrt((pos[0] - endTipPos[0])**2 +
                       (pos[1] - endTipPos[1])**2 +
                       (pos[2] - endTipPos[2])**2
                       )
      #print('DIST', dist)
      return dist

  def getEndEffectorPos(self):
      state = p.getLinkState(self.kukaUid,self.kukaEndEffectorIndex)
      actualEndEffectorPos = state[0]
      return actualEndEffectorPos

  def detectEndEffectorHitObj(self, objectId):
      contactPoints = p.getContactPoints(self.kukaUid, objectId, self.kukaEndEffectorIndex, 0)
      return (len(contactPoints) > 0)

  def applyAction(self, motorCommands):
    
    #print ("self.numJoints")
    #print (self.numJoints)
    if (self.useInverseKinematics):
      
      dx = motorCommands[0]
      dy = motorCommands[1]
      dz = motorCommands[2]
      da = motorCommands[3]
      fingerAngle = motorCommands[4]
      
      state = p.getLinkState(self.kukaUid,self.kukaEndEffectorIndex)
      actualEndEffectorPos = state[0]
      #print("pos[2] (getLinkState(kukaEndEffectorIndex)")
      #print(actualEndEffectorPos[2])

      self.endEffectorPos[0] = self.endEffectorPos[0]+dx
      if (self.endEffectorPos[0]>0.75):
        self.endEffectorPos[0]=0.75
      if (self.endEffectorPos[0]<0.45):
        self.endEffectorPos[0]=0.45
      self.endEffectorPos[1] = self.endEffectorPos[1]+dy
      if (self.endEffectorPos[1]<-0.22):
        self.endEffectorPos[1]=-0.22
      if (self.endEffectorPos[1]>0.22):
        self.endEffectorPos[1]=0.22
      
      #print ("self.endEffectorPos[2]")
      #print (self.endEffectorPos[2])
      #print("actualEndEffectorPos[2]")
      #print(actualEndEffectorPos[2])
      if (dz>0 or actualEndEffectorPos[2]>0.10):
        self.endEffectorPos[2] = self.endEffectorPos[2]+dz
      if (actualEndEffectorPos[2]<0.10):
        self.endEffectorPos[2] = self.endEffectorPos[2]+0.0001
    
     
      self.endEffectorAngle = self.endEffectorAngle + da
      pos = self.endEffectorPos
      orn = p.getQuaternionFromEuler([0,-math.pi,0]) # -math.pi,yaw])
      if (self.useNullSpace==1):
        if (self.useOrientation==1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,pos,orn,self.ll,self.ul,self.jr,self.rp)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,pos,lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr, restPoses=self.rp)
      else:
        if (self.useOrientation==1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,pos,orn,jointDamping=self.jd)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,pos)
    
      #print("jointPoses")
      #print(jointPoses)
      #print("self.kukaEndEffectorIndex")
      #print(self.kukaEndEffectorIndex)
      if (self.useSimulation):
        for i in range (self.kukaEndEffectorIndex+1):
          #print(i)
          p.setJointMotorControl2(bodyIndex=self.kukaUid,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[i],targetVelocity=0,force=self.maxForce,positionGain=0.03,velocityGain=1)
      else:
        #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range (self.numJoints):
          p.resetJointState(self.kukaUid,i,jointPoses[i])
      #fingers
      p.setJointMotorControl2(self.kukaUid,7,p.POSITION_CONTROL,targetPosition=self.endEffectorAngle,force=self.maxForce)
      p.setJointMotorControl2(self.kukaUid,8,p.POSITION_CONTROL,targetPosition=-fingerAngle,force=self.fingerAForce)
      p.setJointMotorControl2(self.kukaUid,11,p.POSITION_CONTROL,targetPosition=fingerAngle,force=self.fingerBForce)
      
      p.setJointMotorControl2(self.kukaUid,10,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)
      p.setJointMotorControl2(self.kukaUid,13,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)

    else:
      #print('MOTOR COMMANDS', motorCommands)
      for action in range (len(motorCommands)):
        motor = self.motorIndices[action]
        p.setJointMotorControl2(self.kukaUid,motor,p.POSITION_CONTROL,targetPosition=motorCommands[action],force=self.maxForce)
      
