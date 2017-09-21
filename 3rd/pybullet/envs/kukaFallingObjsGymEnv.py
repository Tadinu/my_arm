import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
from . import kuka
import random

from threading import Timer
## threading.Timer(1, self.timerTask).start()

from kukaBot import KukaBot
from menace_object import MenaceObject


KEY_UP    = 65297
KEY_DOWN  = 65298
KEY_LEFT  = 65296
KEY_RIGHT = 65295

KEY_A = 65307
KEY_Q = 65306
KEY_D = 110
KEY_E = 112
KEY_Z = 117

KEY_S = 121
KEY_W = 104
KEY_F = 106
KEY_R = 107
KEY_X = 108

JOINT_KEYS_DICT = {}
##p.createCollisionShape(p.GEOM_PLANE)
##p.createMultiBody(0,0)

class KukaFallingObjsGymEnv(gym.Env):
  metadata = {
      'render.modes': ['human', 'rgb_array'],
      'video.frames_per_second' : 50
  }

  def __init__(self,
               urdfRoot="",
               actionRepeat=1,
               isEnableSelfCollision=True,
               renders=True):
    print("init")
    self._timeStep = 1./240.
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._observation = []
    self._envStepCounter = 0
    self._renders = renders
    self._p = p

    if self._renders:
      p.connect(p.GUI)
      p.resetDebugVisualizerCamera(1.3,180,-41,[0.52,-0.2,-0.33])
    else:
      p.connect(p.DIRECT)
    #timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")
    self._seed()

    # INITIALIZE ENVIRONMENT
    self.loadEnvironmentObjects()

    # Reset
    self.reset() ## --> Call self._reset() defined  below!

    # After loading the robot!
    observationDim = len(self.getExtendedObservation())
    #print("observationDim")
    #print(observationDim)

    observation_high = np.array([np.finfo(np.float32).max] * observationDim)
    self.action_space = spaces.Discrete(2)
    self.observation_space = spaces.Box(-observation_high, observation_high)
    self.viewer = None

    ## DEBUG STUFFS --
    self.initializeDebugShapes()

    ## ducta++
    self.printMotorNames()
    JOINT_KEYS_DICT[0] = [KEY_LEFT, KEY_RIGHT]
    JOINT_KEYS_DICT[1] = [KEY_DOWN, KEY_UP]
    JOINT_KEYS_DICT[2] = [KEY_A, KEY_S]
    JOINT_KEYS_DICT[3] = [KEY_Q, KEY_W]
    JOINT_KEYS_DICT[4] = [KEY_D, KEY_F]
    JOINT_KEYS_DICT[5] = [KEY_E, KEY_R]
    JOINT_KEYS_DICT[6] = [KEY_Z, KEY_X]

    #print ("JOINT KEY DICTS:", len(JOINT_KEYS_DICT)," --", JOINT_KEYS_DICT, )

  def loadEnvironmentObjects(self):
      ## FALLING OBJECTS --
      self._CNUM_OBJS = 5
      self._objs = []
      self._sphereUid = self._cubeUId = -1
      self.__reward = 0
      self._CKUKA_MOVE_INTERVAL = 0.5
      #self._timer = Timer(1, self.timerTask)

      ## SETUP ENVIRONMENT --
      p.resetSimulation()
      p.setPhysicsEngineParameter(numSolverIterations=150)
      p.setTimeStep(self._timeStep)
      p.setGravity(0,0,-10)

      ## LOAD GROUND --
      self._groundUId = p.loadURDF("%splane.urdf" % self._urdfRoot,[0,0,-1])
      #print("GROUND ID: ", self._groundUId)

      ## LOAD TABLE --
      self._tableUId = p.loadURDF("table/table.urdf", 0.5000000,0.00000,-.820000,0.000000,0.000000,0.0,1.0)
      #print("TABLE ID: ", self._tableUId)

      ## LOAD MANIPULATOR KUKA --
      self._kuka = kuka.Kuka(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
      kukaBasePos, kukaBaseOrn = self._kuka.getBasePosOrient()
      #print("KUKA BASE POS:", kukaBasePos)

      # Initialize the bot
      self._kukaBot = KukaBot()

  def _reset(self):
    print("_reset")
    self._terminated = 0

    ## LOAD FALLING OBJS --
    self.reloadFallingObjects()

    ## STEP SIMULATION --
    self._envStepCounter = 0
    p.stepSimulation()

    ## OBSERVATION --
    self._observation = self.getExtendedObservation()

    ## ducta--
    #self._timer.start()
    return np.array(self._observation)

  def reloadFallingObjects(self):
      # Remove debug shapes
      #self.removeObject(self._sphereUid)

      # Replace with the news ones
      for x in self._objs: self.removeObject(x.id())
      for x in self._objs: del x
      #self._objs = [MenaceObject(self) for i in range(self._kuka.numJoints)]
      self._objs = [MenaceObject(self) for i in range(self._CNUM_OBJS)]
      for i in range(len(self._objs)):
          linkState = self._kuka.getLinkState(i+2)
          xpos = linkState[4][0]
          ypos = linkState[4][1]
          zpos = 3 + 5 * random.random()
          ang = 3.1415925438*random.random()
          orn = p.getQuaternionFromEuler([0,0,ang])
          self._objs[i].load("sphere_5cm.urdf", xpos,ypos,zpos,orn[0],orn[1],orn[2],orn[3])

  def removeObject(self, objectUId):
      p.removeBody(objectUId)

  def printMotorNames(self):
      motorsIds=[]
      for i in range (len(self._kuka.motorNames)):
          motor = self._kuka.motorNames[i]
          motorJointIndex = self._kuka.motorIndices[i]
          motorsIds.append(self._p.addUserDebugParameter(motor,-3,3,self._kuka.jointPositions[i]))
          print(motor, '-', motorJointIndex)
          #print(motor, self._kuka.getJointInfo(motorJointIndex))

  # This requires a physics server to be run!
  def printConstraintInfo(self):
      constraintNum = self._kuka.getNumConstraints()
      for i in range(constraintNum):
          constraintUniqueId = self._kuka.getConstraintUniqueId(i)
          print("Constraint Unique Id:", constraintUniqueId)
          print("Constraint Info:", self._kuka.getConstraintInfo(constraintUniqueId))

  #def timerTask(self):
  #    #print('TIMER TASK')
  #    for x in self._objs:
  #        x.timerTask()

  def __del__(self):
    p.disconnect()

  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def getExtendedObservation(self):
    self._observation = self._kuka.getObservation()

    baseJointState  = self._kuka.getJointState(0)
    baseJointInfo   = []
    baseJointInfo.append(baseJointState[0])
    baseJointInfo.append(baseJointState[1])

    objPos = []
    objOrn = []
    for i in range(len(self._objs)):
      pos, orn = p.getBasePositionAndOrientation(self._objs[i].id())
      objPos.append(pos)
      objOrn.append(orn)

    #if(len(self._objs) > 0):
    #    self._observati`on.extend(objPos)
    #self._observation.extend(list(baseJointInfo))

    return self._observation

  def updateReward(self):
      #print("updateReward:", isReward)
      for x in self._objs:
          self.__reward += 1 if x.isReward() else -1

#ducta ++
  def controlRobotByKeyboard(self):
      self._kuka.useInverseKinematics = 0
      #realAction = self.createRobotAction(action)

      jointPoses   = []
      jointLimits  = []
      for i in range (len(JOINT_KEYS_DICT)):
          jointPoses.append(self._kuka.getJointState(i)[0]) # 0: Joint Pos
          jointLimits.append(self._kuka.getJointLimit(i))

      while True:
          ########################################################################################################
          # KEYBOARD HANDLING --
          #
          keys = self._p.getKeyboardEvents()
          #print("Keys state:", keys)

          for i in range(len(JOINT_KEYS_DICT)):
              #
              isMoveJoint = (JOINT_KEYS_DICT[i][0] in keys) or (JOINT_KEYS_DICT[i][1] in keys)
              #
              if (JOINT_KEYS_DICT[i][0] in keys):
                  jointPoses[i] -= 0.05
                  if(jointPoses[i] < jointLimits[i][0]): jointPoses[i] = jointLimits[i][0]
              elif (JOINT_KEYS_DICT[i][1] in keys):
                  jointPoses[i] += 0.05
                  if(jointPoses[i] > jointLimits[i][1]): jointPoses[i] = jointLimits[i][1]

              if isMoveJoint:
                  self._kuka.moveJointTo(i, jointPoses[i])
          ########################################################################################################
          # STEPPING --
          #
          self._p.stepSimulation()

          if self._renders:
              time.sleep(self._timeStep)

  def mainRobotTraining(self):
      score = 0

      robotBaseJointLimit = self._kuka.getJointLimit(0)

      ## DEBUG ++
      self.drawDebugShape(self._cubeUId, [1,1,1])
      self.drawDebugShape(self._cubeUId, [0,0,0])
      ## DEBUG --

      run_count = 1
      print('RUN # 1 ---------------------------------')
      while True:
          # 1. ROBOT BASE LINK POSITION & ORIENTATION ----------------------------------------
          robotPos, robotOrn = self._kuka.getBasePosOrient()

          # 2. ROBOT BASE JOINT CURRENT POS, MAX POS, MIN POS, VELOCITY ----------------------
          robotBaseJointInfo  = self._kuka.getJointInfo(0)
          robotBaseJointState = self._kuka.getJointState(0)

          # 3. FALLING OBJECTS ---------------------------------------------------------------
          # self._objs
          #
          # 3.1 Objects Pos & Velocity
          # Compose Environment Info (State input)
          envInfo = []
          objsPos = []
          for i in range(len(self._objs)):
              objInfo = []
              #print("Obj Ids: ", i, self._objs[i].id())
              objPos, objOrn = p.getBasePositionAndOrientation(self._objs[i].id())
              objsPos.append(objPos)
              # This returns a list of two vector3 values (3 floats in a list) representing the linear velocity [x,y,z]
              # and angular velocity [wx,wy,wz] in Cartesian worldspace coordinates.
              objLinearVel, objAngularVel = p.getBaseVelocity(self._objs[i].id())

              objInfo+= objPos
              objInfo.append(objLinearVel[2])
              envInfo.append(objInfo)
              #print("OBJINFO:", objInfo)

          #print(envInfo)
          # 4. ROBOT SHORTEST DISTANCE TO OBJECTS ---------------------------------------------
          #objIdList = []
          #for i in range(len(self._objs)):
          #    objIdList.append(self._objs[i].id())
          #
          #robotDistanceToObjs = self._kuka.getShortestDistanceToObjects(objIdList)

          #####################################################################################
          # Get the object of the lowest pos in X
          lowestObjId = 0
          lowestPosZ = objsPos[0][2]
          for i in range(len(objsPos)):
              if(lowestPosZ > objsPos[i][2]):
                  lowestObjId = i
                  lowestPosZ = objsPos[i][2]
          #print('LOWEST OBJ:', objsPos[lowestObjId], 'DIST', robotDistanceToObjs)

          # 5. ACTION -------------------------------------------------------------------------
          #
          if(self._kukaBot.act(envInfo) == 1):
              self._kuka.moveJoint(0, -self._CKUKA_MOVE_INTERVAL) ## Move Left
          elif(self._kukaBot.act(envInfo) == 2):
              self._kuka.moveJoint(0, self._CKUKA_MOVE_INTERVAL)  ## Move Right
          #else if(self._kukaBot.act(envInfo) == 0): NON_MOVE

          # 6. UPDATE OBJECTS ROBOT-HIT & GROUND-HIT STATUS
          for i in range(len(self._objs)):
              #print("CHECK KUKA COLLISION WITH BALL", i)
              if(self._kuka.detectCollisionWith(self._objs[i].id())):
                  self._objs[i].robotHit()
                  self._terminated = 1
                  basePos, baseOrn = self._objs[i].getBasePosAndOrn()
                  print("COLLIDING OBJ:", self._objs[i].id(), "-", basePos)
                  self._sphereUid = self.drawDebugShape(self._sphere, basePos)
                  break

          if self._terminated: # Terminal state
              run_count += 1
              print('RUN #', run_count, '---------------------------------')
              # Update the q_values
              self._kukaBot.update_qvalues()

              ## 0. RELOAD FALLING OBJS --
              self._p.stepSimulation()
              #if self._renders:
              #    time.sleep(5)
              self._reset()
              continue

          # 7. CHECK COLLISION & REWARD
          # Avoid all balls as much as possible
          # LET THE BALLS SIGNAL UPDATE REWARD TO THE ENV UPON EACH ONE TOUCH THE GROUND
          # WITHOUT HITTING THE KUKA
          reward = self.updateReward() ## Actually, this is just score being given if NOT _terminated!

          #######################################################################################
          # 8. STEPPING --
          #
          self._p.stepSimulation()

          if self._renders:
              time.sleep(self._timeStep)

  def moveRobotJoint(self, jointIndex, dv):
      # PREPARE MOTOR COMMANDS --
      #
      motorsIds=[]
      #for i in range (len(self._kuka.motorNames)):
      #    motorName = self._kuka.motorNames[i]
      #    motorJointIndex = self._kuka.motorIndices[i]
      #    motorsIds.append(self._p.addUserDebugParameter(motorName,-3,3,self._kuka.jointPositions[i]))

      motorsIds.append(self._p.addUserDebugParameter(self._kuka.motorNames[jointIndex], -dv, dv, dv))
      print("dv:", dv)
      print("Base Joint- ", self._kuka.getJointState(jointIndex))
      #motorsIds.append(self._p.addUserDebugParameter("posX",-dv,dv,0))
      #motorsIds.append(self._p.addUserDebugParameter("posY",-dv,dv,0))
      #motorsIds.append(self._p.addUserDebugParameter("posZ",-dv,dv,-dv))
      #motorsIds.append(self._p.addUserDebugParameter("yaw",-dv,dv,0))
      #motorsIds.append(self._p.addUserDebugParameter("fingerAngle",0,0.3,.3))

      # PREPARE ACTIONS --
      #
      action=[]
      for motorId in motorsIds:
          action.append(self._p.readUserDebugParameter(motorId))
          #print (action)
      self._kuka.applyAction(action)
#ducta --

  def createRobotAction(self, action):
    dv = 0.01
    dx = [0,-dv,dv,0,0,0,0][action]
    dy = [0,0,0,-dv,dv,0,0][action]
    da = [0,0,0,0,0,-0.1,0.1][action]
    f = 0.3
    realAction = [dx,dy,-0.002,da,f]
    return realAction

  def _step(self, action):
      #realAction = self.createRobotAction(action)
      dv = 0.01
      motorsIds=[]
      motorsIds.append(self._p.addUserDebugParameter(self._kuka.motorNames[0], -dv, dv, dv))
      realAction=[]
      for motorId in motorsIds:
          realAction.append(self._p.readUserDebugParameter(motorId))

      return self.step2(realAction)

  def step2(self, action):
    self._kuka.applyAction(action)
    for i in range(self._actionRepeat):
      p.stepSimulation()
      if self._renders:
        time.sleep(self._timeStep)
      self._observation = self.getExtendedObservation()
      if self._termination():
        break
      self._envStepCounter += 1
    #print("self._envStepCounter")
    #print(self._envStepCounter)

    done = self._termination()
    reward = self._reward()
    #print("len=%r" % len(self._observation))

    return np.array(self._observation), reward, done, {}

  def _render(self, mode='human', close=False):
      return

  def _termination(self):
    #print("self._envStepCounter")
    #print(self._envStepCounter)
    if (self._terminated or self._envStepCounter>1000):
      self._observation = self.getExtendedObservation()
      return True

    # Avoid all balls as much as possible
    # --> Reward is the number of balls touching the ground without hitting the robot!
    for i in range (len(self._objs)):
        #rewards is height of target object
        blockPos,blockOrn=p.getBasePositionAndOrientation(self._objs[i].id())
        closestPoints = p.getClosestPoints(self._objs[i].id(),self._kuka.kukaUid,1000)
        numPt = len(closestPoints)
        #print(numPt)
        if (numPt == 0): # No Collision
          self._observation = self.getExtendedObservation()
          return True
    return False

  def _reward(self):
    #print("reward")
    #print(__reward)
    return self.__reward

  def initializeDebugShapes(self):
    self._sphere = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
    self._cube   = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1,0.1,0.1])

  def drawDebugShape(self, shapeId, pos):
    return p.createMultiBody(1, shapeId, -1, pos, [0,0,0,1])
