class VirtualRobotVrep(Robot):

    def __init__(self):
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot)
        vrep.simxSynchronous(self.clientID,True)

    def finish_iteration(self):
        vrep.simxSynchronousTrigger(self.clientID)

    def get_joints(self):
    	if self.clientID != -1:
    	    errorCode, handles, intData, floatData, array = vrep.simxGetObjectGroupData(self.clientID,
    	                                                                                vrep.sim_appobj_object_type,
    	                                                                                0,
    	                                                                                vrep.simx_opmode_oneshot_wait)
	
	    	    data = dict(zip(array, handles))
	
	    	    fl_leg = dict((key, value) for key, value in data.iteritems() if "fl" in key and "joint" in key)
	    	    fr_leg = dict((key, value) for key, value in data.iteritems() if "fr" in key and "joint" in key)
	    	    rr_leg = dict((key, value) for key, value in data.iteritems() if "rr" in key and "joint" in key)
	    	    rl_leg = dict((key, value) for key, value in data.iteritems() if "rl" in key and "joint" in key)
	
	    	    return fl_leg, fr_leg, rr_leg, rl_leg
    	return None


class VirtualLegVrep(Leg):

    def __init__(self, name, handles, clientID, position, resting_positions):
        Leg.__init__(self, name, position, resting_positions)
        self.handles = handles
        self.clientID = clientID

        for key in self.handles:
                if "shoulder" in key:
                    self.shoulderHandle = self.handles[key]
                elif "femur" in key:
                    self.femurHandle = self.handles[key]
                elif "tibia" in key:
                    self.tibiaHandle = self.handles[key]

    def move_to_angle(self, shoulderAngle, femurAngle, tibiaAngle):
        vrep.simxSetJointTargetPosition(self.clientID,
                                        self.shoulderHandle,
                                        shoulderAngle,
                                        vrep.simx_opmode_oneshot)

        vrep.simxSetJointTargetPosition(self.clientID,
                                        self.femurHandle,
                                        femurAngle*self.ydirection,
                                        vrep.simx_opmode_oneshot)

        vrep.simxSetJointTargetPosition(self.clientID,
                                        self.tibiaHandle,
                                        tibiaAngle*self.ydirection,
                                        vrep.simx_opmode_oneshot)