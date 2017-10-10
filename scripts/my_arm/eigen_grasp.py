import numpy as np

class EigenGrasp(object):

    def __init__(self, size, eigenval = 0.0, min=0.0, max=0.0):
        self._size = size
        self._eigenval = eigenval
        self._min = min
        self._max = max
        self._amp = 0.0
        self._vals = [0.0] * size

    def setOnes(self):
        vals = [1.0] * self._size
        self.setVals(vals)

    def setRange(self, min, max):
        self._min = min
        self._max = max

    def getAxisVal(self, i):
        return self._vals[i]

    def setAxisVal(self, i, val):
        self._vals[i] = val

    def getVals(self):
        return self._vals

    def setVals(self, vals):
        if (len(vals) != self._size):
            print("ERROR: EigenGrasp(vals), len(vals) != self._size", len(vals), self._size)
            return
        for i in range(len(vals)):
            self._vals[i] = vals[i]

class EigenGraspInterface(object):

    def __init__(self, robot, eigen_grasps, originVals):
        self._robot = robot
        self._dSize = robot.getJointsCount() # dof joint space dimension size

        self._eigen_grasps = eigen_grasps
        self._eSize = len(eigen_grasps) # eigengrasp space dimension size

        self._eg_origin = EigenGrasp(self._dSize)
        self._eg_origin.setVals(originVals)
        self._norm = EigenGrasp(self._dSize)
        self._norm.setOnes()
        self._mP = np.matlib.zeros((self._eSize, self._dSize))
        self._mPInv = np.matlib.zeros((self._dSize, self._eSize))
        # ---------------------------------------------------------------
        # Compute Projection Matrix between dof & eg spaces.
        self.computeProjectionMatrices()

        # Set the min max values for all eigengrasps
        self.setEigenGraspsMinMax()

    def setEigenGraspsMinMax(self):
        # EIGENGRASP_LOOSE
        GB_EG_MIN = +1.0e5
        GB_EG_MAX = -1.0e5
        # mmin = -1.0e5;
        # mmax = +1.0e5;

        dofs = self._robot.getCurrentDofs()
        amps = self.toEigenGrasp(dofs)
        for e in range(self._eSize):
            eg_vals = self._eigen_grasps[e].getVals()
            eg_min, eg_max = GB_EG_MAX, GB_EG_MIN
            for d in range(self._dSize):
                if(eg_vals[d] == 0 or self._eg_origin.getAxisVal(d) == 0):
                    continue
                dof_min, dof_max = self._robot.getDOFRange(d)
                eg_min = (dof_min - dofs[d]) / (eg_vals[d] * self._eg_origin.getAxisVal(d))
                eg_max = (dof_max - dofs[d]) / (eg_vals[d] * self._eg_origin.getAxisVal(d))

                if(eg_min > eg_max):
                    eg_min, eg_max = eg_max, eg_min
                    #x = x + y
                    #y = x - y
                    #x = x - y

                if(eg_min < GB_EG_MIN):
                    GB_EG_MIN = eg_min

                if(eg_max > GB_EG_MAX):
                    GB_EG_MAX = eg_max

            self._eigen_grasps[e].setRange(eg_min, eg_max)

    def checkOrigin(self):
        for d in range(self._dSize):
            dof_min, dof_max = self._robot.getDOFRange(d)
            if(self._eg_origin.getAxisVal(d) < dof_min):
                print("WARNING: Eigengrasp origin lower than DOF range:", d);
                self._eg_origin.setAxisVal(d, dof_min)

            if(self._eg_origin.getAxisVal(d) > dof_max):
                print("WARNING: WARNING: Eigengrasp origin greater than DOF:", d);
                self._eg_origin.setAxisVal(d, dof_max)

    def computeProjectionMatrices(self):
        E = np.matlib.zeros((self._eSize, self._dSize))
        for e in range(self._eSize):
            for d in range(self._dSize):
                E[e,d] = self._eigen_grasps[e].getAxisVal(d)

        # --------------------------------------------------
        ET = E.transpose()
        EET = np.matlib.zeros((self._eSize, self._eSize))
        EET = np.dot(E, ET)
        try:
            EETInv = np.linalg.inv(EET)
        except LinAlgError as err:
            return

        self._mP = np.dot(EETInv, E)
        self._mPInv = ET

    def toDOF(self, amps):
        if(len(amps) != self._eSize):
            print('ERROR: toDOF-Invalid amplitudes!', len(amps), self._eSize)
            return None

        dofs = [0.0] * self._dSize

        a = np.asmatrix(amps).transpose()
        x = np.matlib.zeros((self._dSize,1))
        x = np.dot(self._mPInv, a)
        for d in range(self._dSize):
            dofs[d] = x[d,0] * self._norm.getAxisVal(d) + self._eg_origin.getAxisVal(d)

        return dofs

    def toEigenGrasp(self, dofs):
        if(len(dofs) != self._dSize):
            print('ERROR: toEigenGrasp-Invalid dofs!', len(dofs), self._dSize)
            return

        amps = [0.0] * self._eSize

        x = np.matlib.zeros((self._dSize,1))
        for d in range(self._dSize):
            x[d,0] = (dofs[d] - self._eg_origin.getAxisVal(d)) / self._norm.getAxisVal(d)

        a = np.matlib.zeros((self._eSize,1))
        a = np.dot(self._mP, x)
        for e in range(self._eSize):
            amps[e] = a[e,0]

        return amps
