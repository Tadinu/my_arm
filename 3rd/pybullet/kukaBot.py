import json
import random

class KukaBot(object):
    '''
    The Bot class that applies the Qlearning logic to Kuka manipulator
    After every iteration (iteration = 1 game that ends with Kuka colliding with an object) updates Q values
    After every DUMPING_N iterations, dumps the Q values to the local JSON file
    '''
    def __init__(self):
        self._actions = {
            "stand_still": 0,
            "left"       : 1,
            "right"      : 2
        }

        self._CINITIAL_ACT_IDS = [0] * 3

        self.gameCNT = 0 # Game count of current run, incremented after every death
        self.DUMPING_N = 3 # Number of iterations to dump Q values to JSON after
        self.discount = 1.0
        self.r = {0: 1, 1:-1, 2: -1000} # Reward function
        self.lr = 0.7
        self.load_qvalues()
        self.latest_state  = "0_0_0_-6_0_0_0_-6_0_0_0_-6_0_0_0_-6_0_0_0_-6_"
        self.latest_action = self._actions["stand_still"]
        self.move_history = [] ## The history of moves for each session (start -> terminated)

        self._DELTA_X = 0.1
        self._DELTA_Y = 0.1
        self._DELTA_Z = 1

        self._MAX_X   = 1
        self._MAX_Y   = 1
        self._MAX_Z   = 10

        self._MIN_X   = -1
        self._MIN_Y   = -1
        self._MIN_Z   = -0.1

        self._CELL_NO_X = (self._MAX_X - self._MIN_X)/self._DELTA_X
        self._CELL_NO_Y = (self._MAX_Y - self._MIN_Y)/self._DELTA_Y
        self._CELL_NO_Z = (self._MAX_Z - self._MIN_Z)/self._DELTA_Z

    def load_qvalues(self):
        '''
        Load q values from a JSON file
        '''
        self.qvalues = {}
        try:
            fil = open('qvalues.json', 'r')
        except IOError:
            return
        self.qvalues = json.load(fil)
        fil.close()

    ## 1. Query the database for the state that correspond to the input env elements(xPos, yPos, vel)
    ## 2. Check the current Q-values of that state -> Pick out the action that has largest value!
    ## => Set it to the latest action!
    def act(self, envInfo):
        '''
        Chooses the best action with respect to the current state - Chooses 0 (don't flap) to tie-break
        '''
        current_state = self.map_state(envInfo)

        self.move_history.append( [self.latest_state, self.latest_action, current_state] ) # Add the experience to the history

        self.latest_state = current_state # Update the latest_state with the current state

        # New observed state: Make a random move
        if (not current_state in self.qvalues):
            self.latest_action = random.choice([0,1,2])
            print('New State:', self.latest_action) ### <------------ !!!!!!!!!!!!!!!

        # Observed state: Make the movement that has the largest qvalue!
        else:
            max_act_qvalue = max(self.qvalues[current_state])
            for key in self._actions.keys():
                actionId = self._actions[key] # set the same as the index
                if (self.qvalues[current_state][actionId] == max_act_qvalue):
                    self.latest_action = actionId
                    break

        #print ('Current State:', current_state, 'Action:', self.latest_action)
        return self.latest_action

    def get_latest_state(self):
        return self.latest_state

    # UPDATE Q_VALUES
    def update_qvalues(self):
        '''
        Update qvalues via iterating over experiences
        '''
        ## Moves: [latest_state, latest_action, "zdif_vel"]
        move_history = list(reversed(self.move_history))

        #Q-learning score updates
        t = 1
        for exp in move_history:
            latest_state = exp[0]
            act = exp[1]
            current_state = exp[2]
            if not latest_state in self.qvalues:
                self.qvalues[latest_state] = self._CINITIAL_ACT_IDS

            if not current_state in self.qvalues:
                self.qvalues[current_state] = self._CINITIAL_ACT_IDS

            if t==1 or t==2:
                reward = self.r[2]

            elif act == 0: # Non-move
                reward = self.r[2]

            else:
                reward = self.r[1]

            self.qvalues[latest_state][act] = (1- self.lr) * (self.qvalues[latest_state][act]) + (self.lr) * ( reward + (self.discount)*max(self.qvalues[current_state]) )
            t += 1

        self.gameCNT += 1 #increase game count
        self.dump_qvalues() # Dump q values (if game count % DUMPING_N == 0)
        self.move_history = []  #clear history after updating strategies

    ## Input: The pos of objects
    ## Output: The corresponding env state {Unit coor of objs & vel in air}
    def map_state(self, envInfo):
        '''
        Map the (xdif, ydif, vel) to the respective state, with regards to the grids
        The state is a string, "xdif_ydif_vel"

        X -> [-40,-30...120] U [140, 210 ... 420]
        Y -> [-300, -290 ... 160] U [180, 240 ... 420]
        '''
        state_str = ''
        ## Base joint current pos
        baseJointPos = envInfo[0]
        print
        for i in range(len(envInfo)):
            if i!=0:
                objInfo = envInfo[i]

                ## --
                #print("objInfo:", objInfo[0],"-", objInfo[1],"-", objInfo[2])
                xPos = int(abs(objInfo[0]-self._MIN_X)/self._DELTA_X)
                yPos = int(abs(objInfo[1]-self._MIN_Y)/self._DELTA_Y)
                zPos = int(abs(objInfo[2]-self._MIN_Z)/self._DELTA_Z)

                ##--
                objIndex = self._CELL_NO_X * self._CELL_NO_Y * (zPos-1) + \
                           xPos * yPos - 1
                print("XYZ:", xPos,"-", yPos,"-", zPos, "- ObjIndex:", int(objIndex))
                state_str += str(int(objIndex)) + '_'

        state_str += str(int(baseJointPos*10))

        print (state_str)
        return state_str

    def dump_qvalues(self):
        '''
        Dump the qvalues to the JSON file
        '''
        if self.gameCNT % self.DUMPING_N == 0:
            fil = open('qvalues.json', 'w')
            json.dump(self.qvalues, fil)
            fil.close()
            print('Q-values updated on local file.')
