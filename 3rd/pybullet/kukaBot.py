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

        self.gameCNT = 0 # Game count of current run, incremented after every death
        self.DUMPING_N = 3 # Number of iterations to dump Q values to JSON after
        self.discount = 1.0
        self.r = {0: 1, 1:-1, 2: -1000} # Reward function
        self.lr = 0.7
        self.load_qvalues()
        self.latest_state  = "0_0_0_-6_0_0_0_-6_0_0_0_-6_0_0_0_-6_0_0_0_-6_"
        self.latest_action = self._actions["stand_still"]
        self.move_history = [] ## The history of moves for each session (start -> terminated)

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
            #print('New State:', self.latest_action)

        # Observed state: Make the movement that has the largest qvalue!
        else:
            max_act_qvalue = max(self.qvalues[current_state])
            print('Current State max QValue:', max_act_qvalue)
            if (max_act_qvalue == self.qvalues[current_state][0]):
                print(current_state, 'Stand still')
                self.latest_action = self._actions["stand_still"]
            elif (max_act_qvalue == self.qvalues[current_state][1]):
                print(current_state, 'Move Left')
                self.latest_action = self._actions["left"]
            elif (max_act_qvalue == self.qvalues[current_state][2]):
                print(current_state, 'Move Right')
                self.latest_action = self._actions["right"]

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
                self.qvalues[latest_state] = [0,0,0]

            if not current_state in self.qvalues:
                self.qvalues[current_state] = [0,0,0]

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
        for objInfo in envInfo:
            xPos = (objInfo[0]*100) % 5
            yPos = (objInfo[1]*100) % 5
            zPos = (objInfo[2]*100) % 5
            zVel = objInfo[3]
            state_str += str(int(xPos)) + '_' + str(int(yPos)) + '_' + str(int(zPos))+'_'+str(int(zVel))+'_'

        #print (state_str)
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
