import json


class KukaBot(object):
    '''
    The Bot class that applies the Qlearning logic to Kuka manipulator
    After every iteration (iteration = 1 game that ends with Kuka colliding with an object) updates Q values
    After every DUMPING_N iterations, dumps the Q values to the local JSON file
    '''
    def __init__(self):
        self._actions = {
            "left" : 0,
            "right": 1
        }

        self.gameCNT = 0 # Game count of current run, incremented after every death
        self.DUMPING_N = 25 # Number of iterations to dump Q values to JSON after
        self.discount = 1.0
        self.r = {0: 1, 1: -1000} # Reward function
        self.lr = 0.7
        self.load_qvalues()
        self.latest_state  = "-3_-29_24_0"
        self.latest_action = self._actions["left"]
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
    def act(self, xPos, yPos, zdif, vel):
        '''
        Chooses the best action with respect to the current state - Chooses 0 (don't flap) to tie-break
        '''
        state = self.map_state(xPos, yPos, zdif, vel)

        self.move_history.append( [self.latest_state, self.latest_action, state] ) # Add the experience to the history

        self.latest_state = state # Update the latest_state with the current state

        if self.qvalues[state][0] > self.qvalues[state][1]:
            self.latest_action = self._actions["left"]
        else:
            self.latest_action = self._actions["right"]

        return self.latest_action

    def get_latest_state(self):
        return self.latest_state

    # UPDATE Q_VALUES
    def update_qvalues(self):
        '''
        Update qvalues via iterating over experiences
        '''
        ## Moves: [latest_state, latest_action, "zdif_vel"]
        history = list(reversed(self.move_history))

        #Q-learning score updates
        t = 1
        for exp in history:
            state = exp[0]
            act = exp[1]
            res_state = exp[2]
            if t==1 or t==2:
                self.qvalues[state][act] = (1- self.lr) * (self.qvalues[state][act]) + (self.lr) * ( self.r[1] + (self.discount)*max(self.qvalues[res_state]) )

            else:
                self.qvalues[state][act] = (1- self.lr) * (self.qvalues[state][act]) + (self.lr) * ( self.r[0] + (self.discount)*max(self.qvalues[res_state]) )
            t += 1

        self.gameCNT += 1 #increase game count
        self.dump_qvalues() # Dump q values (if game count % DUMPING_N == 0)
        self.move_history = []  #clear history after updating strategies

    def map_state(self, xPos, yPos, zdif, vel):
        '''
        Map the (xdif, ydif, vel) to the respective state, with regards to the grids
        The state is a string, "xdif_ydif_vel"

        X -> [-40,-30...120] U [140, 210 ... 420]
        Y -> [-300, -290 ... 160] U [180, 240 ... 420]
        '''
        if zdif < 0.1:
            zdif = int(zdif) - (int(zdif) % 1)
        else:
            zdif = int(zdif) - (int(zdif) % 6)

        return str(int(xPos)) + '_' + str(int(yPos)) + '_' + str(int(zdif))+'_'+str(vel)

    def dump_qvalues(self):
        '''
        Dump the qvalues to the JSON file
        '''
        if self.gameCNT % self.DUMPING_N == 0:
            fil = open('qvalues.json', 'w')
            json.dump(self.qvalues, fil)
            fil.close()
            print('Q-values updated on local file.')
