# http://nbviewer.jupyter.org/github/jjehl/poppy_balance/blob/master/balance_leg_math.ipynb

"""

Mathematical method to let poppy vertical

When you want to move the motors of the leg, you can not do whatever you want, because Poppy can fall if it is not balance. So a very simple way to move the leg without any external perturbation (no wind, flat foor, no move of the upper body) is to let the upper body to the veticale of the ankle.

To do that, simple calculation of the angle of ankle, knee and hip can do the job.

Basically, the shin and the thigh form a triangle with a knee angle. So if you determine the angle of the knee, what you want is just to calcul the angle of the ankle and the hip to let Poppy to the verticale position.


To calcul the missing angle, we can use the sinus law and the Al-Kashi theorem. More information here.

https://en.wikipedia.org/wiki/Solution_of_triangles

"""


%pylab inline
from math import *

#Populating the interactive namespace from numpy and matplotlib

class leg_angle:
    def __init__(self,knee=0):
        # different length of poppy in cm
        self.upper_body = 40.0
        self.shin = 18.0
        self.thigh = 18.0
        # the angle of the knee
        self.knee = radians(knee)
        gamma = radians(180 - knee)
        # Al-Kashi theorem to calcul the c side and the missing angle
        c = sqrt(self.shin**2+self.thigh**2-2*self.shin*self.thigh*cos(gamma))
        self.c = c
        self.hip = -acos((self.thigh**2+c**2-self.shin**2)/(2*self.thigh*c))
        self.ankle = -acos((self.shin**2+c**2-self.thigh**2)/(2*self.shin*c))
        # The high of the leg and the foot gap
        self.high = c
        self.foot_gap = 0.0
    def update_knee(self,knee):
        self.knee = radians(knee)
        gamma = radians(180 - knee)
        # Al-Kashi theorem to calcul the c side
        c = sqrt(self.shin**2+self.thigh**2-2*self.shin*self.thigh*cos(gamma))
        self.c = c
        self.hip = -acos((self.thigh**2+c**2-self.shin**2)/(2*self.thigh*c))
        self.ankle = -acos((self.shin**2+c**2-self.thigh**2)/(2*self.shin*c))
        self.high = sqrt(c**2-self.foot_gap**2)
    def update_foot_gap(self,foot_gap):
        if foot_gap >= 0 :
            s = 1
        else :
            s=-1
        self.foot_gap = foot_gap
        # move the foot but let the high constant
        c = sqrt(foot_gap**2+self.high**2)
        self.c = c
        alpha = acos((self.thigh**2+c**2-self.shin**2)/(2*self.thigh*c))
        beta = acos((self.shin**2+c**2-self.thigh**2)/(2*self.shin*c))
        gamma = acos((self.shin**2+self.thigh**2-self.c**2)/(2*self.shin*self.thigh))
        self.knee = pi - gamma
        self.hip = -(alpha + s*acos(self.high/c))
        self.ankle = -(beta - s*acos(self.high/c))
    def update_high(self,high):
        if self.foot_gap >= 0 :
            s = 1
        else :
            s=-1
        self.high = high
        c = sqrt(self.foot_gap**2+self.high**2)
        self.c = c
        alpha = acos((self.thigh**2+c**2-self.shin**2)/(2*self.thigh*c))
        beta = acos((self.shin**2+c**2-self.thigh**2)/(2*self.shin*c))
        gamma = acos((self.shin**2+self.thigh**2-self.c**2)/(2*self.shin*self.thigh))
        self.knee = pi - gamma
        self.hip = -(alpha + s*acos(self.high/c))
        self.ankle = -(beta - s*acos(self.high/c))
    def gravity_center_front(self,d_thigh):
        c = sqrt(self.foot_gap**2+self.high**2)
        self.c = c
        alpha = acos(((self.thigh+d_thigh)**2+c**2-self.shin**2)/(2*(self.thigh+d_thigh)*c))
        beta = acos((self.shin**2+c**2-(self.thigh+d_thigh)**2)/(2*self.shin*c))
        gamma = acos((self.shin**2+(self.thigh+d_thigh)**2-self.c**2)/(2*self.shin*(self.thigh+d_thigh)))
        self.knee = pi - gamma
        self.hip = -(alpha + acos(self.high/c))
        self.ankle = -(beta - acos(self.high/c))
        gamma = pi+self.hip
        self.hip = -(pi-gamma-asin(((d_thigh*sin(gamma)))/self.upper_body))
  

  # Now,You need the robot and the V-REP time.

from poppy.creatures import PoppyHumanoid

poppy = PoppyHumanoid(simulator='vrep')
import time as real_time
class time:
    def __init__(self,robot):
        self.robot=robot
    def time(self):
        t_simu = self.robot.current_simulation_time
        return t_simu
    def sleep(self,t):
        t0 = self.robot.current_simulation_time
        while (self.robot.current_simulation_time - t0) < t-0.01:
            real_time.sleep(0.001)

time = time(poppy)
print time.time()
time.sleep(0.025) #0.025 is the minimum step according to the V-REP defined dt  
print time.time()

# It is now possible to define a mobility in percentage, according to the angle limit of ankle.

class leg_move(leg_angle):
    def __init__(self,motor_limit,knee=0):
        self.ankle_limit_front=radians(motor_limit.angle_limit[1])
        self.ankle_limit_back=radians(motor_limit.angle_limit[0])
        leg_angle.__init__(self,knee)
        
    def update_foot_gap_percent(self,foot_gap_percent):
        #calcul of foot_gap_max to convert foot_gap_percent into value
        if foot_gap_percent>=0:# si le foot_gap est positif
            if  acos(self.high/(self.shin+self.thigh)) > self.ankle_limit_front:
                # construction 1 knee!=0
                gap1 = sin(self.ankle_limit_front)*self.shin
                high1 = cos(self.ankle_limit_front)*self.shin
                high2 = self.high - high1
                gap2 = sqrt(self.thigh**2-high2**2)
                foot_gap_max = gap1 + gap2
                foot_gap = foot_gap_percent * foot_gap_max / 100
                self.update_foot_gap(foot_gap)
            else:
                #construction 2 knee=0
                foot_gap_max = sqrt((self.shin+self.thigh)**2-self.high**2)
                foot_gap = foot_gap_percent * foot_gap_max / 100
                self.update_foot_gap(foot_gap)
        if foot_gap_percent<0:
            if -acos((self.high-self.thigh)/self.shin )< self.ankle_limit_back:
                #construction 1 knee!=0
                print degrees(self.ankle_limit_back)
                print degrees(-acos((self.high-self.thigh)/self.shin ))
                gap1 = sin(self.ankle_limit_back)*self.shin
                high1 = cos(self.ankle_limit_back)*self.shin
                high2 = self.high - high1
                print gap1,high1,high2
                gap2 = sqrt(self.thigh**2-high2**2)
                print gap1,gap2,high1,high2
                foot_gap_max = gap1 + gap2
                foot_gap = -foot_gap_percent * foot_gap_max / 100
                self.update_foot_gap(foot_gap)
            else:
                #constrution 2 knee=0
                foot_gap_max = sqrt((self.shin+self.thigh)**2-self.high**2)
                foot_gap = foot_gap_percent * foot_gap_max / 100
                self.update_foot_gap(foot_gap)
                
    def update_high_percent(self,high_percent,high_min,high_max):
        high_var = high_max-high_min
        high = (high_percent*high_var/100)+high_min
        self.update_high(high)
        
    def high_limit(self):
        high_max = sqrt((self.shin+self.thigh)**2-self.foot_gap**2)
        high1_min = cos(self.ankle_limit_back)*self.shin
        gap2 = self.foot_gap-sin(self.ankle_limit_back)*self.shin
        # si gap2 est supérieur a thigh alors ce n'est plus la flexion de la cheville qui est limitante
        # dans ce cas on met la hauteur a zero
        if gap2 <= self.thigh:
            high2_min = sqrt(self.thigh**2-gap2**2)
            high_min = high1_min + high2_min
        else:
            high_min = 0
        return [high_min,high_max]

# Finaly, a primitive can set the high and the foot gap of poppy.

from pypot.primitive import Primitive

class leg_primitive(Primitive):
    def __init__(self,robot,speed,knee=0):
        self.right = leg_move(robot.l_ankle_y,knee)# il faudrait mettre r_ankle_y mais les angles limites semblent faux, c'est l'opposé
        self.left = leg_move(robot.l_ankle_y,knee)
        self.robot = robot
        Primitive.__init__(self, robot)
        self.high_percent = 100
        self.r_foot_gap_percent = 0
        self.l_foot_gap_percent = 0
        self.speed = speed
                
    def run(self):    
        if self.high_percent !=-1:
            high_limit=(max([self.right.high_limit()[0],self.left.high_limit()[0]]),min([self.right.high_limit()[1],self.left.high_limit()[1]]))
            self.right.update_high_percent(self.high_percent,high_limit[0],high_limit[1])
            self.left.update_high_percent(self.high_percent,high_limit[0],high_limit[1])
        
        if self.r_foot_gap_percent !=-1:  
            self.right.update_foot_gap_percent(self.r_foot_gap_percent)
        
        if self.l_foot_gap_percent !=-1: 
            self.left.update_foot_gap_percent(self.l_foot_gap_percent)
        
        print "left - ankle" ,degrees(self.left.ankle),'knee', degrees(self.left.knee),'hip', degrees(self.left.hip), 'high', self.left.high,'foot_gap',self.left.foot_gap
        print "right - ankle" ,degrees(self.right.ankle),'knee', degrees(self.right.knee),'hip', degrees(self.right.hip), 'high', self.right.high,'foot_gap',self.right.foot_gap
        
        
        self.robot.l_ankle_y.goto_position(degrees(self.left.ankle),self.speed)
        self.robot.r_ankle_y.goto_position(degrees(self.right.ankle),self.speed)

        self.robot.l_knee_y.goto_position(degrees(self.left.knee),self.speed)
        self.robot.r_knee_y.goto_position(degrees(self.right.knee),self.speed)

        self.robot.l_hip_y.goto_position(degrees(self.left.hip),self.speed)
        self.robot.r_hip_y.goto_position(degrees(self.right.hip),self.speed,wait=True)
        


# It is now possible to set the high and the foot gap using the leg_primitive.
