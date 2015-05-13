#!/usr/bin/python -tt

from bzrc import BZRC, Command
import sys, math, time
from threading import Thread
import matplotlib.pyplot as plt
import numpy as np


"""
    Agent Based on Potential fields
"""

#################################################################
# After starting the bzrflag server, this is one way to start
# this code:
# python agent0.py [hostname] [port]
#################################################################

class AgentPotential(object):

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.OBSTACLES = self.bzrc.get_obstacles()
        self.ATTRACTIVE_FIELD_STRENGTH = 2.0
        self.REJECT_FIELD_STRENGTH = 0.8
        self.TANGENTIAL_FIELD_STRENGTH = 0.3
        self.NINTY_DEGREES_IN_RADIANS = 1.57079633
        
        #  radius constants
        self.FLAG_R = float(self.constants['flagradius'])
        self.OBSTACLE_R = 10.0
        self.SHOT_R = float(self.constants['shotradius'])
        self.TANK_R = float(self.constants['tankradius'])
        
        #  spread constants
        #  self.FLAG_S = float(self.constants['worldsize'])
        self.FLAG_S = 100
        self.OBSTACLE_S = 200.0
        self.SHOT_S = 30.0
        self.TANK_S = 150.0
                
        self.commands = []
        self.threads = []

    def tick(self, time_diff):
        '''Some time has passed; decide what to do next'''
        # Get information from the BZRC server
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color != self.constants['team']]

        if(time_diff == 0.0):
            return

        # Reset my set of commands (we don't want to run old commands)
        self.commands = []

        # Decide what to do with each of my tanks
        self.threads = []
        for bot in mytanks:
            thread = Thread(target = self.follow_potential_field, args = (bot,))
            thread.start()
            self.threads.append(thread)
            break
        
#         while True:
#             time.sleep(10)
#             self.plot_potential_field()
        
        
        while True:
            mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
            self.mytanks = mytanks
            self.othertanks = othertanks
            self.flags = flags
            self.shots = shots
            self.enemies = [tank for tank in othertanks if tank.color != self.constants['team']]

        
        
        # Wait for threads to finish
        [thread.join() for thread in self.threads]

#         # Send the commands to the server
#         results = self.bzrc.do_commands(self.commands)



    def follow_potential_field(self, bot):
        
#         self.move_to_position(bot, 0, 0)
        self.move_to_position(bot, 0, 0)
         
        flagCaptured = False
        while not flagCaptured:
            theta = changeInX = changeInY = 0.0
            if(flagCaptured):
                (theta, changeInX, changeInY) = self.get_potential_field(bot.x, bot.y, self.constants["team"])
            else:
                (theta, changeInX, changeInY) = self.get_potential_field(bot.x, bot.y, "red")
            
            print "bot x, y: " + str(bot.x) + " " + str(bot.y)   
            print "real bot x, y: " + str(self.mytanks[0].x) + ", " + str(self.mytanks[0].y)      
            print "theta " + str(theta)
            print "change x " + str(changeInX)
            print "change y " + str(changeInY)
#             self.move_to_position(bot, bot.x + changeInX, bot.y + changeInY)
             
#             self.move_to_angle(bot, theta)
            time.sleep(2)  
                
            if(changeInX == 0.0 and changeInY == 0.0):
                if(flagCaptured == False):
                    flagCaptured = True
                else:
                    flagCaptured = False
     
    def get_potential_field(self, tankX, tankY, flagCol):
        totalChangeInX = 0.0
        totalChangeInY = 0.0
        
        #  add up all potential fields for flags, obstacles, shots, and other tanks
        #  will probably change this a lot as we test the various fields with the visualiztion through matplotlib
        
        for flag in self.flags:
            if(flag.color == flagCol):
                print "flag x, y:  " + str(flag.x) + ", " + str(flag.y) 
                (changeInX, changeInY) = self.get_attractive_field(tankX, tankY, flag.x, flag.y, self.FLAG_R, self.FLAG_S)
    #                 (changeInX, changeInY) = self.get_tangential_field(i, j, -370, 0, 100, 600, True)
                totalChangeInX += changeInX
                totalChangeInY += changeInY
 
#  
# #         (changeInX, changeInY) = self.get_reject_field(tankX, tankY, 0, 0, 20, 600)
# #         totalChangeInX += changeInX
# #         totalChangeInY += changeInY
# # 
# #         (changeInX, changeInY) = self.get_tangential_field(tankX, tankY, 0, 0, 20, 600, True)
# #         totalChangeInX += changeInX
# #         totalChangeInY += changeInY
#  
#         for shot in self.shots:
#             (changeInX, changeInY) = self.get_reject_field(tankX, tankY, shot.x, shot.y, self.SHOT_R, self.SHOT_S)
#             totalChangeInX += changeInX
#             totalChangeInY += changeInY
#   
#         #  TODO:  need to calculate the x and y of the center of each obstacle, then these methods should work just fine
#         #  we might want to think about how to best define fields for obstacles
#         #  he used both tangential and reject combined so I figured it was a good start
#         for obstacle in self.OBSTACLES:
#             #  print obstacle
#             obstacleX = (obstacle[1][0] + obstacle[2][0]) / 2.0
#             obstacleY = (obstacle[0][1] + obstacle[1][1]) / 2.0
#             (changeInX, changeInY) = self.get_reject_field(tankX, tankY, obstacleX, obstacleY, self.OBSTACLE_R, self.OBSTACLE_S)
#             totalChangeInX += changeInX
#             totalChangeInY += changeInY
#             (changeInX, changeInY) = self.get_tangential_field(tankX, tankY, obstacleX, obstacleY, self.OBSTACLE_R, self.OBSTACLE_S, False)
#             totalChangeInX += changeInX
#             totalChangeInY += changeInY
#              
#              
#             
#         #  right now our field ignores team tanks and enemy tanks, we can evolve our strategy after we check out the potential field graphs
        
        theta = math.atan2(totalChangeInY, totalChangeInX)
        return theta, totalChangeInX, totalChangeInY
        

    def get_attractive_field(self, tankX, tankY, obstacleY, obstacleX, obstacleR, obstacleS):
        d = math.sqrt(math.pow((obstacleX - tankX),2) + math.pow((tankY - obstacleY),2))
        theta = math.atan2((obstacleX - tankX), (obstacleY - tankY))
        #  print theta
        changeInX = 0.0
        changeInY = 0.0
        
        if(d < obstacleR):
            pass
        elif(d <= obstacleR + obstacleS):
            changeInX = self.ATTRACTIVE_FIELD_STRENGTH * (d - obstacleR) * math.cos(theta)
            changeInY = self.ATTRACTIVE_FIELD_STRENGTH * (d - obstacleR) * math.sin(theta)
        else:
            changeInX = self.ATTRACTIVE_FIELD_STRENGTH * obstacleS * math.cos(theta)
            changeInY = self.ATTRACTIVE_FIELD_STRENGTH * obstacleS * math.sin(theta)
       
        return changeInX, changeInY


    def get_reject_field(self, tankX, tankY, obstacleY, obstacleX, obstacleR, obstacleS):
        d = math.sqrt(math.pow((obstacleX - tankX),2) + math.pow((tankY - obstacleY),2))
        theta = math.atan2((obstacleX - tankX), (obstacleY - tankY))
        changeInX = 0.0
        changeInY = 0.0
        
        
        if(d < obstacleR):
            #  changeInX = -1.0 * math.copysign(1.0, math.cos(theta)) * float("inf")
            #  changeInY = -1.0 * math.copysign(1.0, math.sin(theta)) * float("inf")
            #  not sure how to deal with infinity down the line so this just became 1000, which hopefully will trigger
            #  the maximum angular and linear velocities
            changeInX = -1.0 * math.copysign(1.0, math.cos(theta)) * float("inf")
            changeInY = -1.0 * math.copysign(1.0, math.sin(theta)) * float("inf")
        elif(d <= obstacleR + obstacleS):
            changeInX = -1.0 * self.REJECT_FIELD_STRENGTH * (obstacleS + obstacleR - d) * math.cos(theta)
            changeInY = -1.0 * self.REJECT_FIELD_STRENGTH * (obstacleS + obstacleR - d) * math.sin(theta)
        else:
            pass
        
        return changeInX, changeInY


    """
        Returns the change in X and Y for a tagential field, the spin is true clockwise, false counter-clockwise I believe
        It might be the reverse though 
    """
    def get_tangential_field(self, tankX, tankY, obstacleY, obstacleX, obstacleR, obstacleS, spin):
        d = math.sqrt(math.pow((obstacleX - tankX),2) + math.pow((tankY - obstacleY),2))
        theta = math.atan2((obstacleX - tankX), (obstacleY - tankY))
        if(spin):
            theta += self.NINTY_DEGREES_IN_RADIANS
        else:
            theta -= self.NINTY_DEGREES_IN_RADIANS
        changeInX = 0.0
        changeInY = 0.0
        
        if(d < obstacleR):
            changeInX = -1.0 * math.copysign(1.0, math.cos(theta)) * float("inf")
            changeInY = -1.0 * math.copysign(1.0, math.sin(theta)) * float("inf")
        elif(d <= obstacleR + obstacleS):
            changeInX = -1.0 * self.TANGENTIAL_FIELD_STRENGTH * (obstacleS + obstacleR - d) * math.cos(theta)
            changeInY = -1.0 * self.TANGENTIAL_FIELD_STRENGTH * (obstacleS + obstacleR - d) * math.sin(theta)
        else:
            pass
        
        return changeInX, changeInY


    def move_to_angle(self, bot, theta):
        print bot.angle
        relative_angle = self.normalize_angle(theta - bot.angle)
        command = Command(bot.index, 1, 2 * relative_angle, True)
        #  self.commands.append(command)
        # Send the commands to the server
        
        self.bzrc.do_commands([command])


    def move_to_position(self, bot, target_x, target_y):
        target_angle = math.atan2(target_y - bot.y, target_x - bot.x)
        relative_angle = self.normalize_angle(target_angle - bot.angle)
        command = Command(bot.index, 1, 2 * relative_angle, True)
        #  self.commands.append(command)
        # Send the commands to the server
        
        self.bzrc.do_commands([command])

    def normalize_angle(self, angle):
        '''Make any angle be between +/- pi.'''
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def plot_potential_field(self):
        #!/usr/bin/env python   
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        numOfPointsOnAxis = 30
        intervalOnWorldRangeToMatchNumOfPoints = (int(self.constants['worldsize'])) / numOfPointsOnAxis
        
        # generate grid
        x=np.linspace(-int(self.constants['worldsize']) / 2, int(self.constants['worldsize']) / 2, numOfPointsOnAxis)
        y=np.linspace(-int(self.constants['worldsize']) / 2, int(self.constants['worldsize']) / 2, numOfPointsOnAxis)
        x, y=np.meshgrid(x, y)
        # calculate vector field
#         vx=-y/np.sqrt(x**2+y**2)*np.exp(-(x**2+y**2))
#         vy= x/np.sqrt(x**2+y**2)*np.exp(-(x**2+y**2))
        vx = np.ndarray(shape=(numOfPointsOnAxis, numOfPointsOnAxis))
        vy = np.ndarray(shape=(numOfPointsOnAxis, numOfPointsOnAxis))
        
        for i in range(-int(self.constants['worldsize']) / 2, int(self.constants['worldsize']) / 2, intervalOnWorldRangeToMatchNumOfPoints):
            for j in range(-int(self.constants['worldsize']) / 2, int(self.constants['worldsize']) / 2, intervalOnWorldRangeToMatchNumOfPoints):
                (theta, changeInX, changeInY) = self.get_potential_field(i, j, "red")
#                 (changeInX, changeInY) = self.get_tangential_field(i, j, -370, 0, 100, 600, True)
                print changeInX, changeInY
                vx[(i + int(self.constants['worldsize']) / 2) / intervalOnWorldRangeToMatchNumOfPoints - 1][(j + int(self.constants['worldsize']) / 2) / intervalOnWorldRangeToMatchNumOfPoints - 1] = changeInX
                vy[(i + int(self.constants['worldsize']) / 2) / intervalOnWorldRangeToMatchNumOfPoints - 1][(j + int(self.constants['worldsize']) / 2) / intervalOnWorldRangeToMatchNumOfPoints - 1] = changeInY
                
        # plot vecor field
        ax.quiver(x, y, vx, vy, pivot='middle', color='r', headwidth=4, headlength=6)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        print "show finished plot"
        plt.show()
        #plt.savefig('visualization_quiver_demo.png')



def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = AgentPotential(bzrc)
    prev_time = time.time()

    #  plot the potential field
    agent.tick(0.0)
    agent.plot_potential_field()
    

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()
