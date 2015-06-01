#!/usr/bin/python -tt

from bzrc import BZRC, Command
import sys, math, time
from threading import Thread
import matplotlib.pyplot as plt
import numpy as np
from pid_controller import PController, PDController
from filter_grid import FilterGrid

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
        self.TANGENTIAL_FIELD_STRENGTH = 0.4
        self.NINTY_DEGREES_IN_RADIANS = 1.57079633
        self.filter_grid = FilterGrid(800, 800)

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
        self.controllers = None
        self.other_teams = None
        self.kP = 0.5
        self.kD = 0.5

    def tick(self, time_diff):
        '''Some time has passed; decide what to do next'''
        # Get information from the BZRC server
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color != self.constants['team']]

        # Load the other teams once
        if self.other_teams is None:
            self.other_teams = set()
            for enemy in self.enemies:
                self.other_teams.add(enemy.color)
            # To get that indexing, yo
            self.other_teams = list(self.other_teams)
            print "Other teams: ", self.other_teams

        if(time_diff == 0.0):
            return

        # Reset my set of commands (we don't want to run old commands)
        self.commands = []

        # Prime PD controllers on first pass
        if self.controllers is None:
            self.controllers = {}
            for bot in mytanks:
                self.controllers[bot.index] = PDController(self.kP, self.kD)


        # Decide what to do with each of my tanks
        self.threads = []
        for bot in mytanks:
            thread = Thread(target = self.follow_potential_field, args = (bot, time_diff))
            thread.start()
            self.threads.append(thread)


#         while True:
#             time.sleep(10)
#             self.plot_potential_field()

        # Wait for threads to finish
        [thread.join() for thread in self.threads]

#         # Send the commands to the server
#         results = self.bzrc.do_commands(self.commands)


    def follow_potential_field(self, bot, time_diff):

#         self.move_to_position(bot, 0, 0)
#         self.move_to_position(bot, 0, 0)

        if(bot.flag != "-"):
            (theta, changeInX, changeInY) = self.get_potential_field(bot.x, bot.y, self.constants["team"])
        else:
            if bot.index <= 2:
                (theta, changeInX, changeInY) = self.get_potential_field(bot.x, bot.y, self.other_teams[0])
            elif bot.index > 2 and bot.index <= 5:
                (theta, changeInX, changeInY) = self.get_potential_field(bot.x, bot.y, self.other_teams[1])
            else:
                (theta, changeInX, changeInY) = self.get_potential_field(bot.x, bot.y, self.other_teams[2])

        print "bot x, y: " + str(bot.x) + " " + str(bot.y)
        print "real bot x, y: " + str(self.mytanks[0].x) + ", " + str(self.mytanks[0].y)
        print "theta " + str(theta)
        print "change x " + str(changeInX)
        print "change y " + str(changeInY)
        self.move_to_position(bot, bot.x + changeInX, bot.y + changeInY, theta, time_diff)

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

        theta = math.atan2(totalChangeInY, totalChangeInX)
        return theta, totalChangeInX, totalChangeInY


    def get_attractive_field(self, tankX, tankY, obstacleX, obstacleY, obstacleR, obstacleS):
        d = math.sqrt(math.pow((obstacleX - tankX),2) + math.pow((tankY - obstacleY),2))
        theta = math.atan2((obstacleY - tankY), (obstacleX - tankX))
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

    def move_to_position(self, bot, target_x, target_y, theta, dt):
        controller = self.controllers[bot.index]

        goal = theta
        current_angle = bot.angle
        error = self.normalize_angle(goal - current_angle)

        controller.set_target(theta)

        ang_vel = controller.update_with_error_and_dt(error, dt)
        command = Command(bot.index, 1, ang_vel, True)

        print "Target: %f\tCurrent: %f\tPD Ang Vel: %f" % (goal, current_angle, ang_vel)
        self.bzrc.do_commands([command])

    def move_to_position_old(self, bot, target_x, target_y):
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

        numOfPointsOnAxis = 50
        intervalOnWorldRangeToMatchNumOfPoints = (int(self.constants['worldsize'])) / numOfPointsOnAxis
        world_radius = int(self.constants['worldsize']) / 2

        # generate grid
        x = np.linspace(-world_radius, world_radius, numOfPointsOnAxis)
        y = np.linspace(-world_radius, world_radius, numOfPointsOnAxis)
        x, y = np.meshgrid(x, y)
        # calculate vector field
        vx = [np.zeros(numOfPointsOnAxis) for row in range(numOfPointsOnAxis)]
        vy = [np.zeros(numOfPointsOnAxis) for row in range(numOfPointsOnAxis)]

        iterate_to = world_radius - intervalOnWorldRangeToMatchNumOfPoints
        for column in range(-world_radius, iterate_to, intervalOnWorldRangeToMatchNumOfPoints):
            for row in range(-world_radius, iterate_to, intervalOnWorldRangeToMatchNumOfPoints):
                (theta, changeInX, changeInY) = self.get_potential_field(column, row, "red")
#                 (changeInX, changeInY) = self.get_tangential_field(column, row, -370, 0, 100, 600, True)
#                 (changeInX1, changeInY1) = self.get_reject_field(column, row, 0, 0, 50, 300)
#                 (changeInX2, changeInY2) = self.get_attractive_field(column, row, 0, 0, 50, 300)
#                 (changeInX3, changeInY3) = self.get_tangential_field(column, row, 0, 0, 50, 300, True)
#
#                 changeInX = changeInX1 + changeInX2 + changeInX3
#                 changeInY = changeInY1 + changeInY2 + changeInY3

                print changeInX, changeInY
                current_column = ((column + world_radius) / intervalOnWorldRangeToMatchNumOfPoints)
                current_row = ((row + world_radius) / intervalOnWorldRangeToMatchNumOfPoints)

                vx[current_row][current_column] = changeInX
                vy[current_row][current_column] = changeInY

        # plot vector field

        ax.quiver(x, y, vx, vy, pivot='middle', color='r', headwidth=4, headlength=6)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        print "show finished plot"
        plt.show()
        # import sys; sys.exit();
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
