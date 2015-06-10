#!/usr/bin/python -tt

from bzrc import BZRC, Command
import sys, math, time
from threading import Thread
from kalman_graph import KalmanGraph
from pid_controller import PController, PDController
import random
import numpy as np

# An incredibly simple agent.  All we do is find the closest enemy tank, drive
# towards it, and shoot.  Note that if friendly fire is allowed, you will very
# often kill your own tanks with this code.

#################################################################
# NOTE TO STUDENTS
# This is a starting point for you.  You will need to greatly
# modify this code if you want to do anything useful.  But this
# should help you to know how to interact with BZRC in order to
# get the information you need.
#
# After starting the bzrflag server, this is one way to start
# this code:
# python agent0.py [hostname] [port]
#
# Often this translates to something like the following (with the
# port name being printed out by the bzrflag server):
# python agent0.py localhost 49857
#################################################################

class AgentKalmanFilter(object):

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.threads = []
        self.graph = KalmanGraph()
        self.do_this_once = True
        self.kP = 0.5
        self.kD = 0.5

        self.controller = PDController(0.5, 0.5)

        #  Kalman filter variables
        self.stepTimeInSeconds = 0.1
        self.frictionCoefficient = 0.0

        self.sigma_t = np.matrix([[100.0,0,0,0,0,0],[0,0.1,0,0,0,0],[0,0,0.1,0,0,0],[0,0,0,100.0,0,0],[0,0,0,0,0.1,0],[0,0,0,0,0,0.1]])
        self.u_t = np.matrix([[0],[0],[0],[0],[0],[0]])

        self.f = np.matrix([[1.0,self.stepTimeInSeconds,math.pow(self.stepTimeInSeconds, 2.0)/2.0,0,0,0],[0,1.0,self.stepTimeInSeconds,0,0,0],[0,-self.frictionCoefficient,1.0,0,0,0],[0,0,0,1.0,self.stepTimeInSeconds,math.pow(self.stepTimeInSeconds, 2.0)/2.0],[0,0,0,0,1.0,self.stepTimeInSeconds],[0,0,0,0,-self.frictionCoefficient,1.0]])
        self.f_trans = np.transpose(self.f)
        self.h = np.matrix([[1.0,0,0,0,0,0],[0,0,0,1.0,0,0]])
        self.h_trans = np.transpose(self.h)
        self.sigma_x = np.matrix([[0.1,0,0,0,0,0],[0,0.1,0,0,0,0],[0,0,100.0,0,0,0],[0,0,0,0.1,0,0],[0,0,0,0,0.1,0],[0,0,0,0,0,100.0]])
        self.sigma_z = np.matrix([[25.0,0],[0,25.0]])

        self.i = np.matrix([[1.0,0,0,0,0,0],[0,1.0,0,0,0,0],[0,0,1.0,0,0,0],[0,0,0,1.0,0,0],[0,0,0,0,1.0,0],[0,0,0,0,0,1.0]])


    def tick(self, time_diff):
        '''Some time has passed; decide what to do next'''
        # Get information from the BZRC server
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color != self.constants['team']]

        # Reset my set of commands (we don't want to run old commands)
        self.commands = []

        # Decide what to do with each of my tanks
        if self.do_this_once:
            print "Shooting team: %s" % self.enemies[0].color
            self.do_this_once = False
        self.shoot_target(0)

        # Send the commands to the server
        results = self.bzrc.do_commands(self.commands)

    # def move_to_position(self, bot, target_x, target_y):
    #     target_angle = math.atan2(target_y - bot.y,
    #             target_x - bot.x)
    #     relative_angle = self.normalize_angle(target_angle - bot.angle)
    #     command = Command(bot.index, 1, 2 * relative_angle, False)
    #     self.commands.append(command)

    # def normalize_angle(self, angle):
    #     '''Make any angle be between +/- pi.'''
    #     angle -= 2 * math.pi * int (angle / (2 * math.pi))
    #     if angle <= -math.pi:
    #         angle += 2 * math.pi
    #     elif angle > math.pi:
    #         angle -= 2 * math.pi
    #     return angle

    def shoot_target(self, team):
        '''Move every 3 to 8 seconds and then rotate by 60 degrees'''
        tenths_of_seconds_in_the_future = 10
        (current_loc, a, b, target) = self.get_target(team, tenths_of_seconds_in_the_future)

        # Graph what we have
        x = current_loc[0, 0]
        y = current_loc[3, 0]
        target_x = target[0, 0]
        target_y = target[3, 0]
        self.graph.add_point(x, y, a, b, target_x, target_y)

        #print "current loc: " + str(current_loc), "a: " +  str(a), "b: " +  str(b), "target: " + str(target)
        #print ""
        bot = self.mytanks[0]
        target_angle = math.atan2((target_y - y), (target_x - x))
        error_angle =  self.normalize_angle(target_angle - bot.angle)
        ang_vel = self.controller.update_with_error_and_dt(error_angle, self.stepTimeInSeconds)

        print target_angle, bot.angle, error_angle, ang_vel

        if((target_angle - bot.angle) < 0.01):
            #print "SHOOTING!"
            should_shoot = True
        else:
            should_shoot = False

        aim_and_shoot_command = Command(0, 0, ang_vel, should_shoot)
        self.bzrc.do_commands([aim_and_shoot_command])
#         time.sleep(23)

    def normalize_angle(self, angle):
        '''Make any angle be between +/- pi.'''
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    '''Uses Kalman Filter to get target several steps in the future'''
    def get_target(self, team, tenths_of_seconds_in_the_future):

        tempExpr = self.f * self.sigma_t * self.f_trans + self.sigma_x

        k_t1 = tempExpr * self.h_trans * np.linalg.inv((self.h * tempExpr * self.h_trans + self.sigma_z))
        #  k_t1 = np.linalg.inv(k_t1)
        #  sample the noisy position of the target

        other_tanks = self.bzrc.get_othertanks()
        z_t1 = np.matrix([[other_tanks[team].x],[other_tanks[team].y]])

        u_t1 = self.f * self.u_t + k_t1 * (z_t1 - self.h * self.f * self.u_t)

        sigma_t1 = (self.i - k_t1 * self.h) * tempExpr


        #  save new values for next iteration
        self.u_t = u_t1
        self.sigma_t = sigma_t1

        #  return a prediction i steps into the future
        for i in range(tenths_of_seconds_in_the_future):
            u_t1 = self.f * u_t1

        a = sigma_t1[0,0]
        b = sigma_t1[3,3]

        return self.u_t, a, b, u_t1

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

    agent = AgentKalmanFilter(bzrc)
    # Render graph now
    agent.graph.start_rendering()
    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
            time.sleep(agent.stepTimeInSeconds)

    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()
