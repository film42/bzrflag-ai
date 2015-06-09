#!/usr/bin/python -tt

from bzrc import BZRC, Command
import sys, math, time
from threading import Thread
import random

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

        self.shoot_target(1)

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
        tenths_of_seconds_in_the_future = 1000
        target = self.get_target(team, tenths_of_seconds_in_the_future)
        
        angVel = math.sin(time.time());
        print angVel
        aim_and_shoot_command = Command(0, 0, angVel, True)
        self.bzrc.do_commands([aim_and_shoot_command])
#         time.sleep(23)

    '''Uses Kalman Filter to get target several steps in the future'''
    def get_target(self, tenths_of_seconds_in_the_future):
        f = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
        sigma_t = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
        h = [[0,0,0,0,0,0],[0,0,0,0,0,0]]
        sigma_x = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
        sigma_z = [[0,0],[0,0]]

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
    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
            time.sleep(1)

    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()
