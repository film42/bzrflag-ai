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

class AgentDumb(object):

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
        self.threads = []
        for bot in mytanks:
            thread = Thread(target = self.move_or_rotate, args = (bot,))
            thread.start()
            self.threads.append(thread)

        # Wait for threads to finish
        [thread.join() for thread in self.threads]

        # Send the commands to the server
        results = self.bzrc.do_commands(self.commands)

    def move_or_rotate(self, bot):
        '''Move every 3 to 8 seconds and then rotate by 60 degrees'''
        stop_moving_command = Command(bot.index, 0, 0, True)
        self.bzrc.do_commands([stop_moving_command])

        start_rotate_command = Command(bot.index, 0, 1, False)
        self.bzrc.do_commands([start_rotate_command])
        time.sleep(1)

        stop_rotate_command = Command(bot.index, 0, 0, False)
        self.bzrc.do_commands([stop_rotate_command])

        time.sleep(random.random())

        start_moving_command = Command(bot.index, 1, 0, True)
        self.bzrc.do_commands([start_moving_command])
        time.sleep(1.5 + random.random())
        start_moving_command = Command(bot.index, 1, 0, True)


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

    agent = AgentDumb(bzrc)
    prev_time = time.time()

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
