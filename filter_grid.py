import numpy as np
from matplotlib.pylab import *
import matplotlib.pyplot as plt
import time
import random

plt.ion()

class FilterGrid:
    def __init__(self, width, height,
                 default_value=0.5, new_target_delay=10,
                 true_positive=0.97, true_negative=0.9):
        # Origin is top left
        self.matrix = np.zeros((width, height))
        self.matrix += default_value
        self.width = width
        self.height = height
        self.default_value = default_value
        self.last_target_update = time.time()
        self.new_target_delay = new_target_delay
        self.new_target_value = (0, 0,)
        self.true_positive = true_positive
        self.true_negative = true_negative

    def update_not_occupied(self, x, y):
        current_prior = self.matrix[y, x]
        probability_not_occupied = self.true_negative * (1 - current_prior)
        probability_occupied = (1 - self.true_positive) * current_prior
        alpha = 1 / (probability_not_occupied + probability_occupied)
        new_posterior = probability_occupied * alpha
        self.matrix[y, x] = new_posterior

    def update_occupied(self, x, y):
        current_prior = self.matrix[y, x]
        probability_not_occupied = (1 - self.true_negative) * (1 - current_prior)
        probability_occupied = self.true_positive * current_prior
        alpha = 1 / (probability_not_occupied + probability_occupied)
        new_posterior = probability_occupied * alpha
        self.matrix[y, x] = new_posterior

    def update(self, x, y, input):
        # print "Current index: (%d, %d)" % (x, y,)

        # I'm dumb right now
        try:
            # Set a lower bound
            if self.matrix[y, x] <= 0.00001:
                self.matrix[y, x] = 0.1

            old_value = self.matrix[y, x]

            if input is 1:
                self.update_occupied(x, y)
            else:
                self.update_not_occupied(x, y)

            # Set a lower bound
            if self.matrix[y, x] <= 0.00001:
                self.matrix[y, x] = 0.1

            # print "Old value: %f\t\tNew value: %f" % (old_value, self.matrix[y, x])
            # print "New value at (%d, %d): %f" % (x, y, self.matrix[y, x])
        except:
            # pass
            print "Bad index: (%d, %d)" % (x, y,)

    # Returns an empty place in the grid for the robots to look
    def new_target(self):
        return (300, 300)

        if (time.time() - self.last_target_update) < self.new_target_delay:
            return self.new_target_value

        top_left_score = 0
        for y in xrange(self.height / 2):
            for x in xrange(self.width / 2):
                top_left_score += self.matrix[y][x]

        top_right_score = 0
        for y in range(self.height / 2):
            for x in range(self.width / 2, self.height):
                top_right_score += self.matrix[y][x]

        bottom_left_score = 0
        for y in range(self.height / 2, self.height):
            for x in range(self.width / 2):
                bottom_left_score += self.matrix[y][x]

        bottom_right_score = 0
        for y in range(self.height / 2, self.height):
            for x in range(self.width / 2, self.height):
                bottom_right_score += self.matrix[y][x]

        # Now compare to find the least explored area.
        min_value = min(top_left_score, top_right_score, bottom_left_score, bottom_right_score)

        if min_value is top_left_score:
            self.new_target_value = (-300, 300,)
        elif min_value is top_right_score:
            self.new_target_value = (300, 300,)
        elif min_value is bottom_left_score:
            self.new_target_value = (-300, -300,)
        else:
            self.new_target_value = (300, -300,)

        self.last_target_update = time.time()
        print "New Target Point: (%d, %d)" % self.new_target_value

        return self.new_target_value

    # Graphing
    def open_graph(self):
        self.on_launch()

    def update_graph(self):
        # TODO: Find out why this is drawn backwards
        copy_matrix = np.array(self.matrix[::-1])

        # Contstrain matrix to only show values above 0.5
        # for y in xrange(self.height):
        #     for x in xrange(self.width):
        #         if copy_matrix[y, x] <= 0.5:
        #             copy_matrix[y, x] = 0
        #         else:
        #             copy_matrix[y, x] = 1

        self.ax.matshow(copy_matrix, cmap=cm.gray)
        self._update_open_graph()

    def on_launch(self):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        # self.lines, = self.ax.plot([], [], 'o')
        # Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        # Other stuff
        self.ax.grid()

    def _update_open_graph(self):
        #Update data (with the new _and_ the old points)
        # self.lines.set_xdata(xdata)
        # self.lines.set_ydata(ydata)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
