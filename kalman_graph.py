from matplotlib.patches import Ellipse
from matplotlib.pylab import *
import matplotlib.pyplot as plt
import numpy as np
import random

plt.ion()

class KalmanGraph:
    def __init__(self):
        self.projected_points_x = []
        self.projected_points_y = []

    def add_point(self, x, y, width, height, predicted_x, predicted_y):
        self.projected_points_x.append(predicted_x)
        self.projected_points_y.append(predicted_y)

        # TODO: Make sure the angle doesn't mess up our W and H
        ellipse = Ellipse(xy=(x, y), width=width, height=height, angle=0)
        self.ax.add_artist(ellipse)

        # Re-render
        self._update_open_graph()

    def start_rendering(self):
        print "Opening Plot..."
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([], [], 'go')
        # Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set_xlim(-400, 400)
        self.ax.set_ylim(-400, 400)
        #self.ax.get_yaxis().set_visible(False)
        #self.ax.get_xaxis().set_visible(False)
        # Other stuff
        self.ax.grid()

    def _update_open_graph(self):
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(self.projected_points_x)
        self.lines.set_ydata(self.projected_points_y)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
