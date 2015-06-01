import numpy as np

class FilterGrid:
    def __init__(self, width, height, default_value=0.5):
        # Origin is top left
        self.matrix = np.zeros((width, height))
        self.matrix += default_value

    def update(self, x, y, value):
        pass

    # Returns an empty place in the grid for the robots to look
    def new_target(self):
        pass
