import numpy as np

class FilterGrid:
    def __init__(self, width, height, default_value=0.5):
        self.matrix = np.zeros((width, height))
        self.matrix += default_value

    def update(self, x, y):
        pass
