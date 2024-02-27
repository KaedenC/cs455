import numpy as np

class Vector:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
    
    @property
    def magnitude(self):
        return np.sqrt((self.x**2) + (self.y**2))

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return Vector(self.x * other, self.y * other)

    def __truediv__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return Vector(self.x / other, self.y / other)
        
