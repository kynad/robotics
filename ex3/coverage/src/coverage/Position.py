#!/user/bin/python

class Position:

    def __init__(self, square):
        self.x = square.x
        self.y = square.y
        self.i = square.i
        self.j = square.j

    def get_coords(self):
        return (self.i, self.j)

    def get_position(self):
        return (prescision(self.x, 2), prescision(self.y, 2))

def prescision(double, prescision):
    divisor = pow(10, prescision)
    return int(double*divisor)/(1.0*divisor)
