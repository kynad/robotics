#!/user/bin/python

class Position:

    def __init__(self, square):
        self.x = square.x
        self.y = square.y
        self.i = square.i
        self.j = square.j

    def get_coords(self):
        return (self.i, self.j)

        
