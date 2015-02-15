#!/usr/bin/python

class Square2d:
    
    UP = 1
    RIGHT = 2
    DOWN = 3
    LEFT = 4
    
    def __init__(self, i, j, x, y):
        self.i, self.j, self.x_cord, self.y_cord = i, j, x, y
        self.ceiling_left = self.ceiling_right = True
        self.floor_left = self.floor_right = True
        self.left_top = self.left_bottom = True
        self.right_top = self.right_bottom = True
        self.inner_up=self.inner_right=self.inner_down=self.inner_left=False

    def block_inner_wall(self, direction):
        #this function takes care of blocking inner walls
        #and opening relevant outer walls
        if (direction==self.UP):
            self.inner_up=True
            self.ceiling_left=self.ceiling_right=False
        elif (direction==self.RIGHT):
            self.inner_right=True
            self.right_top=self.right_bottom=False
        elif (direction==self.DOWN):
            self.inner_down=True
            self.floor_left=self.floor_right=False
        elif (direction==self.LEFT):
            self.inner_left=True
            self.left_top=self.left_bottom=False
        else:
            raise Exception("")
        
              
            
