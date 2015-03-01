#!/usr/bin/python

class square:
        
    UP_LEFT= 1
    UP_RIGHT = 2
    DOWN_RIGHT = 3
    DOWN_LEFT = 4
    
    def __init__(self, square2d, direction, map_D_resolution):
        if direction == self.UP_LEFT:
            self.x = square2d.x_cord - map_D_resolution * 0.5
            self.y=square2d.y_cord+map_D_resolution*0.5
            self.i=square2d.i*2
            self.j=square2d.j*2
            self.ceiling=square2d.ceiling_left
            self.floor=square2d.inner_left
            self.right=square2d.inner_up
            self.left=square2d.left_top
        elif direction== self.UP_RIGHT:
            self.x=square2d.x_cord+map_D_resolution*0.5
            self.y=square2d.y_cord+map_D_resolution*0.5
            self.i=square2d.i*2+1
            self.j=square2d.j*2
            self.ceiling=square2d.ceiling_right
            self.floor=square2d.inner_right
            self.right=square2d.right_top
            self.left=square2d.inner_up
        elif direction==self.DOWN_RIGHT:
            self.x=square2d.x_cord+map_D_resolution*0.5
            self.y=square2d.y_cord-map_D_resolution*0.5
            self.i=square2d.i*2+1
            self.j=square2d.j*2+1
            self.ceiling=square2d.inner_right
            self.floor=square2d.floor_right
            self.right=square2d.right_bottom
            self.left=square2d.inner_down
        elif direction==self.DOWN_LEFT:
            self.x=square2d.x_cord-map_D_resolution*0.5;
            self.y=square2d.y_cord-map_D_resolution*0.5;
            self.i=square2d.i*2;
            self.j=square2d.j*2+1;
            self.ceiling=square2d.inner_left;
            self.floor=square2d.floor_left;
            self.right=square2d.inner_down;
            self.left=square2d.left_bottom;
        else:
            raise Exception("")

    def is_all_blocked(self):
        return self.ceiling and self.floor and self.right and self.left
    
    def is_it_open(self, direction):
        if direction==1 and self.ceiling==False:
            return True        
        if direction==2 and self.right==False:
            return True;
        if direction==3 and self.floor==False:
            return True;
        if direction==4 and self.left==False:
            return True;
        return False;
    
