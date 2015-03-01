#!/usr/bin/python
import math
from Position import Position
from Square import square
from Square2d import Square2d

class mapreader:
    
#resolution is the length in meters of each square

#I should get from the file all these parameters
    left_bottom_x=0.0
    left_bottom_y=0.0

    def __init__(self, origin_map, origin_resolution, robot_width, robot_height, robot_x, robot_y):

        print "initial position is (%f, %f)" % (robot_x, robot_y)

        self.robot_hight = robot_height
        self.robot_width = robot_width

        self.robot_location_x = robot_x
        self.robot_location_y = robot_y

        self.small_map_square_resolution = origin_resolution
        self.highresul_map = origin_map
        self.small_map_width_in_meters = len(self.highresul_map[0]) * origin_resolution
        self.small_map_hight_in_meters = len(self.highresul_map) * origin_resolution

        print "Validate width and hight: %f & %f" % (self.small_map_width_in_meters, self.small_map_width_in_meters) 
        
        self.map_2D_resolution= 2*max( self.robot_width,  self.robot_hight)
        self.map_D_resolution=self.map_2D_resolution/2.0

        self.topleft_x=self.left_bottom_x+self.map_D_resolution/2; #X coordinate of the middle point of top left square
        self.topleft_y=self.left_bottom_y+self.small_map_hight_in_meters-self.map_D_resolution/2;
        print "small map width is %f and height is %f" % (self.small_map_width_in_meters, self.small_map_hight_in_meters )
        self.map_2D_dimention_width =int(math.floor(self.small_map_width_in_meters/self.map_2D_resolution))
        self.map_2D_dimention_hight =int(math.floor(self.small_map_hight_in_meters/self.map_2D_resolution))
        print "map_2D width is %d and height is %d" % (self.map_2D_dimention_width, self.map_2D_dimention_hight)
        self.map_2D = [ [ 0 for j in range(self.map_2D_dimention_width)] for i in range(self.map_2D_dimention_hight)]
        self.map_1D_dimention_width=2*self.map_2D_dimention_width;
        self.map_1D_dimention_hight=2*self.map_2D_dimention_hight;
        
        
        self.robots_j_2DMap = int(math.floor((self.robot_location_x-self.left_bottom_x) / self.map_2D_resolution))
       
        self.robots_i_2DMap = int(math.floor((self.small_map_hight_in_meters-self.robot_location_y) / self.map_2D_resolution))
    
        self.robots_j_DMap = int(math.floor((self.robot_location_x-self.left_bottom_x) / self.map_D_resolution))
        print "Map D resolution: ",self.map_D_resolution
        self.robots_i_DMap = int(math.floor((self.small_map_hight_in_meters-self.robot_location_y) / self.map_D_resolution))
    
        #Run
        self.map_twoD_creator() #creates map of squares in size 2d
        
        self.Create_path_int_2dmap()
        
        self.TranslateTo1d()#Fill the 1d array based on the path created
        self.Is_Robot_located_right() #Check if the robot located in a free 2D X 2D Square
        self.print1d()
        self.path = self.CreatePath()

    def map_twoD_creator(self):
        #notice that number of small squares not must divide to D.
        #ignore the left overs!!!!****        
        num_of_small_squares_in_a_d_square_width=int(self.map_2D_resolution/self.small_map_square_resolution)
        num_of_small_squares_in_a_d_square_hight=int(self.map_2D_resolution/self.small_map_square_resolution)
        print "num of squares in a 2d squre width is %d and height is %d" % (num_of_small_squares_in_a_d_square_width, num_of_small_squares_in_a_d_square_hight)
        
        # create occupation map where each square is in the size of 2DX2D
        for i in range(self.map_2D_dimention_hight):
            for j in range(self.map_2D_dimention_width):
                for k in range(num_of_small_squares_in_a_d_square_hight):
                    for m in range(num_of_small_squares_in_a_d_square_width):
                        if self.highresul_map[k+i*num_of_small_squares_in_a_d_square_hight][m+j*num_of_small_squares_in_a_d_square_width] > 0:
                            self.map_2D[i][j] = 1

        
        # Print the occupation map
        self.printMatrix(self.map_2D, self.map_2D_dimention_width, self.map_2D_dimention_hight)
        
        
    def Is_Robot_located_right(self):
        if self.map_2D[self.robots_i_2DMap][self.robots_j_2DMap]==1:
            print "Hey!!! Stop!! Robot is located in an occupied square. Thats not how STC works.. ):"
            
    def printMatrix(self, matrix, width, height):
        output = ""
        for i in range(height):
            for j in range(width):
                output+=str(matrix[i][j])
            output+="\n"
        print output
            
    def Create_path_int_2dmap(self):
        # /* In this function I'll use few helping arrays
        # * 1. nodesof2d  - This map contains the nodes with the walls for all the 2D tiles
        # * 2. visitedMap - This array contains 0 if node is not yet part of the planned route
        # */
        
        self.nodesof2d = [[None for j in range(self.map_2D_dimention_width)] for i in range(self.map_2D_dimention_hight)]
        visitedMap = self.map_2D[:]
        print "the Center coordinates of the top left square are: X- "+str(self.topleft_x)+" Y- "+str(self.topleft_y)
        #self.printMatrix(visitedMap, self.map_2D_dimention_width, self.map_2D_dimention_hight)
        
        
        #Start STC from robots location
        
        # TODO: convert to initial position
        first = self.Create_a_node( self.robots_i_2DMap, self.robots_j_2DMap) 
        visitedMap[self.robots_i_2DMap][self.robots_j_2DMap]= 2;    #//this location is visited
        self.nodesof2d[self.robots_i_2DMap][self.robots_j_2DMap]=first;
                    
        #       /*Create a path algorithm:
        #           *    1.    Push S into stack 
        #           *    2.    If stack is empty Finish
        #           *    3.    Pop from S ->curr
        #           *    4.    For each neighbor of curr
        #           *        a. If there is a neighbor
        #           *        b.         if neighbor is not visited
        #           *        c.            Create a new-node
        #           *        e.            update inner walls of curr
        #           *        f.            update inner walls of new-node
        #           *        g.            add to square2d(nodesof2d) array
        #           *        h.            push new_node into stack
        #           *        i.            update visited
        #           *    5. return to 2
        #           */
              
        #     /*Create a path algorithm:
        #      *    1.    Push S into stack 
        #     **/

        Stack = []
        Stack.append(first); 

        #            //2.    If stack is empty Finish
        neighbor_i=0
        neighbor_j=0
        while len(Stack) != 0:
            #             //3.    Pop from S ->curr
            curr_node=Stack.pop()
     
            #       //4.    For each neighbor of curr
            for k in range(1,5):
                if k==1: #up
                    neighbor_i=curr_node.i-1
                    neighbor_j=curr_node.j
                if k==2: #right
                    neighbor_i=curr_node.i
                    neighbor_j=curr_node.j+1
                if k==3: #down
                    neighbor_i=curr_node.i+1
                    neighbor_j=curr_node.j
                if k==4: #left
                    neighbor_i= curr_node.i
                    neighbor_j= curr_node.j-1
             
                if 0 <= neighbor_i < self.map_2D_dimention_hight and 0 <= neighbor_j < self.map_2D_dimention_width:
                    #//a. If there is a neighbor
                    if visitedMap[neighbor_i][neighbor_j]==0:
                        #                      //b.if neighbor is free for a visit
                        #                      //c.            Create a new-node
                        new_node = self.Create_a_node( neighbor_i, neighbor_j);
                        #                      //e.            update inner walls of curr
                        #                      //f.            update inner walls of new-node
                        if k==1: #//up from curr_node to new_node
                            curr_node.block_inner_wall(1) #;//block up
                            new_node.block_inner_wall(3) #;//block down
                        if k==2: #//right from curr_node to new_node
                            curr_node.block_inner_wall(2) #//block right
                            new_node.block_inner_wall(4)  #//block left
                        if k==3: #//down from curr_node to new_node
                            curr_node.block_inner_wall(3) #;//block down
                            new_node.block_inner_wall(1)  #;//block up
                        if k==4: #//left from curr_node to new_node
                            curr_node.block_inner_wall(4) #;//block left
                            new_node.block_inner_wall(2) #;//block right
                            #//g. add to nodesof2d array
                        self.nodesof2d[new_node.i][new_node.j]=new_node
                        self.nodesof2d[curr_node.i][curr_node.j]=curr_node
                        #//h. push new_node into stack
                        Stack.append(new_node);
                        #//i. update visited
                        visitedMap[new_node.i][new_node.j]=2
        #self.print2d();  

    def TranslateTo1d(self):
        self.nodesof1d = [[ None for j in range(self.map_1D_dimention_width)] for i in range(self.map_1D_dimention_hight)]
        for i in range(self.map_2D_dimention_hight):
            for j in range(self.map_2D_dimention_width):
                if self.nodesof2d[i][j] is None:
                    self.nodesof1d[2*i][2*j]=None;
                    self.nodesof1d[2*i+1][2*j]=None;
                    self.nodesof1d[2*i+1][2*j+1]=None;
                    self.nodesof1d[2*i][2*j+1]=None;
                else:
                    self.nodesof1d[2*i][2*j]= square(self.nodesof2d[i][j], 1, self.map_D_resolution);
                    self.nodesof1d[2*i][2*j+1]= square(self.nodesof2d[i][j], 2, self.map_D_resolution);
                    self.nodesof1d[2*i+1][2*j+1]= square(self.nodesof2d[i][j], 3, self.map_D_resolution);
                    self.nodesof1d[2*i+1][2*j]= square(self.nodesof2d[i][j], 4, self.map_D_resolution);

    def CreatePath(self):
        
        #start walk from robots location
        walking = self.nodesof1d[self.robots_i_DMap][self.robots_j_DMap]
        
        #p = Position(walking)
        Positions_stack = []
        
        """   
        /*
         * create a vector of points:
         * 
         * 1. for each wall search if open
         * 2. if no one open -> finish
         * 3. else:
         * 4.    close the wall at current
         * 5.    move to next in the direction of the wall
         * 5.    close the wall at next in the direction arrived from
         * 7. add position to vector
         * 8. return to 1
         *
         */
         """    
        while not walking.is_all_blocked():
            for direction in range(1,5):
                if walking.is_it_open(direction):
                    if direction == 1: #up
                        walking.ceiling=True #block the direction going to
                        self.nodesof1d[walking.i][walking.j] = walking
                        walking =self.nodesof1d[walking.i-1][walking.j] #move to next square
                        walking.floor=True #block the direction arrived from
                    if direction==2: #right
                        walking.right=True #block the direction going to
                        self.nodesof1d[walking.i][walking.j] = walking
                        walking =self.nodesof1d[walking.i][walking.j+1] #move to next square
                        walking.left=True #block the direction arrived from
                    if direction==3: #down
                        walking.floor=True #block the direction going to
                        self.nodesof1d[walking.i][walking.j] = walking
                        walking =self.nodesof1d[walking.i+1][walking.j] #move to next square
                        walking.ceiling=True #block the direction arrived from
                    if direction==4: #//left
                        walking.left=True #block the direction going to
                        self.nodesof1d[walking.i][walking.j] = walking
                        walking =self.nodesof1d[walking.i][walking.j-1] #move to next square
                        walking.right=True #block the direction arrived from
                    Positions_stack.append(Position(walking))
                    break
        
        print
        for position in Positions_stack:
            print "(%d,%d)" % (position.i, position.j)

        return Positions_stack


    def print2d(self):
        output = "\n"
        for i in range(self.map_2D_dimention_hight):
            for k in range(3):
                for j in range(self.map_2D_dimention_width):
                    node = self.nodesof2d[i][j]
                    if node is None:
                        output+="XXX"
                    else:
                        if k == 0:
                            if node.inner_up:
                                output += "010"
                            else:
                                output+="000"
                        elif k == 1:
                            left = "1" if node.inner_left else "0"
                            right = "1" if node.inner_right else "0"
                            output+=left+"1"+right
                        elif k == 2:
                            if node.inner_down:
                                output+= "010"
                            else:
                                output+="000"
                output+="\n"
        print output

    def print1d(self):
        print "Robots i, j are: ", self.robots_i_DMap, self.robots_j_DMap
        print "Robots x, y are: ", self.robot_location_x , self.robot_location_y
        output = "\n"
        for i in range(self.robots_i_DMap-20, self.robots_i_DMap+20): #self.map_2D_dimention_hight*2):
            for k in range(3):
                for j in range(self.robots_j_DMap-20, self.robots_j_DMap+20): #self.map_2D_dimention_width*2):
                    node = self.nodesof1d[i][j]
                    if node is None:
                        output += "XXX"
                    else:
                        if k == 0:
                            if node.ceiling:
                                output += "111"
                            else:
                                output += "1 1"
                        elif k == 1:
                            left = "1" if node.left else " "
                            right = "1" if node.right else " "
                            if i==self.robots_i_DMap and j==self.robots_j_DMap:
                                output += left+"R"+right
                            else:
                                output += left+" "+right
                        elif k == 2:
                            if node.floor:
                                output += "111"
                            else:
                                output += "1 1"
                output+="\n"
        print output


    def Create_a_node(self, i, j):
        CurX = self.topleft_x + j * self.map_2D_resolution
        CurY = self.topleft_y - i * self.map_2D_resolution
        print "DEbug: creating a 2d node in i=%d and j=%d, at (%f,%f)" % (i,j,CurX,CurY)
        return Square2d(i, j, CurX, CurY)

     
if __name__ == "__main__":
    mapreader()
    
