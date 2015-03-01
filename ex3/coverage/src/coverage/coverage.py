#!/usr/bin/python

from robot import DirectionalBot
from Mapreader import mapreader
from nav_msgs.srv import GetMap
import numpy, rospy


ROBOT_GIRTH = 0.333

def convert_occupance_values(values):
    # for now, just convert all numbers to positive.
    # TODO: decide later if there is a need to make all values either 0 or 100.
    return map(lambda x: x*x, values)

def values_derivative(values):
    print map(lambda x: x.get_position(), values)
    values = map(lambda x: x.get_coords(), values)
    return [tuple(numpy.subtract(values[i], values[i-1])) for i in xrange(len(values))]
    
def convert_path_to_directions(path):
    # take a path, i.e. a list of matrix indecies and convert it to directions (i.e. a list of "left", "right", or similar)
    derivatives = values_derivative(path)
    conversion_dict = {(-1,0) : "north", (0,1) : "east", (1,0) : "south", (0,-1) : "west"}
    result = []
    for delta in derivatives:
        if delta in conversion_dict:
            result.append(conversion_dict[delta])
        else:
            raise Exception("Wrong path")
    return result

def dense_line(line):
    result=[]
    for line_idx in xrange(len(line)):
        index = line_idx / 10
        if len(result) <= index: result.append(0)
        result[index] += line[line_idx]
    return result

def transpose(matrix):
    return zip(*matrix)

def diam():
    # This is to acount for the robot turns
    return 1.45*ROBOT_GIRTH

if __name__ == "__main__":
    map_client = rospy.ServiceProxy("static_map", GetMap)
    rospy.wait_for_service("static_map")
    responce = map_client()
    info = responce.map.info
    width = info.width
    height = info.height
    resolution = info.resolution
    print "width is %d, height is %d and resolution is %f meters" % (width, height, resolution)
    pgm_map = [convert_occupance_values(responce.map.data[i:i+width]) for i in xrange(0, width*height, width)]
    print "again, width is %d and height is %d" % (len(pgm_map), len(pgm_map[0]))

    #small_map = map(dense_line, pgm_map)
    #print small_map
    #small_map = transpose(map(dense_line, transpose(small_map)))

    #print "width of 2d is %d and height is %d" % (len(small_map), len(small_map[0]))
    #print small_map
    #for line in small_map:
    #    print "".join(map(lambda x: str(min(1,x)), line))
       

    robot_x = rospy.get_param('/amcl/initial_pose_x')
    robot_y = rospy.get_param('/amcl/initial_pose_y')

    print "initial pose is (%f,%f)" % (robot_x, robot_y)

    final_map = pgm_map
    final_map.reverse()
    #final_map= transpose(final_map)
    reader = mapreader(final_map, resolution, diam(), diam(), robot_x, robot_y)

    directions = convert_path_to_directions(reader.path)

    bot = DirectionalBot(ROBOT_GIRTH)
    index = 0
    bot.turn_left()
    bot.turn_left()
    bot.turn_left()
    bot.turn_left()

    
#    while not rospy.is_shutdown():
#         bot.go(directions[index])
#         index = (index + 1) % len(directions)
#         rospy.sleep(1)
