#!/usr/bin/python

from robot import DirectionalBot
from Mapreader import mapreader
from nav_msgs.srv import GetMap
import numpy, rospy


ROBOT_GIRTH = 1

def values_derivative(values):
    values = map(lambda x: x.get_coords(), values)
    return [tuple(numpy.subtract(values[i-1], values[i])) for i in xrange(len(values))]
    
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
    

if __name__ == "__main__":
    map_client = rospy.ServiceProxy("static_map", GetMap)
    rospy.wait_for_service("static_map")
    responce = map_client()
    info = responce.map.info
    width = info.width
    height = info.height
    resolution = info.resolution
    pgm_map = [responce.map.data[i:i+width] for i in xrange(0, width*height, width)]

    robot_x = rospy.get_param('/amcl/initial_pose_x')
    robot_y = rospy.get_param('/amcl/initial_pose_y')

    reader = mapreader(pgm_map, resolution, ROBOT_GIRTH, ROBOT_GIRTH, robot_x, robot_y)

    directions = convert_path_to_directions(reader.path)

    bot = DirectionalBot(ROBOT_GIRTH)
    index = 0
    while not rospy.is_shutdown():
         bot.go(directions[index])
         index = (index + 1) % len(directions)
         rospy.sleep(1)
