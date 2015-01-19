import datetime, rospy, math
from geometry_msgs.msg import PoseStamped, Point

## Simple logging to file mechanism.
class Logger:
    def __init__(self, name):
        self.__logname = name
        self.clear()

    ## Appends the log msg to a logfile.
    def log(self, msg, *args):
        with open(self.__logname, "a") as log:
            log.write("[%s] " % datetime.datetime.now().isoformat())
            log.write(msg % args)
            log.write("\n")

    ## Wipes the logfile empty.
    def clear(self):
        with open(self.__logname, "w"):
            pass


## Extracts the list of points from launch file params.
# Assumes the points are represented as a list of x axises and a list of y axises.
#
# @return list of Point objects (geometry_msgs/Point.msg).
def get_points_list_from_rosparams():
    x_list = rospy.get_param("/x_locations")
    y_list = rospy.get_param("/y_locations")
    return map(Point, x_list, y_list, [0]*len(x_list))

## Creates and returns a valid PoseStamped object from a point.
# Fills the fields relevant for this exersize with default values.
def create_pose(point):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = point
    pose.pose.orientation.w = 1.0
    return pose

## Calculates the 2d Eucledian distance between two points
def distance(point1, point2):
    delta = Point(point1.x - point2.x, point1.y - point2.y, point1.z - point2.z)
    return math.sqrt(delta.x*delta.x + delta.y*delta.y)

