#!/usr/bin/python


import rospy, actionlib, utils, tf
from planner import Route
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

## The patrol_bot class
#
# Uses the Route class to plan a route and attempts to visit every point in that route, logging the results.
class Patrol:

    STEADY_TIME = 5.0

    LOG_RESULT_STRING = {True : "Succeded" , False : "Failed"}
    LOG_MSG = "%s to reach location #%d in this patroling round. Total distance traveled=%d. Total patroling time=%dsec."

    def __init__(self, logname, route_planner):
        self.route_planner = route_planner
        self.logger = utils.Logger(logname)
        self.mbclient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.wait_for_move_base_server()
        self.total_distance = 0.0

    ## Waits for move_base action server to become online.
    def wait_for_move_base_server(self):
        while not self.mbclient.wait_for_server(rospy.Duration(30)):
            rospy.logwarn("Waiting for move_base action server...")
        else:
            rospy.loginfo("Server is ready")

    ## Sends a goal to the move_base action server
    def send_goal(self, point):
        goal = MoveBaseGoal()
        goal.target_pose = utils.create_pose(point)
        self.mbclient.send_goal(goal)
        rospy.loginfo("Goal Sent\n%s", str(point))

    ## Does all the required action to advance to the next point in route.
    #
    # @return True if reached the next point in route and False otherwise.
    def advance(self):
        next_point, distance = self.route_planner.next_in_route()
        self.total_distance += distance
        self.send_goal(next_point)
        return self.mbclient.wait_for_result(rospy.Duration(30))

    ## The main loop:
    # Tries to advance to the next point in route, logs the result.
    # On success, stays in the point reached for self.steady_time seconds.
    def patrol(self):
        while not rospy.is_shutdown():
            step_result = self.advance()
            self.logger.log(self.LOG_MSG,
                            self.LOG_RESULT_STRING[step_result],
                            self.route_planner.per_round_points_counter + 1,
                            self.total_distance,
                            rospy.Time.now().secs)
            if step_result:
                rospy.sleep(self.STEADY_TIME)
            else:
                self.mbclient.cancel_goal()        
        

## Extracts the coordinates of the current location of the bot.
def find_current_location():
    dest_frame = "/map"
    targ_frame = "/base_footprint"
    listener = tf.TransformListener()
    listener.waitForTransform(dest_frame, targ_frame, rospy.Time(0), rospy.Duration(10.0))
    rate = rospy.Rate(10.0)
    try:
        (coords, rot) = listener.lookupTransform(dest_frame, targ_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
       rospy.logerr("Had the following error"+str(err))
    else:
        rospy.loginfo("Found self location %s" % str(coords))
    return coords


if __name__ == "__main__":
    rospy.init_node("patrol")
    router = Route(find_current_location())
    rospy.loginfo("Built a route")
    patrol_bot = Patrol("patrol_log", router)
    rospy.loginfo("Beginning patroling")
    patrol_bot.patrol()
        
