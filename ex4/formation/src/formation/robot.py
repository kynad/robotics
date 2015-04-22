#!/usr/bin/python

import rospy, my_utils, actionlib
from follower import Follower
from formation.msg import RobotLocation, start_followAction

# TODO: move to common/utils ?
def str_id_to_numerical(_id):
    return int(_id.split("_")[-1])

class Robot:
    def __init__(self, _id):
        self.id = _id
        self.numerical_id = str_id_to_numerical(self.id)
        self.publisher = rospy.Publisher("/monitor_topic", RobotLocation, queue_size=10, latch=True)
        self.start_server()
            
    def start_server(self):
        rospy.loginfo("starting server")
        self.server = actionlib.SimpleActionServer('/start_following_%d' % self.numerical_id, start_followAction, self.execute, False)
        self.server.start()
        rospy.loginfo("server started")

    def send_ready_status(self):
        x_y = my_utils.TransformListenerWrapper("/map", "/%s/base_footprint" % self.id).get_2d_coordinates()
        status_msg = RobotLocation(None, self.numerical_id, True, *x_y)
        print self.id+": sending ready message."
        self.publisher.publish(status_msg)
        
    def execute(self, goal):
        if goal.local_leader == self.numerical_id:
            self.take_the_lead()
        else:
            self.follow(goal.local_leader)
        self.server.set_succeeded()
    
    def take_the_lead(self):
        print "I'm the leader!"
    
    def follow(self, leader_id):
        print "I'm %d will follow %d" % (self.numerical_id, leader_id)
        follower = Follower(self.numerical_id)
        follower.follow(leader_id)
    

        
if __name__ == "__main__":
    _id = rospy.get_param("tf_prefix")
    rospy.init_node(_id)
    rospy.sleep(1)
    robot = Robot(_id)
    robot.send_ready_status()
    rospy.spin()

