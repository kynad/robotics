#!/usr/bin/python

import rospy, actionlib, my_utils, sys
from formation.msg import RobotLocation, start_followAction, start_followGoal

GROUP_STATUS = {}

def find_nearest_robot(robot1, exclude=[]):
    robot1_location = GROUP_STATUS[robot1]
    return sorted(set(GROUP_STATUS)-set(exclude), key=lambda key: my_utils.distance(GROUP_STATUS[key], robot1_location))[0]


class Monitor:

    def __init__(self, number_of_robots, leader_id):
        self.number_of_robots = number_of_robots
        self.leader_id = leader_id
        self.group_order = [self.leader_id]
        rospy.Subscriber("/monitor_topic", RobotLocation, self.monitor_callback)
        print "Working with %d robots. robot_%d is the group's leader." % (self.number_of_robots, self.leader_id)
        self.rate = rospy.Rate(10)
        self.wait_for_the_robots_to_update_location()
        print self.group_order
        self.clients = self.init_clients()
        print "servers online"
        
    def monitor_callback(self, data):
       GROUP_STATUS[data.robot_id] = (data.x_coord, data.y_coord)
       rospy.loginfo("Found robot %d, at (%f,%f)" % (data.robot_id, data.x_coord, data.y_coord))
        
    def init_clients(self):
        clients = {}
        for _id in self.group_order:
            clients[_id] = actionlib.SimpleActionClient('start_following_%d' % _id, start_followAction)
            clients[_id].wait_for_server()
        return clients
        
    def wait_for_the_robots_to_update_location(self):
        num_ready = len(GROUP_STATUS)
        while num_ready < self.number_of_robots:
            print "still waiting for all robots in the group to be ready, currently only %d are" % num_ready
            self.rate.sleep()
            num_ready = len(GROUP_STATUS)
        print "Ready!"
        self.build_list_based_on_distances()       
        
    def send_goals_to_group(self):
        goal = start_followGoal
        temp_list = [self.leader_id] + self.group_order
        for client_id,local_leader in zip(temp_list[1:], temp_list[:-1]):
            goal.local_leader = local_leader
            self.clients[client_id].send_goal(goal)
            
    def wait_for_results(self):
        map(lambda client: client.wait_for_result(), self.clients.values())
        
    def build_list_based_on_distances(self):
        for i in range(self.number_of_robots - 1):
            self.group_order.append(find_nearest_robot(self.group_order[-1], self.group_order))
        



if __name__ == "__main__":
    rospy.init_node("monitor")
    group_size = rospy.get_param("number_of_robots")
    leader_id = rospy.get_param("leader_num")
    monitor = Monitor(group_size, leader_id)
    monitor.send_goals_to_group()
    monitor.wait_for_results()
    
