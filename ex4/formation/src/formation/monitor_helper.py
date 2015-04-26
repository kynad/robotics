#!/usr/bin/python

import rospy, actionlib, my_utils
from formation.msg import StartFollowAction, StartFollowGoal

## @class RobotClient    
#  Represents a logical unit that helps the monitor node to communicate with the real robot node,
#  as well as performing distance calculations from other robots.
class RobotClient:
    ##
    # @param name     - the string identifier of the real robot
    # @param location - the coordinates of the robots location
    def __init__(self, name, location):
        self.name = name
        self.location = location
        self.client = actionlib.SimpleActionClient(my_utils.C_ACTION_SERVER_STR % name, StartFollowAction)
        self.client.wait_for_server()
        
    ## Sends a command to the real robot to start moving and whom it should follow.
    #  If it sends the command to follow yourself - it means that this robot should lead.
    #
    # @param leader_robot the name of the robot to follow
    def follow(self, leader_robot):
        goal = StartFollowGoal(leader_robot.name)
        self.client.send_goal(goal)
        
    ## Wait for the robot (that is a SimpleActionServer) to reply to the goal.
    #  In the scope of this exersize, there won't be any such reply, as the robots are expected to move indefinitely
    def wait_for_result(self):
        self.client.wait_for_result()
        
    ## Finds the distance from another robot
    #
    # @param other_robot the robot to calculate distance from
    #
    # @return 2D euclidean distance from other_robot
    def find_distance_from(self, other_robot):
        return my_utils.distance(self.location, other_robot.location)
        
    ## Finds the closest robot to self, out of a list of given robots.
    #
    # @param @robots_list - the list of the robots to look in.
    def find_nearest_robot_out_of(self, robots_list):
        sorted_list = sorted(robots_list, key=lambda x: self.find_distance_from(x))
        return sorted_list[0]


## @class GroupManager
#  A manager class for multiple robots.
#  Used to group multiple robot objects together and perform actions on them.
class GroupManager:            
    def __init__(self):
        self.robots = {}
        
    ## len() will return the number of robots in the group.
    def __len__(self):
        return len(self.robots)
        
    ## str() will return a list (as string) of robot names.
    def __str__(self):
        return str(self.robots.keys())
        
    ## Adds a robot to the group.
    #
    # @param robot_name     - the name of the robot to add.
    # @param robot_location - an (x,y) tuple of 2D coordinates of the robot's current position.
    def add_robot(self, robot_name, robot_location):
        self.robots[robot_name] = RobotClient(robot_name, robot_location)
        
    ## Builds a list of robots where robot #i should follow robot #i+1, ending with the leader.
    #  Will build a list of the form (leader -> nearest robot to leader -> nearest robot to the previous one -> ... -> last robot)
    #  and will reverse it.
    #
    # @param leader_name - the name of the leader robot.
    #
    # @return a list of robot names that complies to the above rule.
    def get_followers_list(self, leader_name):
        leader = self.robots[leader_name]
        followed_list = [leader]
        for i in range(len(self.robots) - 1):
            followed_list.append(leader.find_nearest_robot_out_of(set(self.robots.values()) - set(followed_list)))
            leader = followed_list[-1]
        return followed_list[::-1]
        
    ## Provide each robot with the name of it's immediate leader and tell all robots to start moving.
    #  Uses get_followers_list() to find the immediate leaders.
    #
    # @param leader_name - the name of the leading robot (the one that doesn't follow anyone).
    def send_actions(self, leader_name):
        followers_list = self.get_followers_list(leader_name) + [self.robots[leader_name]]
        for (robot, local_leader) in zip(followers_list[:-1], followers_list[1:]):
            robot.follow(local_leader)

    ## Delegates the call to wait_for_result() to every robot.
    def wait_for_results(self):
        for robot in self.robots.values():
            robot.wait_for_result()
        
    

