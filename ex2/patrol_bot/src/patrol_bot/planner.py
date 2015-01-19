#!/bin/python

import sys, rospy, utils
from geometry_msgs.msg import Point
from nav_msgs.srv import GetPlan


## A route representation
#
# Gets a list of points, and calculates a circular route (an order by which these points need to be visisted)
# Provides a method to iterate over these points and a counter property that tracks the number of points visited in each round.
class Route:

    MAKE_PLAN_SERVICE_NAME = "move_base_node/make_plan"

    def __init__(self, current_location):
        self.__plan_client = rospy.ServiceProxy(self.MAKE_PLAN_SERVICE_NAME, GetPlan)
        rospy.wait_for_service(self.MAKE_PLAN_SERVICE_NAME)
        self.__points = utils.get_points_list_from_rosparams()
        self.__route = {}
        self.__distances = {}
        self.__current_point = utils.Point(*current_location)
        self.__find_shortest_path()
        self.__points_counter = -1

    @property
    def per_round_points_counter(self):
        return self.__points_counter % len(self.__points)

    ## Uses GetPlan to calculate the real distance between two points.
    def __get_distance(self, first_point, second_point):
        start = utils.create_pose(first_point)
        goal = utils.create_pose(second_point)
        tolerance = 0.5
        responce = self.__plan_client(start=start, goal=goal, tolerance=tolerance)
        points = map(lambda x: x.pose.position, responce.plan.poses)
        return sum([utils.distance(points[i+1], points[i]) for i in xrange(len(points)-1)])

    ## Selects a closest point from a list
    #
    # @return the closest point (from points_list) to current_point and the distance to that point.
    def __nearest_point(self, current_point, points_list):
        distances = map(self.__get_distance, [current_point]*len(points_list), points_list)
        result = min(zip(distances, points_list))
        return tuple(reversed(result))

    ## Finds a route between all points contained in self.__points.
    # Fills self.__route and self.__distances accordingly.
    #
    # Uses Nearest Neighbour algorithm for the route calculation.
    def __find_shortest_path(self):
        points = set(self.__points)
        visited_points = set()
        current_point = self.__current_point
        while not points.issubset(visited_points):
            self.__route[current_point], self.__distances[current_point] = self.__nearest_point(current_point, points - visited_points)
            current_point = self.__route[current_point]
            visited_points.add(current_point)
        self.__route[current_point] = self.__route[self.__current_point]
        self.__distances[current_point] = self.__get_distance(current_point, self.__route[current_point])

    ## Gets the next point in route to go to
    #
    # @return the next point in route and distance to travel to that point.
    def next_in_route(self):
        distance = self.__distances.get(self.__current_point)
        self.__current_point = self.__route.get(self.__current_point)
        self.__points_counter += 1
        return self.__current_point, distance
