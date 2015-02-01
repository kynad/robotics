#!/usr/bin/python

"""
This class will represent a robot that is aware of it's dimentions and is able to move to one of the four directions, advancing by it's own length each time.
"""

import rospy
from geometry_msgs.msg import Twist

class TraverseBot:

    ANGLE = 1.57
    RATE = 5.0
    LEFT=1
    RIGHT=-1
    
    def __init__(self, girth):
        self.girth = girth
        rospy.init_node("traverse_bot")
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=int(self.RATE))
        rospy.sleep(1)
        self.rate = rospy.Rate(self.RATE)

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)

    def move(self, twist):
        for i in xrange(int(self.RATE)):
            self.publisher.publish(twist)
            self.rate.sleep()
        self.stop()
            
    def turn(self, direction):
        twist = Twist()
        twist.angular.z = self.ANGLE * direction
        self.move(twist)
            
    def turnLeft(self):
        self.turn(self.LEFT)
    
    def turnRight(self):
        self.turn(self.RIGHT)
            
    def goForward(self):
        twist = Twist()
        twist.linear.x = self.girth / self.RATE
        self.move(twist)


if __name__ == "__main__":
    bot = TraverseBot(10)
    while not rospy.is_shutdown():
        bot.turnLeft()
        bot.goForward()
        bot.turnRight()
        bot.goForward()
