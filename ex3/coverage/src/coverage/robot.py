#!/usr/bin/python


import rospy, math
from geometry_msgs.msg import Twist

class TraverseBot:

    ANGLE = math.radians(90)
    RATE = 10.0
    LEFT=1
    RIGHT=-1
    
    def __init__(self, girth):
        self.girth = girth
        rospy.init_node("coverage_node")
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
            
    def turn_left(self):
        self.turn(self.LEFT)
    
    def turn_right(self):
        self.turn(self.RIGHT)
            
    def go_forward(self):
        twist = Twist()
        twist.linear.x = self.girth
        self.move(twist)


class DirectionalBot(TraverseBot):

    def __init__(self, girth):
        TraverseBot.__init__(self, girth)

    def go_north(self):
        self.go_forward()
    def go_east(self):
        self.turn_right()
        self.go_forward()
        self.turn_left()
    def go_south(self):
        self.turn_right()
        self.turn_right()
        self.go_forward()
        self.turn_left()
        self.turn_left()
    def go_west(self):
        self.turn_left()
        self.go_forward()
        self.turn_right()
    def go(self, direction):
        print "going "+direction
        eval("self.go_"+direction)()
    
