'''ROS interface to the youBot'''
import logging

import better_exceptions
import rospy
from geometry_msgs.msg import Twist

import coloredlogs

logging.basicConfig(format='%(name)s @ [%(asctime)s] %(levelname)s:\t%(message)s')
coloredlogs.install(level='DEBUG')

DIRECTIONS = {
    'N': (0, 1),
    'S': (0, -1),
    'E': (1, 0),
    'W': (-1, 0)
}

SPEED = 0.1
CELL_TIME = 3.0


class YouBot(object):
  '''The youBot object, encapsulating the ROS node '''
  def __init__(self):
    self.vel_pub = rospy.Publisher('cmd_vel', Twist)
    self.stop_twist = Twist()
    self.stop_twist.linear.x = 0
    self.stop_twist.linear.y = 0
    # TODO: Make field for obstacle detection service

  def check_obstacles(self):
    '''Check all directions for obstacles'''
    return [self._check_direction(direction) for direction in DIRECTIONS]

  def _check_direction(self, direction):
    '''Check one direction for an obstacle'''
    # TODO: Call obstacle detection service
    pass

  def move(self, direction):
    '''Move one cell in a given direction'''
    twist = Twist()
    x, y = DIRECTIONS[direction]
    twist.linear.x = x * SPEED
    twist.linear.y = y * SPEED
    self.vel_pub.publish(twist)
    rospy.sleep(CELL_TIME)
    self.vel_pub.publish(self.stop_twist)
