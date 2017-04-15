'''ROS interface to the youBot'''
import logging

import better_exceptions
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import coloredlogs

logging.basicConfig(format='%(name)s @ [%(asctime)s] %(levelname)s:\t%(message)s')
coloredlogs.install(level='DEBUG')

DIRECTIONS = {
    'N': (1, 0),
    'S': (-1, 0),
    'E': (0, -1),
    'W': (0, 1)
}

SPEED = 0.2
CELL_TIME = 1.7 / 2

REAR_LASER_TOPIC = '/scan1'
FRONT_LASER_TOPIC = '/scan0'

OBSTACLE_BACK_TOPIC = "/obstacle_back"
OBSTACLE_FRONT_TOPIC = "/obstacle_front"
OBSTACLE_LEFT_TOPIC = "/obstacle_left"
OBSTACLE_RIGHT_TOPIC = "/obstacle_right"

RIGHT_INTERVAL = (450, 512)
LEFT_INTERVAL = (0, 62)
FRONT_INTERVAL = (206, 306)
REAR_INTERVAL = (206, 306)

LEFT_THRESHOLD = 0.4
RIGHT_THRESHOLD = 0.4
FRONT_THRESHOLD = 0.3
REAR_THRESHOLD = 0.5

REQUIRED_POINTS_FRAC = 0.5


class YouBot(object):
  '''The youBot object, encapsulating the ROS node '''
  def __init__(self):
    self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.stop_twist = Twist()
    self.stop_twist.linear.x = 0
    self.stop_twist.linear.y = 0
    self.rear_sub = rospy.Subscriber(REAR_LASER_TOPIC, LaserScan, self.rear_callback)
    self.front_sub = rospy.Subscriber(FRONT_LASER_TOPIC, LaserScan, self.front_callback)
    self.wall_states = {'N': False, 'E': False, 'S': False, 'W': False}

  def check_obstacles(self):
    '''Check all directions for obstacles'''
    return [self.check_direction(direction) for direction in DIRECTIONS]

  def check_direction(self, direction):
    '''Check one direction for an obstacle'''
    return self.wall_states[direction]

  def move(self, direction):
    '''Move one cell in a given direction'''
    twist = Twist()
    x_mul, y_mul = DIRECTIONS[direction]
    twist.linear.x = x_mul * SPEED
    twist.linear.y = y_mul * SPEED
    self.vel_pub.publish(twist)
    rospy.sleep(CELL_TIME)
    self.vel_pub.publish(self.stop_twist)

  def rear_callback(self, data):
    '''Handle data from the rear lidar'''
    rear_col = is_obstacle(data.ranges, REAR_INTERVAL, REAR_THRESHOLD)
    left_col = is_obstacle(data.ranges, LEFT_INTERVAL, LEFT_THRESHOLD)
    right_col = is_obstacle(data.ranges, RIGHT_INTERVAL, RIGHT_THRESHOLD)

    self.wall_states['S'] = rear_col
    self.wall_states['W'] = left_col
    self.wall_states['E'] = right_col

  def front_callback(self, data):
    '''Handle data from the front lidar'''
    front_col = is_obstacle(data.ranges, FRONT_INTERVAL, FRONT_THRESHOLD)
    self.wall_states['N'] = front_col

  def kill_speed(self):
    self.vel_pub.publish(self.stop_twist)


def is_obstacle(ranges, interval, dst_threshold):
  '''Check if a set of readings constitute an obstacle'''
  count = 0
  points = interval[1] - interval[0]
  point_count_threshold = points * REQUIRED_POINTS_FRAC
  for i in range(interval[0], interval[1]):
    if ranges[i] < dst_threshold:
      count += 1
  return count > point_count_threshold
