#!/usr/bin/python
'''The interpreter for the Picobot language'''
import logging
import re
from itertools import groupby
from sys import stdin

import better_exceptions
import fire
import rospy

import coloredlogs
import youbot

logging.basicConfig(format='%(name)s @ [%(asctime)s] %(levelname)s:\t%(message)s')
coloredlogs.install(level='INFO')


def collect_state_data():
  '''Reads a Picobot program from stdin'''
  print("Enter your program's states, one line at a time. End with 'q'")
  state_data = []
  for state in stdin:
    state = state.strip()
    if state == 'q':
      return state_data
    state_data.append(state)


def parse(state_data):
  '''Given a set of strings each representing a state, remove comments and
  parse out the relevant parts of each state into a dictionary, grouped by
  state number'''
  log = logging.getLogger('picobot-parser')
  state_regex = re.compile(
      r'(?P<state_num>[0-9]+)\s+(?P<sensor_state>[xXnNsSwWeE*]{4})\s+->\s+(?P<direction>[nNsSwWeE])\s+(?P<new_state>[0-9]+)'
  )
  comment_regex = re.compile(r'.*#.*$')
  log.info('Parsing state data')
  state_data = [state_regex.match(state) for state in state_data if not comment_regex.match(state)]
  state_data = [match.groupdict() for match in state_data if match]

  def get_state_num(match_dict):
    '''Utility function to return the state number from a regex match'''
    return match_dict['state_num']

  log.info('Grouping parsed states')
  return {key: list(group) for key, group in groupby(state_data, get_state_num)}


def load_states(state_filename):
  '''Read a state file if one was provided, else prompt for a set of states on
  stdin. Return the parsed state data.'''
  log = logging.getLogger('picobot-loader')
  if state_filename:
    log.info(f'Loading states from {state_filename}')
    with open(state_filename, 'r') as state_file:
      state_data = state_file.readlines()
  else:
    log.info('Reading state data from stdin')
    state_data = collect_state_data()

  return parse(state_data)


def make_state_machine(states):
  '''Given parsed state data, form a state machine mapping states to pairs of
  directions to check and possible sensor states. Each sensor state maps a
  4-tuple to a direction in which to drive and a new state to which to transition.'''
  log = logging.getLogger('picobot-constructor')
  machine = {}
  log.info('Building state machine from grouped states')
  for state in states:
    state_edges = states[state]
    direction_sets = [edge['sensor_state'].upper() for edge in state_edges]
    directions = []
    for dir_state in direction_sets:
      if dir_state[0] != '*':
        directions.append('N')
      if dir_state[1] != '*':
        directions.append('E')
      if dir_state[2] != '*':
        directions.append('W')
      if dir_state[3] != '*':
        directions.append('S')
    directions = set(directions)
    sensor_states = {
        edge['sensor_state'].upper(): (edge['direction'].upper(), edge['new_state'])
        for edge in state_edges
    }

    log.debug(directions)
    log.debug(sensor_states)
    log.debug(direction_sets)
    machine[state] = (directions, sensor_states, direction_sets)

  log.info('Assembled state machine')
  return machine


def transition(sensor_state, ordered_states):
  '''Check a sensor state against the transition states of a state, to deal
  with translating *s'''
  for state in ordered_states:
    match_flag = True
    for direction in range(4):
      match_flag = match_flag and\
          (state[direction] == sensor_state[direction] or state[direction] == '*')
    if match_flag:
      return state

  return 'No such state'


def run_state_machine(machine):
  '''Execute the state machine by transitioning between states until a
  terminal state is reached'''
  log = logging.getLogger('picobot-execute')
  state = '0'
  you_bot = youbot.YouBot()
  direction_map = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
  while True:
    directions, transition_states, ordered_states = machine[state]
    direction_states = [(direction, you_bot.check_direction(direction)) for direction in directions]
    sensor_state = list('****')
    for dir_name, dir_val in direction_states:
      sensor_state[direction_map[dir_name]] = dir_name if dir_val else 'X'

    sensor_state = ''.join(sensor_state)
    log.info(f'Read sensor values: {sensor_state}')
    direction, new_state = transition_states[transition(sensor_state, ordered_states)]
    if direction == 'X':
      return

    log.info(f'Driving {direction}')
    you_bot.move(direction)
    log.info(f'Transitioning to state {new_state}')
    state = new_state


def main(states_file):
  '''The primary entry point'''
  log = logging.getLogger('picobot')
  log.setLevel(logging.INFO)
  log.info('Constructing state machine')
  states = load_states(states_file if states_file else None)
  state_machine = make_state_machine(states)

  log.info('Starting execution')
  rospy.init_node('picobot', anonymous=True)
  run_state_machine(state_machine)
  log.info('Terminal state reached')
  rospy.spin()


if __name__ == '__main__':
  fire.Fire(main)
