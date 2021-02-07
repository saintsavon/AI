#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# NAME: Jaylen Scott
# DATE: February 7th, 2021
# COURSE: Artificial Intelligence
# SEMESTER: Spring 2021
# MP1: A* search algorithm implementation for a maze

"""
Created on Wed Jan 16 08:39:28 2019

@author: szczurpi

This program implements the uniform cost search algorithm for solving a maze
Allows moves in 4 directions (1 point cost for each move)
"""

import numpy as np
import queue  # Needed for frontier queue


class MazeState:
    # If this breaks, put '()' after MazeState. Going through corrections from PyCharm IDE - JS

    """
    Stores information about each visited state within the search
    """

    # Define constants
    SPACE = 0
    WALL = 1
    EXIT = 2
    #START = (1, 1)
    #END = (9, 1)

    # If it breaks, comment this out - JS
    #maze = np.array([
    #    [0, 1, 1, 0, 1],
    #    [0, 0, 0, 0, 1],
    #    [0, 0, 1, 1, 1],
    #    [1, 0, 0, 0, 0],
    #    [1, 1, 0, 1, 0],
    #    [1, 0, 0, 1, 0],
    #    [1, 0, 0, 1, 0],
    #    [1, 0, 1, 1, 0],
    #    [1, 0, 0, 0, 0],
    #    [0, 0, 1, 1, 1]], dtype=np.int32)
    #maze = np.loadtxt('mp1-2021-input.txt', dtype=np.int32)

    START = (0, 0)
    END = (4, 2)
    maze = np.array([
           [0, 1, 0, 0, 0],
           [0, 1, 0, 1, 1],
           [0, 0, 0, 0, 0],
           [1, 0, 1, 1, 0],
           [0, 0, 0, 1, 0]], dtype=np.int32)
    maze = np.loadtxt('test.txt', dtype=np.int32)

    maze[END] = EXIT

    def __init__(self, conf=START, g=0, pred_state=None, pred_action=None):
        """ Initializes the state with information passed from the arguments """
        self.pos = conf  # Configuration of the state - current coordinates
        self.gcost = g  # Path cost
        self.pred = pred_state  # Predecessor state
        self.action_from_pred = pred_action  # Action from Predecessor state to current state

    def __hash__(self):
        """ Returns a hash code so that it can be stored in a set data structure """
        return self.pos.__hash__()

    def is_goal(self):
        """ Returns true if current position is same as the exit position """
        return self.maze[self.pos] == MazeState.EXIT

    def __eq__(self, other):
        """ Checks for equality of states by positions only """
        return self.pos == other

    # EDIT THIS TO CREATE ASTAR
    def __lt__(self, other):
        """ Allows for ordering the states by the path (g) cost """
        return self.gcost < other.gcost

    def __str__(self):
        """ Returns the maze representation of the state """
        # 'str' was deprecated in NumPY 1.20, added '_' to keep in code as I am using 3.8 - JS
        a = np.array(self.maze)
        a[self.pos] = 4
        return np.str_(a)

    move_num = 0  # Used by show_path() to count moves in the solution path

    def show_path(self):
        """ Recursively outputs the list of moves and states along path """
        if self.pred is not None:
            self.pred.show_path()

        if MazeState.move_num == 0:
            print('START')
        else:
            print('Move', MazeState.move_num, 'ACTION:', self.action_from_pred)
        MazeState.move_num = MazeState.move_num + 1
        print(self)

    def can_move(self, move):
        """ Returns true if agent can move in the given direction """
        # if move == 'up':
        #    new_pos = (self.pos[0] - 1, self.pos[1])
        # elif move == 'down':
        #    new_pos = (self.pos[0] + 1, self.pos[1])
        # elif move == 'left':
        #    new_pos = (self.pos[0], self.pos[1] - 1)
        # elif move == 'right':
        #    new_pos = (self.pos[0], self.pos[1] + 1)
        # else:
        # If this breaks, remove 'Exception' - JS
        #    raise Exception('wrong direction for checking move')

        # NEED TO EDIT THESE TO ALLOW WRAP AROUNDS (POSSIBLE MOVES)
        if move == 'up' and self.pos[0] or self.pos[1] == 0:
            new_pos = (self.pos[0] + 9, self.pos[1])
        elif move == 'up' and self.pos[0] < 1:
            new_pos = (self.pos[0] - 1, self.pos[1])
        elif move == 'down':
            new_pos = (self.pos[0] + 1, self.pos[1])
        elif move == 'left' and self.pos[0] == 0:
            new_pos = (self.pos[0], self.pos[1] + 4)
        elif move == 'left' and self.pos[0] != 0:
            new_pos = (self.pos[0], self.pos[1] - 1)
        elif move == 'right':
            new_pos = (self.pos[0], self.pos[1] + 1)
        else:
            # If this breaks, remove 'Exception' - JS
            raise Exception('wrong direction for checking move')

        # NEED TO EDIT THIS TO ALLOW TO MOVE OUTSIDE MAZE
        if new_pos[0] < 0 or new_pos[0] >= self.maze.shape[0] or new_pos[1] < 0 or new_pos[1] >= self.maze.shape[1]:
            return False
        else:
            return self.maze[new_pos] == MazeState.SPACE or self.maze[new_pos] == MazeState.EXIT

    def gen_next_state(self, move):
        """ Generates a new MazeState object by taking move from current state """
        if move == 'up':
            new_pos = (self.pos[0] - 1, self.pos[1])
        elif move == 'down':
            new_pos = (self.pos[0] + 1, self.pos[1])
        elif move == 'left':
            new_pos = (self.pos[0], self.pos[1] - 1)
        elif move == 'right':
            new_pos = (self.pos[0], self.pos[1] + 1)
        else:
            # If this breaks, remove 'Exception' - JS
            raise Exception('wrong direction for next move')

        return MazeState(new_pos, self.gcost + 1, self, move)


# Display the heading info
print('Artificial Intelligence')
print('MP1: A* search algorithm implementation for a maze')
print('SEMESTER: Spring 2021')
print('NAME: Jaylen Scott')
print()

# THIS IS WHERE THE SEARCH ACTUALLY GETS SET UP

# load start state onto frontier priority queue
frontier = queue.PriorityQueue()
start_state = MazeState()
frontier.put(start_state)

# Keep a closed set of states to which optimal path was already found
closed_set = set()

num_states = 0
while not frontier.empty():
    # Choose state at front of priority queue
    next_state = frontier.get()
    num_states = num_states + 1

    # If goal then quit and return path
    if next_state.is_goal():
        next_state.show_path()
        break

    # Add state chosen for expansion to closed_set
    closed_set.add(next_state)

    # Expand state (up to 4 moves possible)
    possible_moves = ['down', 'right', 'left', 'up']
    for move in possible_moves:
        if next_state.can_move(move):
            neighbor = next_state.gen_next_state(move)
            if neighbor in closed_set:
                continue
            if neighbor not in frontier.queue:
                frontier.put(neighbor)
            # If it's already in the frontier, it's guaranteed to have
            # lower cost since all moves have same cost, so no need to update

print('\nNumber of states visited =', num_states)
