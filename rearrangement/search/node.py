#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Node class definition

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

from rearrangement.errors import type_error
from rearrangement.search import Problem


class Node(object):
    """
    Defines a node.

    Parameters
    ----------
    problem : Problem
        the problem definition.
    state : Any
        the state.

    Attributes
    ----------
    problem : Problem
        the problem definition.
    state : Any
        the state.
    neighbors : list
        successor states according to the problem transition model.
    value : float
        value of the state.

    """

    def __init__(self, problem, state):
        self.problem = problem
        self.state = state
        self.successors = None
        self.value = problem.get_value(self.state)

    @property
    def problem(self):
        return self.__problem

    @problem.setter
    def problem(self, problem):
        if isinstance(problem, Problem):
            self.__problem = problem
        else:
            raise type_error("problem", Problem, type(problem))

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, state):
        self.__state = state

    @property
    def successors(self):
        return self.__successors

    @successors.setter
    def successors(self, successors):
        self.__successors = successors

    @property
    def value(self):
        return self.__value

    @value.setter
    def value(self, value):
        self.__value = value

    def expand(self):
        successor_states = self.problem.get_successors(self.state)
        self.successors = [Node(self.problem, x) for x in successor_states]
