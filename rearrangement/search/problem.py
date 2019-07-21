#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Problem class definition

    A problem is the definition of a local search's properties, such as its
    initial state, objective function, and transition model.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

from rearrangement.errors import type_error


class Problem(object):
    """
    Defines a problem.
    
    Parameters
    ----------
    init_state : Any
        the initial state from which the problem starts.
    maximality : bool
        whether the objective is to maximize the objective function or not.
    lexi : bool
        whether the objective is to be compared lexicographically.
    goal : Any
        the goal state where the problem terminates (optional)

    Attributes
    ----------
    init_state : Any
        the initial state from which the problem starts.
    maximality : bool
        whether the objective is to maximize the objective function or not.
    lexi : bool
        whether the objective is to be compared lexicographically.
    goal : Any
        the goal state where the problem terminates (optional)
    
    """

    def __init__(self, init_state, maximality, lexi, goal=None):
        self.init_state = init_state
        self.maximality = maximality
        self.goal = goal
        self.lexi = lexi

    @property
    def init_state(self):
        return self.__init_state

    @init_state.setter
    def init_state(self, state):
        self.__init_state = state

    @property
    def maximality(self):
        return self.__maximality

    @maximality.setter
    def maximality(self, maximality):
        self.__maximality = maximality

    @property
    def lexi(self):
        return self.__lexi

    @lexi.setter
    def lexi(self, lexi):
        if isinstance(lexi, bool):
            self.__lexi = lexi
        else:
            raise type_error("lexi", Problem, type(lexi))

    @property
    def maximimality(self):
        return self.__maximality

    @maximality.setter
    def maximality(self, maximality):
        if isinstance(maximality, bool):
            self.__maximality = maximality
        else:
            raise type_error("maximality", Problem, type(maximality))

    def set_goal(self, goal):
        self.__goal = goal

    def get_neighbors(self, state):
        """Returns a set of states according to the problem transition model."""

        raise NotImplementedError()

    def get_value(self, state):
        """Returns the value of a state."""

        raise NotImplementedError()

    def get_random_restart(self):
        """Returns a random state."""

        raise NotImplementedError()
