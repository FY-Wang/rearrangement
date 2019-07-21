#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    LocalSearch class definition

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import time
import numpy as np

from rearrangement.errors import type_error
from rearrangement.search import Node, Problem, pairwise_comparison, multiple_comparison


class LocalSearch(object):
    """
    Defines a local search.

    Parameters
    ----------
    problem : Problem
        the problem definition.
    start_time : float
        the start time in time.time() format.
    timeout : float
        the maximum amount of time, in seconds, the search is allowed to go on,
        starting from start_time.
    random_restart : bool
        whether or not random restarts should be activated.

    Attributes
    ----------
    problem : Problem
        the problem definition.
    lexi : bool
        whether the objective is to be compared lexicographically.
    maximality : bool
        whether the objective is to maximize the objective function or not.
    start_time : float
        the start time in time.time() format.
    timeout : float
        the maximum amount of time, in seconds, the search is allowed to go on,
        starting from start_time.
    random_restart : bool
        whether or not random restarts should be activated.

    """

    def __init__(
        self, problem, timeout=500.0, random_restart=False, start_time=time.time()
    ):
        self.problem = problem
        self.lexi = problem.lexi
        self.maximality = problem.maximality
        self.start_time = start_time
        self.timeout = timeout * 60.0
        self.random_restart = random_restart

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
    def lexi(self):
        return self.__lexi

    @lexi.setter
    def lexi(self, lexi):
        if isinstance(lexi, bool):
            self.__lexi = lexi
        else:
            raise type_error("lexi", Problem, type(lexi))

    @property
    def maximality(self):
        return self.__maximality

    @maximality.setter
    def maximality(self, maximality):
        if isinstance(maximality, bool):
            self.__maximality = maximality
        else:
            raise type_error("maximality", Problem, type(maximality))

    @property
    def start_time(self):
        return self.__start_time

    @start_time.setter
    def start_time(self, start_time):
        if isinstance(start_time, float):
            self.__start_time = start_time
        else:
            raise type_error("start_time", float, type(start_time))

    @property
    def timeout(self):
        return self.__timeout

    @timeout.setter
    def timeout(self, timeout):
        if isinstance(timeout, float):
            self.__timeout = timeout
        else:
            raise type_error("timeout", float, type(timeout))

    @property
    def random_restart(self):
        return self.__random_restart

    @random_restart.setter
    def random_restart(self, random_restart):
        if isinstance(random_restart, bool):
            self.__random_restart = random_restart
        else:
            raise type_error("random_restart", Problem, type(random_restart))

    def _min_cond(self, reference, query):
        """Conditional statement for searches that minimize cost."""

        if self.lexi:
            return pairwise_comparison(reference, query, True)

        return query < reference

    def _max_cond(self, reference, query):
        """Conditional statement for searches that maximize score."""

        if self.lexi:
            return pairwise_comparison(reference, query, False)

        return query > reference

    def simple(self, variant="steepest", verbose=False):
        """From the initial state, keep generating successive successor states,
        until no successor state has a smaller cost. There are three implemented
        variants:

        1) 'steepest': the successor state is chosen deterministically from the
        list of all neighbors. This is prone to get stuck in local minima but
        can converge very quickly.

        2) 'stochastic': the successor state is chosen by random from a list of
        neighbors who are all better than the current state. This is more likely
        to avoid converging to local minima but can be slower.

        3) 'Stubborn': the successor state is chosen the same as in the steepest
        variant until timeout """

        def timeout():
            """Returns True if the time limit is exceeded."""

            if time.time() - self.start_time > self.timeout:
                return True
            return False

        if self.maximality:
            cond = self._max_cond
        else:
            cond = self._min_cond

        if verbose:
            iterations = 0

        current = Node(self.problem, self.problem.init_state)
        while not timeout():
            values = []
            better_neighbors = []
            current.expand()
            for i in current.successors:
                if cond(current.value, i.value):
                    better_neighbors.append(i)
                    values.append(i.value)

            if verbose:
                iterations += 1

            value = current.value
            if better_neighbors:
                if variant == "steepest":
                    if self.lexi:
                        successor = better_neighbors[
                            values.index(
                                multiple_comparison(values, not self.maximality)
                            )
                        ]
                    else:
                        op = max if self.maximality else min
                        successor = better_neighbors[values.index(op(values))]

                elif variant == "stochastic":
                    successor = np.random.choice(better_neighbors)

                current = successor

                if self.__problem.goal is not None:
                    if current == self.__problem.goal:
                        break

            # Random restart
            elif (isinstance(value, float) or isinstance(value, int)) and value < 0.01:
                break
            elif isinstance(value, tuple) and value[0] < 1:
                break
            elif self.random_restart:
                current = Node(self.problem, self.problem.get_random_restart())
            else:
                break

        if verbose:
            return current.state, iterations
        return current.state

    # TODO: Simulated Annealing or similar technique that enables 'bad' moves
    # def simulated_annealing(self, schedule):
    #     """From the initial state, keep generating successive successor states,
    #     until no successor state has a smaller cost."""

    #     start_time = time()
    #     time_elapsed = time() - start_time

    #     # Initialize the first node
    #     current = Node(self.problem.initial_state)
    #     current.expand(self.problem)
    #     while current.cost != self.goal_cost and time_elapsed < self.timeout:
    #         temperature = schedule(time_elapsed)
    #         if not current.neighbors or temperature == 0:
    #             break

    #         neighbor = choice(current.neighbors)
    #         neighbor.expand(self.problem)

    #         move_badness = lexicographic_difference(neighbor.cost, current.cost)

    #         # Return the current state if it costs less than the best of its
    #         # neighbors
    #         if lexicographic_comparison(move_badness, self.goal_cost, True):
    #             current = copy(neighbor)
    #         elif exp(move_badness/temperature) > uniform(0, 1):
    #             current = copy(neighbor)

    #         time_elapsed = time() - start_time

    #     return current.state, current.cost, depth
