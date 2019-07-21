#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Baseline class definitions.

    Baselines for comparison with the placement generation algorithms are:

    -   'Random' is a local search whose transition model is to randomly
        generate configurations for all objects.
    
    -   'RandomPotentialField' is a local search whose transition model is to
        randomly generate configurations for all objects, then call on 'Inner'.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import copy
import time

import numpy as np

from rearrangement.errors import type_error
from rearrangement.physics import Engine
from rearrangement.placement import Inner
from rearrangement.search import LocalSearch, Problem


class Random(Problem):
    """This is the definition of the random baseline."""

    def __init__(self, init_state, engine, start_time=time.time()):
        self.start_time = start_time
        self.engine = engine
        self.engine.configuration = init_state

        super(Random, self).__init__(init_state, maximality=False, lexi=False)

    @property
    def engine(self):
        return self.__engine

    @engine.setter
    def engine(self, engine):
        if isinstance(engine, Engine):
            self.__engine = engine
        else:
            raise type_error("engine", Engine, type(engine))

    @property
    def start_time(self):
        return self.__start_time

    @start_time.setter
    def start_time(self, start_time):
        if isinstance(start_time, float):
            self.__start_time = start_time
        else:
            raise type_error("start_time", float, type(start_time))

    def get_cost(self, state):
        """The cost of a state depends on how many clutter bodies have moved and
        by how much, and whether the state is in collision or not."""

        return round(self.engine.get_collision_info(state)["severity"], 2)

    def get_random_restart(self):
        state = copy.deepcopy(self.init_state)
        s_aabb = state.surface.aabb_info
        anti_padding = s_aabb["2D diagonal length"] * 0.0125
        s_x_min = s_aabb["min x"] + anti_padding
        s_y_min = s_aabb["min y"] + anti_padding
        s_x_max = s_aabb["max x"] - anti_padding
        s_y_max = s_aabb["max x"] - anti_padding

        for body in state.movable:
            pose_x = np.random.uniform(s_x_min, s_x_max)
            pose_y = np.random.uniform(s_y_min, s_y_max)
            pose_theta = np.random.uniform(0, 2 * np.pi)
            body.pose = [pose_x, pose_y, pose_theta]

        return state

    @staticmethod
    def get_neighbors(state):
        """Returns the list of states that are successors."""

        s_aabb = state.surface.aabb_info
        anti_padding = s_aabb["2D diagonal length"] * 0.0125
        s_x_min = s_aabb["min x"] + anti_padding
        s_y_min = s_aabb["min y"] + anti_padding
        s_x_max = s_aabb["max x"] - anti_padding
        s_y_max = s_aabb["max x"] - anti_padding

        state = copy.deepcopy(state)
        for body in state.movable:
            pose_x = np.random.uniform(s_x_min, s_x_max)
            pose_y = np.random.uniform(s_y_min, s_y_max)
            pose_theta = np.random.uniform(0, 2 * np.pi)
            body.pose = [pose_x, pose_y, pose_theta]

        return [state]


class RandomPotentialField(Problem):
    """This is the definition of the random with potential field baseline."""

    def __init__(self, init_state, engine, batch_size=10, start_time=time.time()):
        self.batch_size = batch_size
        self.engine = engine
        self.start_time = start_time

        init_state = LocalSearch(Inner(init_state, engine), timeout=1).simple()

        super(RandomPotentialField, self).__init__(
            init_state, maximality=False, lexi=False
        )

    @property
    def engine(self):
        return self.__engine

    @engine.setter
    def engine(self, engine):
        if isinstance(engine, Engine):
            self.__engine = engine
        else:
            raise type_error("engine", Engine, type(engine))

    @property
    def start_time(self):
        return self.__start_time

    @start_time.setter
    def start_time(self, start_time):
        if isinstance(start_time, float):
            self.__start_time = start_time
        else:
            raise type_error("start_time", float, type(start_time))

    def get_neighbors(self, state):
        """Returns the list of states that are successors."""

        s_aabb = state.surface.aabb_info
        anti_padding = s_aabb["2D diagonal length"] * 0.0125
        s_x_min = s_aabb["min x"] + anti_padding
        s_y_min = s_aabb["min y"] + anti_padding
        s_x_max = s_aabb["max x"] - anti_padding
        s_y_max = s_aabb["max x"] - anti_padding

        state = copy.deepcopy(state)
        for body in state.get_movable_bodies():
            pose_x = np.random.uniform(s_x_min, s_x_max)
            pose_y = np.random.uniform(s_y_min, s_y_max)
            pose_theta = np.random.uniform(0, 2 * np.pi)
            body.pose = [pose_x, pose_y, pose_theta]

        return [
            LocalSearch(
                Inner(state, self.engine, start_time=self.start_time),
                start_time=self.start_time,
            ).simple()
        ]

    def get_cost(self, state):
        """The cost of a state depends on how many clutter bodies have moved and
        by how much, and whether the state is in collision or not."""

        return round(self.engine.get_collision_info(state)["severity"], 2)

    def get_random_restart(self):
        state = copy.deepcopy(self.init_state)

        s_aabb = state.surface.aabb_info
        anti_padding = s_aabb["2D diagonal length"] * 0.0125
        s_x_min = s_aabb["min x"] + anti_padding
        s_y_min = s_aabb["min y"] + anti_padding
        s_x_max = s_aabb["max x"] - anti_padding
        s_y_max = s_aabb["max x"] - anti_padding

        for body in state.movable:
            pose_x = np.random.uniform(s_x_min, s_x_max)
            pose_y = np.random.uniform(s_y_min, s_y_max)
            pose_theta = np.random.uniform(0, 2 * np.pi)
            body.pose = [pose_x, pose_y, pose_theta]

        return state
