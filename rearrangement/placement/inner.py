#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Inner class definition.

    'Inner' is a local search whose transition model is a wrapper of the
    collision resolver of pybullet that makes it act as a potential field to
    minimize the total penetration depth of all pairs of objects.

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
from rearrangement.search import Problem


class Inner(Problem):
    """This is the definition of the inner search problem."""

    def __init__(self, init_state, engine, batch_size=10, start_time=time.time()):
        self.start_time = start_time
        self.batch_size = batch_size
        self.engine = engine
        self.engine.configuration = init_state

        super(Inner, self).__init__(init_state, maximality=False, lexi=False)

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

    @property
    def batch_size(self):
        return self.__batch_size

    @batch_size.setter
    def batch_size(self, batch_size):
        if isinstance(batch_size, int):
            self.__batch_size = batch_size
        else:
            raise type_error("batch_size", int, type(batch_size))

    def get_successors(self, state):
        """Returns the list of states that are successors. Here, this is the
        result of the physics engine pushing objects in collision outside of
        collision, attempting to reduce the cumulative penetration depth."""

        new_state = copy.deepcopy(state)
        self.engine.push_bodies(new_state, self.batch_size)

        return [new_state]

    def get_value(self, state):
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
