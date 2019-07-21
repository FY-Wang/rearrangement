#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Outer class definition.

    'Outer' is a local search whose transition model is the modification of
    circular constraints to generate intelligent initializations for, and then
    calls, 'Middle' to minimize the number of original objects moved.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import copy
import time

import numpy as np

from rearrangement.errors import type_error
from rearrangement.physics import Constraint, Engine
from rearrangement.search import LocalSearch, Problem
from rearrangement.placement import Middle


class Outer(Problem):
    """This is the definition of the outer search problem."""

    def __init__(self, init_state, engine, start_time=time.time()):
        # For each original body, add a circular constraint of radius zero, and
        # store the relationship between each body and its constraint in a dict

        self.start_time = start_time
        init_state = copy.deepcopy(init_state)
        self.engine = engine
        self.const_dict = {}

        for body in init_state.originals:

            center_x, center_y, angle = body.init_pose
            geometry = {"center": (center_x, center_y), "radius": 0.0}
            const = Constraint("circular", geometry)

            rot_geometry = {"max": angle, "min": angle}
            rot_const = Constraint("rotational", rot_geometry)

            body.add_constraint(const)
            body.add_constraint(rot_const)

            buuid = body.oid
            cuuid = const.oid
            rot_cuuid = rot_const.oid
            self.const_dict.update({"{}".format(buuid): (cuuid, rot_cuuid)})

        middle = Middle(init_state, engine, start_time=start_time)
        init_state = LocalSearch(middle, start_time=start_time).simple()

        self.engine.configuration = init_state

        super(Outer, self).__init__(init_state, maximality=False, lexi=True)

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
    def const_dict(self):
        return self.__const_dict

    @const_dict.setter
    def const_dict(self, const_dict):
        if isinstance(const_dict, dict):
            self.__const_dict = const_dict
        else:
            raise type_error("const_dict", dict, type(const_dict))

    def get_successors(self, state):
        """This is the successor state function of the problem. It returns a set
        of states, with each containing a single cluttered body with an
        augmented freedom."""

        neighbors = set()
        for body in state.originals:
            buuid = body.oid
            cuuid, rot_cuuid = self.const_dict["{}".format(body.oid)]
            if rot_cuuid is not None:
                rot_const = body.find_constraint(rot_cuuid)
                body.remove_constraint(rot_const)
                self.const_dict["{}".format(body.oid)] = (cuuid, None)
            const = body.find_constraint(cuuid)
            geometry = const.geometry

            limit = state.get_max_displacement(body)
            delta = limit / 4

            if geometry["radius"] + delta <= limit:
                geometry["radius"] += delta
                successor = copy.deepcopy(state)
                successor.find_body(buuid).find_constraint(cuuid).geometry = geometry
                successor = LocalSearch(
                    Middle(successor, self.engine, start_time=self.start_time),
                    start_time=self.start_time,
                ).simple()
                neighbors.add(successor)

        return neighbors

    def get_value(self, state):
        """The cost of a state depends on how many bodies are in collision and
        how many clutter bodies have moved and by how much."""

        col_info = self.engine.get_collision_info(state)
        mov_info = state.movement_info

        return (col_info["number"], mov_info["number"], mov_info["severity"])

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

        self.const_dict = {}
        for body in state.originals:
            center_x, center_y, angle = body.init_pose
            geometry = {"center": (center_x, center_y), "radius": 0.0}
            const = Constraint("circular", geometry)
            rot_geometry = {"max": angle, "min": angle}
            rot_const = Constraint("rotational", rot_geometry)
            body.add_constraint(const)
            body.add_constraint(rot_const)

            buuid = body.oid
            cuuid = const.oid
            rot_cuuid = rot_const.oid
            self.const_dict.update({"{}".format(buuid): (cuuid, rot_cuuid)})

        return state
