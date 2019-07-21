#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Middle class definition.

    'Middle' is a local search whose transition model is a grid-based heuristic
    that generates intelligent initializations for, and then calls, 'Inner' to
    minimize the number of objects in collision.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import copy
import time
import uuid
import numpy as np

from rearrangement.errors import type_error
from rearrangement.physics import Engine
from rearrangement.search import LocalSearch, Problem
from rearrangement.placement import Inner


class Middle(Problem):
    """This is the definition of the middle search problem."""

    def __init__(self, init_state, engine, seed=1, start_time=time.time()):
        """Constructor/Initializer for the Middle class."""

        self.start_time = start_time
        self.seed = seed
        self.engine = engine

        inner = Inner(init_state, engine, start_time=start_time)
        init_state = LocalSearch(inner, start_time=start_time).simple()

        super(Middle, self).__init__(init_state, maximality=False, lexi=True)

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
    def seed(self):
        return self.__seed

    @seed.setter
    def seed(self, seed):
        if isinstance(seed, int):
            self.__seed = seed
        else:
            raise type_error("start_time", int, type(seed))

    def get_successors(self, state):
        """Get the neighbors."""

        colliding = self.engine.get_collision_info(state)["list"]
        colliding_movable = [x for x in colliding if x in state.movable]

        neighbors = set()
        for body in colliding_movable:
            buuid = body.oid
            free_cells = get_free_cells(self.seed, state, body)
            neighbors = set()
            for free_cell in free_cells:
                self.engine.configuration = state
                successor = copy.deepcopy(state)
                pose = create_pose(free_cell, "center")
                successor.find_body(buuid).pose = pose
                inner = Inner(successor, self.engine)
                local_search = LocalSearch(inner, start_time=self.__start_time)
                successor = local_search.simple()
                neighbors.add(successor)

        return neighbors

    def get_value(self, state):
        """The cost of a state depends on how many bodies are in collision and
        how severe all the collisions are."""

        col_info = self.engine.get_collision_info(state)

        return (col_info["number"], round(col_info["severity"], 2))

    def get_random_restart(self):
        """Returns a random initialization."""

        state = copy.deepcopy(self.init_state)
        s_aabb = state.surface.aabb_info
        anti_padding = s_aabb["2D diagonal length"] * 0.0125
        x_1 = s_aabb["min x"] + anti_padding
        y_1 = s_aabb["min y"] + anti_padding
        x_2 = s_aabb["max x"] - anti_padding
        y_2 = s_aabb["max y"] - anti_padding

        for body in state.movable:
            pose_x = np.random.uniform(x_1, x_2)
            pose_y = np.random.uniform(y_1, y_2)
            pose_theta = np.random.uniform(0, 2 * np.pi)
            body.pose = [pose_x, pose_y, pose_theta]

        return state


def create_pose(cell, method="normal"):
    """Returns a pose given a cell."""

    if method == "normal":
        mean_x = cell[0][0] + ((cell[1][0] - cell[0][0]) / 2)
        mean_y = cell[0][1] + ((cell[1][1] - cell[0][1]) / 2)
        mean_theta = np.pi
        means = [mean_x, mean_y, mean_theta]

        std_x = ((cell[1][0] - cell[0][0]) / 2) / 3
        std_y = ((cell[1][1] - cell[0][1]) / 2) / 3
        std_theta = np.pi / 2
        stds = [std_x, std_y, std_theta]

        pose_x, pose_y, pose_theta = np.random.multivariate_normal(means, np.diag(stds))

    elif method == "uniform":
        pose_x = np.random.uniform(cell[0][0], cell[1][0])
        pose_y = np.random.uniform(cell[0][1], cell[1][1])
        pose_theta = np.random.uniform(0, 2 * np.pi)

    elif method == "center":
        pose_x = cell[0][0] + ((cell[1][0] - cell[0][0]) / 2)
        pose_y = cell[0][1] + ((cell[1][1] - cell[0][1]) / 2)
        pose_theta = np.pi

    return [pose_x, pose_y, pose_theta]


def empty_cell(cell, centroids):
    """Checks if the cell contains any of the centroids. """

    for j in centroids:
        # center of mass in x
        cond_1 = cell[0][0] <= j[0] <= cell[1][0]
        # center of mass in y
        cond_2 = cell[0][1] <= j[1] <= cell[1][1]

        # if COM is in both x and y ranges of the cell, the cell is not free
        if cond_1 and cond_2:
            return False
    return True


def generate_cells(seed, x_size, y_size):
    """Generates a list of cells using a seed."""

    no_of_cells = (seed * 2) ** 2
    x_cell_len = x_size / (2 * np.sqrt(no_of_cells))
    y_cell_len = y_size / (2 * np.sqrt(no_of_cells))
    sqrt_cell_2 = np.sqrt(no_of_cells) / 2
    my_range = range(-int(sqrt_cell_2), int(sqrt_cell_2))

    cells = []
    for i in my_range:
        for j in my_range:
            cells.append(
                [
                    (i * x_cell_len * 2, j * y_cell_len * 2),
                    ((i + 1) * x_cell_len * 2, (j + 1) * y_cell_len * 2),
                ]
            )

    return cells


def get_free_cells(seed, state, body):
    """Returns a list of free cells"""

    s_aabb = state.surface.aabb_info
    anti_padding = s_aabb["2D diagonal length"] * 0.0125
    x_1 = s_aabb["min x"] + anti_padding
    y_1 = s_aabb["min y"] + anti_padding
    x_2 = s_aabb["max x"] - anti_padding
    y_2 = s_aabb["max y"] - anti_padding

    x_size = x_2 - x_1
    y_size = y_2 - y_1

    centroids = [x.pose[:-1] for x in state.collidable]
    free_cells = []

    while not free_cells:
        cells = generate_cells(seed, x_size, y_size)
        for cell in cells:
            if empty_cell(cell, centroids):
                free_cells.append(cell)

        seed += 1

    return free_cells
