#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Feasible Plan Search State class definition

    Defines the state for the feasible plan search.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import copy
import json

import pandas as pd

from rearrangement import DEBUG_PATH, TEMP_PATH
from rearrangement.errors import type_error
from rearrangement.physics import Engine


class Plan(object):
    """
    State for the Collision local search.
    
    PARAMETERS
    ----------
    engine : Engine
        The pyBullet engine to be used.
    configurations : list
        The plan as a list, ordered in plan execution sequence.
    planner_ids : list
        The ID's given by the DLVHEX planner for the bodies.

    ATTRIBUTES
    ----------
    engine : Engine
        The pyBullet engine used.
    configurations : list
        The plan as a list of configurations, in plan execution order.
    planner_ids : DataFrame
        The ID's given by the DLVHEX planner for the bodies in the discrete
        domain linked to their names and oids defined in the continuous domain.
    plan : list
        The plan as list of actions, in plan execution order.

    """

    def __init__(self, configurations, engine, planner_ids):
        self.engine = engine
        self.configurations = configurations
        self.planner_ids = planner_ids
        self.plan = self.calculate_plan()

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
    def configurations(self):
        return self.__configurations

    @configurations.setter
    def configurations(self, configurations):
        if isinstance(configurations, list):
            # for configuration in configurations:
            #     self.engine.push_bodies(configuration)
            self.__configurations = configurations
        else:
            raise type_error("configurations", list, type(configurations))

    @property
    def planner_ids(self):
        return self.__planner_ids

    @planner_ids.setter
    def planner_ids(self, planner_ids):
        self.__planner_ids = planner_ids

    @property
    def plan(self):
        return self.__plan

    @plan.setter
    def plan(self, plan):
        self.__plan = plan

    def calculate_plan(self):
        """
        Calculates the plan (sequence of actions) according to the current list
        of configurations.
        
        RETURNS
        -------
        plan : list
            The plan as list of actions, in plan execution order.
        
        """

        plan = []
        for c_ind, previous_configuration in enumerate(self.configurations[:-1]):
            next_configuration = self.configurations[c_ind + 1]
            movable = next_configuration.movable
            for next_body in movable:
                previous_body = previous_configuration.find_body(next_body.oid)
                different_pose = next_body.pose != previous_body.pose
                if different_pose:
                    planner_id = self.planner_ids[
                        self.planner_ids["name"] == next_body.name
                    ].squeeze()["plannerID"]
                    from_pose = previous_body.pose
                    to_pose = next_body.pose
                    plan.append([next_body.name, planner_id, from_pose, to_pose])
                    break
        return plan

    def save_plan(self, filename="plan", csv=True, screenshots=True):
        """
        Saves the plan as a CSV and screenshots of every step.
        
        PARAMETERS
        ----------
        filename : str
            Name of the files to use for saving CSV and screenshots
        csv : bool
            Whether or not to save the plan as a CSV.
        screenshots : bool
            Whether or not to save the plan as screenshots.

        """

        if csv:
            columns = ["bodyName", "plannerID", "poseFrom", "poseTo"]
            data = self.calculate_plan()
            dataframe = pd.DataFrame(data=data, columns=columns)
            dataframe.to_csv(TEMP_PATH + "/{}.csv".format(filename), index=False)

        if screenshots:
            for index, configuration in enumerate(self.configurations):
                self.engine.configuration = configuration
                img = self.engine.get_image()
                img.save(DEBUG_PATH + "/{}-step_{}.png".format(filename, index))
