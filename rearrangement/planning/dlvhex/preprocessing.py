#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Preprocess

    Functions to preprocess configuration information for hybrid planning.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import os

import pandas as pd

from rearrangement import TEMP_PATH


def preprocess(continuous_configuration, discretized_configuration):
    """
    Parses and saves info from configs needed by external atoms as CSV files.

    PARAMETERS
    ----------
    continuous_configuration : Configuration
        The configuration, containing initial and goal poses of all bodies.
    discretized_configuration : Discrete
        The discretized representation of the configuration

    RETURNS
    -------
    initial_info : DataFrame
        A dataframe containing all the information about the initial config.
    goal_info : DataFrame
        A dataframe containing all the information about the goal config.
    pandized_grid : DataFrame
        The discrete representation of the suface as a DataFrame.

    """

    # Get centroids, lines, and borders from discrete configuration
    cents = discretized_configuration.centroids
    lines = discretized_configuration.optimal_lines

    # Get grid from lines and borders
    grid = get_grid(lines)

    # Get initial for originals and obstacles from centroids
    initial_org = cents[cents["category"] == "org"]
    initial_obs = cents[cents["category"] == "obs"]
    initial_info = pd.concat([initial_obs, initial_org], ignore_index=True)
    initial_info = initial_info[initial_info["stage"] == "init"]
    initial_info = localize(initial_info, grid)

    # Get goal poses for originals, obstacles, and news from centroids
    goal_org = get_goals(cents, "org")
    goal_obs = get_goals(cents, "obs")
    new_info = cents[cents["category"] == "new"]
    goal_info = pd.concat([goal_obs, goal_org, new_info], ignore_index=True)
    goal_info = localize(goal_info, grid)

    # Store configuration info, including initial and goal poses and cells
    config_info = get_config_info(continuous_configuration, initial_info, goal_info)
    config_info_path = os.path.abspath(TEMP_PATH + "/config_info.csv")
    config_info.to_csv(str(config_info_path), index=False)

    # Store discretized surface info
    pandized_grid = pandize_grid(grid)
    discrete_info_path = os.path.abspath(TEMP_PATH + "/discrete_info.csv")
    pandized_grid.to_csv(str(discrete_info_path), index=False)

    return initial_info, goal_info, pandized_grid


def get_grid(optimal_lines):
    """
    Returns a list of tuples, each of which represents a grid cell as
    (left, upper, right, lower). This way the index of the grid cell can be
    used as the identifier in the ASP program.

    PARAMETERS
    ----------
    optimal_lines : dict
        dict of lines created by the sophisticated discretizer.

    RETURNS
    -------
    grid : list
        list of tuples of grid cells in (left, upper, right, lower) format.

    """

    xs = []
    vlines = sorted(optimal_lines["vlines"])
    for i, vline in enumerate(vlines):
        if i != len(vlines) - 1:
            xs.append((vline, vlines[i + 1]))

    ys = []
    hlines = sorted(optimal_lines["hlines"])
    for i, hline in enumerate(hlines):
        if i != len(hlines) - 1:
            ys.append((hline, hlines[i + 1]))

    # Python list comprehension magic
    grid = [
        (left, upper, right, lower) for (left, right) in xs for (lower, upper) in ys
    ]

    return grid


def localize(config_info, grid):
    """
    Adds the cell numbers of the centroids to the dataframe.

    PARAMETERS
    ----------
    config_info : DataFrame
        The poses of all bodies in the configuration as a DataFrame.
    grid : list
        The discrete surface as a list of tuples (cells).

    RETURNS
    -------
    config_info : DataFrame
        The poses and cells of all bodies in the configuration as a DataFrame.

    """

    def inside(pose, cell):
        """Checks if the pose is contained within the cell."""

        if (cell[0] <= pose[0] <= cell[2]) and (cell[3] <= pose[1] <= cell[1]):
            return True
        return False

    cell_nos = []
    for pose in config_info["pose"]:
        for i, cell in enumerate(grid):
            if inside(pose, cell):
                cell_nos.append(i)

    config_info["cell"] = cell_nos

    return config_info


def get_goals(df, category):
    """
    Returns dataframe of goal configuration only. If there are no goal poses for
    a body, it has not moved and a new row containing the original body info is
    included in the goal dataframe.

    PARAMETERS
    ----------
    df : DataFrame
        The queried dataframe.
    category : str
        The queried type of body: 'obs', 'org', or 'new'.

    RETURNS
    -------
    goal_info : DataFrame
        The poses and cells of all bodies in the goal config as a DataFrame.

    """

    goal_info = df[df["category"] == category]

    for index, body in goal_info.iterrows():
        goal_body = df[(df["oid"] == body["oid"]) & (df["stage"] == "goal")]
        if len(goal_body) == 0:
            goal_info.at[index, "stage"] = "goal"

    return goal_info[goal_info["stage"] == "goal"]


def get_config_info(cont_config, initial_info, goal_info):
    """
    Creates a DataFrame describing the initial and goal configurations in their
    continuous and discrete forms.

    PARAMETERS
    ----------
    continuous_configuration : Configuration
        The configuration, containing initial and goal poses of all bodies.
    initial_info : DataFrame
        The poses and cells of all bodies in the initial config as a DataFrame.
    goal_info : DataFrame
        The poses and cells of all bodies in the goal config as a DataFrame.

    Returns
    ------
    comprehensive_config_info : DataFrame
        A DataFrame describing the initial and goal configurations in their
        continuous and discrete forms.

    """

    comprehensive_config_info = []
    columns = [
        "oid",
        "name",
        "path",
        "xPertub",
        "yPertub",
        "initPose",
        "initCell",
        "goalPose",
        "goalCell",
        "category",
    ]

    for _, body_row in initial_info.iterrows():
        oid = body_row["oid"]
        x_pertubation = body_row["xPertub"]
        y_pertubation = body_row["yPertub"]
        init_cell = body_row["cell"]
        goal_cell = goal_info[goal_info["oid"] == oid]["cell"].array[0]
        category = body_row["category"]

        body = cont_config.find_body(oid)
        body_name = body.name
        body_path = body.path
        init_pose = body.init_pose
        goal_pose = body.pose

        comprehensive_config_info.append(
            [
                oid,
                body_name,
                body_path,
                x_pertubation,
                y_pertubation,
                init_pose,
                init_cell,
                goal_pose,
                goal_cell,
                category,
            ]
        )

    return pd.DataFrame(data=comprehensive_config_info, columns=columns)


def pandize_grid(grid):
    """
    Makes human-readable form of grid.

    PARAMETERS
    ----------
    grid : list
        The discrete surface as a list of tuples (cells).

    RETURNS
    -------
    pandized_grid : DataFrame
        The discrete surface as a DataFrame.

    """

    new_grid = [[index, list(cell)] for index, cell in enumerate(grid)]
    pandized_grid = pd.DataFrame(data=new_grid, columns=["cell", "coordinates"])

    return pandized_grid
