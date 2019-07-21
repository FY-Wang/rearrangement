#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Runner

    Functions to safely handle running DLVHEX.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import ast
import json
import os
import subprocess
import sys

import pandas as pd

from rearrangement import TEMP_PATH


def run_dlvhex_planner(
    collision_threshold,
    initial_info,
    goal_info,
    pandized_grid,
    min_steps,
    name,
    verbose,
):
    """
    Incremental planning with DLVHEX

    PARAMETERS
    ----------
    collision_threshold : float
        The minimium penetration depth between two bodies for a collision.
    initial_info : DataFrame
        The poses and cells of all bodies in the initial config as a DataFrame.
    goal_info : DataFrame
        The poses and cells of all bodies in the goal config as a DataFrame.
    pandized_grid : DataFrame
        The discrete surface as a DataFrame of lists.
    name : str
        Base filename to use for debug files.
    verbose : bool
        Verbosity.

    RETURNS
    -------
    success : bool
        Whether or not a plan was found.

    """

    def create_input_file(initial, goal, grid, time_steps, input_path):
        """Creates an input file for the hybrid planner."""

        with open(str(input_path), "w") as f:
            f.write("% INPUT:\n\n% 1. Max. time step of the solution:\n")
            f.write("const(time_step,{}).\n".format(time_steps))
            for i in range(time_steps + 1):
                f.write("time({}). ".format(i))
            f.write("\n\n")

            f.write("% 2. Bodies (enumerated):\n")
            for _, body in initial.iterrows():
                body_index = goal[goal["oid"] == body["oid"]].index.values.astype(int)[
                    0
                ]
                f.write("body({}). ".format(body_index))
                if body["category"] == "obs":
                    f.write("obs({}). ".format(body_index))
            f.write("\n\n")

            f.write("% 3. Optimal Grid (grid cells are enumerated):\n")
            for i in range(len(grid)):
                f.write("cell({}). ".format(i))
            f.write("\n\n")

            f.write("% 4. Initial configuration:\n")
            for _, body in initial.iterrows():
                body_index = goal[goal["oid"] == body["oid"]].index.values.astype(int)[
                    0
                ]
                cell_index = body["cell"]
                f.write(":- not location({},{},0).\n".format(body_index, cell_index))
            f.write("\n")

            f.write("% 5. Goal configuration:\n")
            for _, body in goal.iterrows():
                body_index = goal[goal["oid"] == body["oid"]].index.values.astype(int)[
                    0
                ]
                cell_index = goal[goal["oid"] == body["oid"]]["cell"].values.astype(
                    int
                )[0]
                f.write(
                    ":- not location({},{},{}).\n".format(
                        body_index, cell_index, time_steps
                    )
                )

    def create_bash_exec(dlvhex_path, input_path, domain_path, plugin_path, exec_path):
        """Return the path of the newly created bash exec for the planning step."""

        with open(exec_path, "w") as f:
            bin_path = os.path.abspath(os.path.dirname(sys.executable))
            plugin_cmd = "--python-plugin=" + str(plugin_path)
            number_cmd = "-n 1"
            filter_cmd = "-f moveTo"
            verbose_cmd = "--verbose=0"

            f.write('PATH="{}:$PATH"; export PATH\n'.format(bin_path))
            buff = " ".join(
                [
                    str(dlvhex_path),
                    str(input_path),
                    str(domain_path),
                    plugin_cmd,
                    number_cmd,
                    filter_cmd,
                    verbose_cmd,
                ]
            )
            f.write(buff)

    success = False
    dlvhex_path = "dlvhex"
    dir_path = os.path.abspath(os.path.dirname(__file__))
    domain_path = os.path.abspath(dir_path + "/hybrid_planner.lp")
    plugin_path = os.path.abspath(dir_path + "/external_atom.py")
    input_path = os.path.abspath(TEMP_PATH + "/plan_input.lp")
    exec_path = os.path.abspath(TEMP_PATH + "/exec.sh")
    settings_path = os.path.abspath(TEMP_PATH + "/settings.json")

    with open(settings_path, "w") as settings:
        json.dump(
            {
                "collision_threshold": collision_threshold,
                "name": name,
                "verbose": verbose,
            },
            settings,
        )

    time_steps = min_steps
    while not success:
        create_input_file(
            initial_info, goal_info, pandized_grid, time_steps, input_path
        )

        # Very weird behaviour when using subprocess with dlvhex, use bash.
        create_bash_exec(dlvhex_path, input_path, domain_path, plugin_path, exec_path)

        process = subprocess.Popen(
            ["bash", str(exec_path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        ans, _ = process.communicate()

        if not isinstance(ans, str):
            ans = ans.decode("utf-8")
        time_steps += 1
        if "{" in ans:
            success = True

    if not verbose:
        os.remove(input_path)
        os.remove(exec_path)
        os.remove(settings_path)

    return success
