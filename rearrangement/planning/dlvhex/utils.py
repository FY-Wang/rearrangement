#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Utilities

    Utility functions for DLVHEX.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import ast
import json

import pandas as pd

from rearrangement import TEMP_PATH


def read_settings():
    settings_json = json.load(open(TEMP_PATH + "/settings.json"))

    verbose, collision_threshold = (
        settings_json["verbose"],
        settings_json["collision_threshold"],
    )

    return verbose, collision_threshold


def get_task_plan(dlvhex_input):
    """Returns the discretized plan as a list of states."""

    # Make format human-readable through magic of list comprehension in Python
    out = [list(s[:-1].replace("(", ",").split(",")) for s in dlvhex_input]
    out = [int(y) for x in out for y in x if y != "location"]
    out = [out[i : i + 3] for i in range(0, len(out), 3)]
    out = pd.DataFrame(data=out, columns=["plannerID", "cell", "time"])

    # Return states grouped by time step
    return [out[out["time"] == x] for x in range(0, max(out["time"] + 1))]


def get_dataframes(plan):
    """Returns two DataFrames:
    1) the continuous configuration (initial and final configurations only), and
    2) the discretization of the continuous configuration - cell boundaries"""

    def integrate_states(states, cont_config_df):
        """Returns the body info with the ID's given by the physics engine and
        the hybrid planner integrated."""

        # Initialize the Planner ID to the dataframe
        cont_config_df["plannerID"] = None

        # Assign planner ID's to UUID's according to init cell
        init_state = states[0]
        for body_index, body in cont_config_df.iterrows():
            for _, obj in init_state.iterrows():
                if body["initCell"] == obj["cell"]:
                    cont_config_df.at[body_index, "plannerID"] = obj["plannerID"]

        # Initialize pose and cell information to the dataframe
        for i in range(len(states) - 2):
            cont_config_df["step{}Cell".format(i + 1)] = None
            cont_config_df["step{}Pose".format(i + 1)] = None

        # Assign cells occupied by each body in intermediate step
        for body_index, body in cont_config_df.iterrows():
            planner_id = body["plannerID"]
            for state in states[1:-1]:
                obj = state[state["plannerID"] == planner_id].squeeze()
                cell_no = obj["cell"]
                column_name = "step{}Cell".format(state["time"].values[0])
                cont_config_df.at[body_index, column_name] = cell_no
        # cont_config_df.to_csv("checkthis.csv")

        return cont_config_df

    cont_config_csv_path = TEMP_PATH + "/config_info.csv"
    disc_config_csv_path = TEMP_PATH + "/discrete_info.csv"

    # Read and patch continuous configuration information
    cont_config_df = pd.read_csv(
        cont_config_csv_path,
        converters={"initPose": ast.literal_eval, "goalPose": ast.literal_eval},
    )
    cont_config_df = integrate_states(plan, cont_config_df)

    # Read discretization of continuous configuration information
    disc_config_df = pd.read_csv(
        disc_config_csv_path, converters={"coordinates": ast.literal_eval}
    )

    return cont_config_df, disc_config_df
