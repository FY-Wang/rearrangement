#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Discrete Placement

    The Dicrete Placement problem is that of finding the minimum number of
    non-uniform grid cells, such that each cell contains only one centroid of
    all the objects in the initial and tentative goal configurations.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import itertools
import os
import subprocess

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from rearrangement import TEMP_PATH
from rearrangement.errors import type_error
from rearrangement.physics import Configuration, Engine


def get_col(array, column_index):
    """
    Returns the column of a 2D array.
    
    Parameters
    ----------
    array : list
        2D list (list of lists).
    column_index : int
        Index of queried column.

    Returns
    -------
    column : list
        Queried column.

    """

    column = [row[column_index] for row in array]

    return column


class Discrete(object):
    """
    Defines a discrete representation of a configuration.

    Parameters
    ----------
    continuous_configuration : Configuration
        The configuration, containing initial and goal poses of all bodies.
    new : bool
        Whether or not new objects need to be planned for.

    Attributes
    ----------
    continuous_configuration : Configuration
        The configuration, containing initial and goal poses of all bodies.
    centroids : list
        list of (x,y) tuples representing the location of all body centroids.
    borders : list
        list of the outermost lines, each is represented by two coordinates.
    suboptimal_lines : dict
        dict of lines created by the naive discretizer.
    optimal_lines : dict
        dict of lines created by the sophisticated discretizer.
    """

    def __init__(self, continuous_configuration, new=True):
        self.continuous_configuration = continuous_configuration
        self.centroids = self._extract_centroids(new=new)
        self.borders = self._extract_borders()
        self.suboptimal_lines = self._suboptimal_discretizer()
        self.optimal_lines = self._optimal_discretizer()

    @property
    def continuous_configuration(self):
        return self.__continuous_configuration

    @continuous_configuration.setter
    def continuous_configuration(self, continuous_configuration):
        if isinstance(continuous_configuration, Configuration):
            self.__continuous_configuration = continuous_configuration
        else:
            raise type_error(
                "continuous_configuration",
                Configuration,
                type(continuous_configuration),
            )

    @property
    def centroids(self):
        return self.__centroids

    @centroids.setter
    def centroids(self, centroids):
        self.__centroids = centroids

    @property
    def borders(self):
        return self.__borders

    @borders.setter
    def borders(self, borders):
        self.__borders = borders

    @property
    def suboptimal_lines(self):
        return self.__suboptimal_lines

    @suboptimal_lines.setter
    def suboptimal_lines(self, suboptimal_lines):
        self.__suboptimal_lines = suboptimal_lines

    @property
    def optimal_lines(self):
        return self.__optimal_lines

    @optimal_lines.setter
    def optimal_lines(self, optimal_lines):
        self.__optimal_lines = optimal_lines

    def _extract_borders(self):
        """
        Extracts the borders of the surface using its AABB.

        Returns
        -------
        borders : list
            list of the outermost lines, each is represented by two coordinates.
    
        """

        surf_aabb_info = self.continuous_configuration.surface.aabb_info
        x_min = surf_aabb_info["min x"]
        x_max = surf_aabb_info["max x"]
        y_min = surf_aabb_info["min y"]
        y_max = surf_aabb_info["max y"]
        borders = [
            [[x_min, x_max], [y_min, y_min]],
            [[x_min, x_max], [y_max, y_max]],
            [[x_min, x_min], [y_min, y_max]],
            [[x_max, x_max], [y_min, y_max]],
        ]

        return borders

    def _extract_centroids(self, new):
        """
        Extracts centroids of all bodies in  initial and goal configurations.
        
        Parameters
        ----------
        new : bool
            Whether or not new objects need to be planned for.

        Returns
        -------
        centroids : DataFrame
            (x,y) tuples representing the location of all body centroids.

        """

        acquired_poses = []
        centroids = []

        for body in self.continuous_configuration.obstacles:
            oid = body.oid
            init_pose = body.init_pose[:-1]
            pose = body.pose[:-1]

            x_perturbation = 0
            y_perturbation = 0
            while round(pose[0] + x_perturbation, 3) in get_col(acquired_poses, 0):
                x_perturbation += np.random.uniform(-0.05, 0.05)
            pose[0] = round(pose[0] + x_perturbation, 3)

            while round(pose[1] + y_perturbation, 3) in get_col(acquired_poses, 1):
                y_perturbation += np.random.uniform(-0.05, 0.05)
            pose[1] = round(pose[1] + y_perturbation, 3)

            centroids.append([oid, pose, x_perturbation, y_perturbation, "init", "obs"])
            acquired_poses.append(pose)

        for body in self.continuous_configuration.originals:
            oid = body.oid
            init_pose = body.init_pose[:-1]
            pose = body.pose[:-1]

            x_perturbation = 0
            y_perturbation = 0
            while round(init_pose[0] + x_perturbation, 3) in get_col(acquired_poses, 0):
                x_perturbation += np.random.uniform(-0.05, 0.05)
            init_pose[0] = round(init_pose[0] + x_perturbation, 3)

            while round(init_pose[1] + y_perturbation, 3) in get_col(acquired_poses, 1):
                y_perturbation += np.random.uniform(-0.05, 0.05)
            init_pose[1] = round(init_pose[1] + y_perturbation, 3)

            centroids.append(
                [oid, init_pose, x_perturbation, y_perturbation, "init", "org"]
            )
            acquired_poses.append(init_pose)

            if body.pose[:-1] != body.init_pose[:-1]:
                x_perturbation = 0
                y_perturbation = 0
                while round(pose[0] + x_perturbation, 3) in get_col(acquired_poses, 0):
                    x_perturbation += np.random.uniform(-0.05, 0.05)
                pose[0] = round(pose[0] + x_perturbation, 3)

                while round(pose[1] + y_perturbation, 3) in get_col(acquired_poses, 1):
                    y_perturbation += np.random.uniform(-0.05, 0.05)
                pose[1] = round(pose[1] + y_perturbation, 3)

                centroids.append(
                    [oid, pose, x_perturbation, y_perturbation, "goal", "org"]
                )
                acquired_poses.append(pose)
        if new:
            for body in self.continuous_configuration.news:
                oid = body.oid
                pose = body.pose[:-1]

                x_perturbation = 0
                y_perturbation = 0
                while round(pose[0] + x_perturbation, 3) in get_col(acquired_poses, 0):
                    x_perturbation += np.random.uniform(-0.05, 0.05)
                pose[0] = round(pose[0] + x_perturbation, 3)

                while round(pose[1] + y_perturbation, 3) in get_col(acquired_poses, 1):
                    y_perturbation += np.random.uniform(-0.05, 0.05)
                pose[1] = round(pose[1] + y_perturbation, 3)

                centroids.append(
                    [oid, pose, x_perturbation, y_perturbation, "goal", "new"]
                )
                acquired_poses.append(pose)

        centroids = pd.DataFrame(
            centroids,
            columns=["oid", "pose", "xPertub", "yPertub", "stage", "category"],
        )
        return centroids

    def plot_suboptimal(self, filename="suboptimal.pdf"):
        """
        Saves a plot of the suboptimal discretization of the configuration with
        centroids.

        PARAMETERS
        ----------
        filename : str
            Path to the file to store the plot.

        """

        self._plot(
            self.suboptimal_lines["hlines"],
            self.suboptimal_lines["vlines"],
            color="g",
            linewidth=0.1,
        )
        plt.savefig(filename)

    def plot_optimal(self, filename="optimal.pdf"):
        """
        Saves a plot of the optimal discretization of the configuration with
        centroids.

        PARAMETERS
        ----------
        filename : str
            Path to the file to store the plot.

        """

        self._plot(
            self.optimal_lines["hlines"],
            self.optimal_lines["vlines"],
            color="g",
            linewidth=1,
        )
        plt.savefig(filename)

    def plot_grid(self, filename="grid.pdf"):
        """
        Saves a plot of the optimal discretization of the configuration without
        centroids.

        PARAMETERS
        ----------
        filename : str
            Path to the file to store the plot.

        """

        self._plot(
            self.optimal_lines["hlines"],
            self.optimal_lines["vlines"],
            color="g",
            linewidth=1,
            centroids=False,
        )
        plt.savefig(filename)

    def plot_both(self, filename="both.pdf"):
        """
        Saves a plot of the suboptimal and optimal discretizations of the
        configuration.

        PARAMETERS
        ----------
        filename : str
            Path to the file to store the plot.

        """

        self._plot(
            self.suboptimal_lines["hlines"],
            self.suboptimal_lines["vlines"],
            color="g",
            linewidth=0.1,
        )
        self._plot(
            self.optimal_lines["hlines"],
            self.optimal_lines["vlines"],
            color="g",
            linewidth=1,
            clear=False,
        )
        plt.savefig(filename)

    def _plot(self, hlines, vlines, color, linewidth, clear=True, centroids=True):
        """
        Plots the horizontal and vertical lines, and centroids.
        
        PARAMETERS
        ----------
        hlines : list
            Horizontal lines to plot.
        vlines : list
            Vertical lines to plot.
        color : str
            Color to use for plotting the lines.
        linewidth : float
            Width of lines.
        clear : bool
            Whether or not to clear all previous plots.
        centroids : bool
            Whether or not to plot the centroids.

        """

        borders = self.borders
        x_min = borders[0][0][0]
        x_max = borders[0][0][1]
        y_min = borders[2][1][0]
        y_max = borders[2][1][1]
        if clear:
            plt.close("all")

        if centroids:
            cents = self.centroids
            init_org = cents[(cents["stage"] == "init") & (cents["category"] == "org")][
                "pose"
            ].values.tolist()
            goal_org = cents[(cents["stage"] == "goal") & (cents["category"] == "org")][
                "pose"
            ].values.tolist()
            obs = cents[cents["category"] == "obs"]["pose"].values.tolist()
            new = cents[cents["category"] == "new"]["pose"].values.tolist()
            if init_org:
                plt.scatter(get_col(init_org, 0), get_col(init_org, 1), color="m", s=5)
            if goal_org:
                plt.scatter(get_col(goal_org, 0), get_col(goal_org, 1), color="r", s=5)
            if obs:
                plt.scatter(get_col(obs, 0), get_col(obs, 1), color="k", s=5)
            if new:
                plt.scatter(get_col(new, 0), get_col(new, 1), color="b", s=5)

        for hline in hlines:
            plt.plot([x_min, x_max], [hline, hline], color=color, linewidth=linewidth)
        for vline in vlines:
            plt.plot([vline, vline], [y_min, y_max], color=color, linewidth=linewidth)
        for border in borders:
            plt.plot(border[0], border[1], color="k", linewidth=3)

        plt.xlim(min(x_min - 1, y_min - 1), max(x_max + 1, y_max + 1))
        plt.ylim(min(x_min - 1, y_min - 1), max(x_max + 1, y_max + 1))
        plt.axis("off")

    def _suboptimal_discretizer(self):
        """
        Divides the surface into a non-uniform, possibly non-optimal grid by
        iteratively dividing the midway of the two closest centroids
        horizontally or vertically.
        
        """

        poses = [tuple(x) for x in self.centroids["pose"]]

        x_centroids = sorted(get_col(poses, 0))
        y_centroids = sorted(get_col(poses, 1))
        hlines = set()
        vlines = set()
        for i, x_cent in enumerate(x_centroids):
            if i != len(x_centroids) - 1:
                vlines.add((x_cent + x_centroids[i + 1]) / 2)

        for j, y_cent in enumerate(y_centroids):
            if j != len(y_centroids) - 1:
                hlines.add((y_cent + y_centroids[j + 1]) / 2)

        hlines = sorted(list(hlines))
        vlines = sorted(list(vlines))

        return {"hlines": hlines, "vlines": vlines}

    def _optimal_discretizer(self):
        """
        Divides the surface into the minimum number of non-uniform cells.
        
        """

        cd_path = os.path.dirname(__file__)
        input_path = os.path.abspath(TEMP_PATH + "/naive_discretization.lp")
        optimizer_path = os.path.abspath(cd_path + "/optimal_discretizer.lp")

        x_size = self.borders[0][0][1] - self.borders[0][0][0]
        y_size = self.borders[3][1][1] - self.borders[3][1][0]

        hlines = (
            [0] + [y + (y_size / 2) for y in self.suboptimal_lines["hlines"]] + [y_size]
        )
        vlines = (
            [0] + [x + (x_size / 2) for x in self.suboptimal_lines["vlines"]] + [x_size]
        )
        places = []

        with open(str(input_path), "w") as f:
            f.write("#const maxy={}.\n".format(len(hlines) - 1))
            f.write("#const maxx={}.\n".format(len(vlines) - 1))
            f.write("hline(0..maxy).\n")
            f.write("vline(0..maxx).\n")
            poses = [tuple(x) for x in self.centroids["pose"]]
            for centroid in poses:
                lower = hlines[0]
                for hline in hlines:
                    if lower < hline < centroid[1] + (y_size / 2):
                        lower = hline
                upper = hlines[-1]
                for hline in hlines:
                    if centroid[1] + (y_size / 2) < hline < upper:
                        upper = hline
                left = vlines[0]
                for vline in vlines:
                    if left < vline < centroid[0] + (x_size / 2):
                        left = vline
                right = vlines[-1]
                for vline in vlines:
                    if centroid[0] + (x_size / 2) < vline < right:
                        right = vline
                places.append(
                    (
                        vlines.index(left),
                        hlines.index(upper),
                        vlines.index(right),
                        hlines.index(lower),
                    )
                )
                f.write(
                    "obj({},{},{},{}).\n".format(
                        vlines.index(left),
                        hlines.index(upper),
                        vlines.index(right),
                        hlines.index(lower),
                    )
                )
        process = subprocess.Popen(
            ["clingo", str(input_path), str(optimizer_path), "--verbose=0"],
            stdout=subprocess.PIPE,
        )

        ans, _ = process.communicate()

        os.remove(input_path)

        vlines_d = []
        hlines_d = []
        if not isinstance(ans, str):
            ans = ans.decode("utf-8")
        if "UNSATISFIABLE" not in ans:
            first_index = 0
            counter = 0
            for i, string in enumerate(reversed(ans)):
                if string == "\n":
                    counter += 1
                if counter == 2:
                    last_index = len(ans) - i - 3
                if counter == 4:
                    first_index = len(ans) - i
                    break
            ans = ans[first_index : last_index + 1]
            for part in ans.split():
                if "newhline" in part:
                    index = part[part.find("(") + 1 : part.find(")")]
                    hlines_d.append(hlines[int(index)] - (y_size / 2))
                elif "newvline" in part:
                    index = part[part.find("(") + 1 : part.find(")")]
                    vlines_d.append(vlines[int(index)] - (x_size / 2))

        if not (vlines_d or hlines_d):
            return {"hlines": hlines, "vlines": vlines}

        return {"hlines": hlines_d, "vlines": vlines_d}
