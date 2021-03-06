#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Engine class definition

    Engine is a wrapper around pybullet for placement generation problems.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import copy
import itertools
import json
import os
import uuid
import warnings

import numpy as np
import pybullet as p
from PIL import Image

from rearrangement.physics import (
    Body,
    Constraint,
    Configuration,
    to_euler,
    to_quaternion,
)
from rearrangement.errors import type_error, value_error, length_error


class Engine(object):
    """
    A wrapper for the Bullet physics engine for rearrangement problems.

    Attributes
    ----------
    collision_threshold : float
        the distance after which a geometry overlap between two bodies is
        considered a collision.
    configuration : Configuration
        the configuration currently loaded into the instance.
    connected : bool
        whether or not the instance is connected to a physics backend.
    eid : int
        the ID of the engine generated by pybullet.
    oid : UUID
        the UUID of the engine; generated by uuid1.
    visual : bool
        whether or not the instance is being visualized.

    """

    def __init__(self):
        self.collision_threshold = 0.001
        self.configuration = None
        self.connected = False
        self.eid = None
        self.oid = uuid.uuid1()
        self.visual = None

    @property
    def collision_threshold(self):
        return self.__collision_threshold

    @collision_threshold.setter
    def collision_threshold(self, col_thresh):
        if isinstance(col_thresh, float):
            self.__collision_threshold = col_thresh
        else:
            raise type_error("collision_threshold", float, type(col_thresh))

    @property
    def configuration(self):
        return self.__configuration

    @configuration.setter
    def configuration(self, configuration):
        if isinstance(configuration, Configuration):
            if self.configuration is not None:
                for body in configuration.movable:
                    # body.set_color([1,0,0,1])
                    loaded_body = copy.deepcopy(self.configuration.find_body(body.oid))
                    loaded_body.constraints = copy.deepcopy(body.constraints)
                    loaded_body.pose = body.pose
            self.__configuration = configuration

        elif configuration is None:
            self.__configuration = configuration

        else:
            raise type_error("configuration", Configuration, type(configuration))

    @property
    def connected(self):
        return self.__connected

    @connected.setter
    def connected(self, connected):
        if isinstance(connected, bool):
            self.__connected = connected
        else:
            raise type_error("connected", bool, type(connected))

    @property
    def eid(self):
        return self.__eid

    @eid.setter
    def eid(self, eid):
        if isinstance(eid, int) or eid is None:
            self.__eid = eid
        else:
            raise type_error("eid", int, type(eid))

    @property
    def oid(self):
        return self.__oid

    @oid.setter
    def oid(self, oid):
        if isinstance(oid, uuid.UUID):
            self.__oid = oid
        else:
            raise type_error("oid", uuid.UUID, type(oid))

    @property
    def visual(self):
        return self.__oid

    @visual.setter
    def visual(self, visual):
        if isinstance(visual, bool) or visual is None:
            self.__visual = visual
        else:
            raise type_error("visual", bool, type(visual))

    def connect(self, visual):
        """Starts the Bullet Physics Engine."""

        if self.connected:
            warnings.warn("Physics engine already connected.")
            return

        if visual:
            self.eid = p.connect(
                p.GUI,
                options="--background_color_red=1 --background_color_green=1 --background_color_blue=1",
            )
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.eid)
            self.set_camera_parameters()
            self.visual = True
        else:
            self.eid = p.connect(p.DIRECT)
            self.visual = False

        self.connected = True

    def disconnect(self):
        """Stops the Bullet Physics Engine."""

        if not self.connected:
            warnings.warn("Physics engine already disconnected.")
            return

        p.disconnect(physicsClientId=self.eid)

        self.connected = False

    def set_parameters(self, *args, **kwargs):
        """Sets several parameters of the physics engine. Refer to the pybullet
        Quickstart Guide for the list of exposed parameters."""

        p.setPhysicsEngineParameter(physicsClientId=self.eid, *args, **kwargs)

    def get_parameters(self):
        """Gets several parameters of the physics engine. Refer to the pybullet
        Quickstart Guide for the list of exposed parameters."""

        return p.getPhysicsEngineParameters(physicsClientId=self.eid)

    def get_image(self, width=1920, height=1920, distance=10, fov=90, new=True):
        """Returns an image of the setup."""

        if not new:
            for body in self.configuration.news:
                color = body.color
                color[-1] = 0.0
                body.color = color

        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            [0, 0, 0], distance, 0, -90, 0, 2
        )

        projection_matrix = p.computeProjectionMatrixFOV(fov, width / height, 0.01, 100)

        image_info = p.getCameraImage(
            width,
            height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            physicsClientId=self.eid,
        )
        image_array = np.uint8(np.reshape(image_info[2], (height, width, 4)))

        image = Image.fromarray(image_array, "RGBA")

        if not new:
            for body in self.configuration.news:
                color = body.color
                color[-1] = 1.0
                body.color = color

        return image

    def set_camera_parameters(self, center=None, distance=6, yaw=0, pitch=-89.99):
        """Sets the GUI visualizer settings. Pitch and yaw are in degrees."""

        if center is None:
            center = [0, 0, 0]
        p.resetDebugVisualizerCamera(
            distance, yaw, pitch, center, physicsClientId=self.eid
        )

    def draw_line(self, point1, point2, color=[1, 1, 1], width=3):
        """Draws a line between two 3D points and returns its ID."""

        return p.addUserDebugLine(
            point1,
            point2,
            lineColorRGB=color,
            lineWidth=width,
            physicsClientId=self.eid,
        )

    def remove_line(self, line_id):
        """Removes a previously drawn line, given the line ID."""

        return p.removeUserDebugItem(line_id, physicsClientId=self.eid)

    def remove_all_lines(self):
        """Removes all lines drawn."""

        return p.removeAllUserDebugItems(physicsClientId=self.eid)

    def load_configuration(self, json_input):
        """Loads the configuration from a JSON object."""

        def load_bodies(dic, surf, fixed=False, color=None):
            """Loads bodies into the physics engine."""

            # Define a constraint to maintain the body to be in the surface
            s_aabb = surf.aabb_info
            anti_padding = s_aabb["2D diagonal length"] * 0.0125
            s_aabb["min x"] = s_aabb["min x"] + anti_padding
            s_aabb["min y"] = s_aabb["min y"] + anti_padding
            s_aabb["max x"] = s_aabb["max x"] - anti_padding
            s_aabb["max y"] = s_aabb["max y"] - anti_padding
            s_const = Constraint("rectangular", s_aabb)

            loaded_bodies = []
            for key, body in dic.items():
                path = os.path.join(home_dir, body["path"])
                z_pos = body["z offset"] + (surf.z_off * 2.0)
                if "pose" in body:
                    pose = [float(x) for x in body["pose"]]
                else:
                    pose = [
                        np.random.uniform(s_aabb["min x"], s_aabb["max x"]),
                        np.random.uniform(s_aabb["min y"], s_aabb["max y"]),
                        np.random.uniform(0, 2 * np.pi),
                    ]
                constraints = []
                constraints.append(s_const)
                if "constraints" in body:
                    for _, constraint in body["constraints"].items():
                        constraints.append(
                            Constraint(constraint["shape"], constraint["geometry"])
                        )

                orn = to_quaternion([0.0, 0.0, float(pose[2])])
                if "real" in path:
                    globalScaling = 15.0
                else:
                    globalScaling = 1.0
                z_pos = z_pos * globalScaling
                pos = [float(pose[0]), float(pose[1]), float(z_pos)]
                bid = p.loadURDF(
                    path,
                    pos,
                    orn,
                    useFixedBase=fixed,
                    physicsClientId=self.eid,
                    globalScaling=globalScaling,
                )
                p.changeVisualShape(bid, -1, rgbaColor=color, physicsClientId=self.eid)
                p.changeVisualShape(bid, 0, rgbaColor=color, physicsClientId=self.eid)
                p.changeVisualShape(bid, 1, rgbaColor=color, physicsClientId=self.eid)
                p.changeVisualShape(bid, 2, rgbaColor=color, physicsClientId=self.eid)

                body = Body(
                    int(bid),
                    color,
                    str(key),
                    pose,
                    constraints,
                    z_pos,
                    float(body["area"]),
                    str(path),
                )
                loaded_bodies.append(body)

            return loaded_bodies

        if not self.connected:
            raise ValueError("Physics engine not connected.")
        if self.configuration is not None:
            # warnings.warn("The current configuration will be replaced.")
            self.destroy_configuration()
            self.configuration = None

        home_dir = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        )
        surf = json_input["surface"]
        obs = json_input["obstacles"]
        original = json_input["originals"]
        new = json_input["news"]

        s_path = os.path.join(home_dir, surf["path"])
        if "real" in s_path:
            globalScaling = 10
        else:
            globalScaling = 1

        surf_id = p.loadURDF(
            s_path,
            useFixedBase=True,
            physicsClientId=self.eid,
            globalScaling=globalScaling,
        )
        surf = Body(
            int(surf_id),
            [0.0, 1.0, 0.0, 1.0],
            "surface",
            [0.0, 0.0, 0.0],
            [],
            float(surf["z offset"]),
            float(surf["area"]),
            str(s_path),
        )
        p.changeVisualShape(
            surf_id, -1, rgbaColor=[0.0, 1.0, 0.0, 1.0], physicsClientId=self.eid
        )

        l_obs = []
        if obs:
            l_obs = load_bodies(obs, surf, fixed=True, color=[0.0, 0.0, 0.0, 1.0])

        l_original = []
        if original:
            l_original = load_bodies(original, surf, color=[1.0, 0.0, 0.0, 1.0])

        l_new = load_bodies(new, surf, color=[0.0, 0.0, 1.0, 1.0])

        self.configuration = Configuration(surf, l_obs, l_original, l_new)
        return self.configuration

    def dump_configuration(self):
        """Dumps the engine's current configuration into a JSON object."""

        if not self.connected:
            raise ValueError("Physics engine not connected.")

        if self.configuration is None:
            raise ValueError("No configuration is in the engine.")

        def unpack(obj):
            """Unpacks an object into a dictionary."""

            dic = {}
            for item in obj:
                key = item.name
                pose = item.pose
                pose[2] = pose[2] - self.configuration.surface.z_off
                value = {"pose": pose}
                dic[key] = value
            return dic

        surface = unpack([self.configuration.surface])
        originals = unpack(self.configuration.originals)
        news = unpack(self.configuration.news)
        obstacles = unpack(self.configuration.obstacles)

        data = {
            "surface": surface,
            "obstacles": obstacles,
            "original": originals,
            "new": news,
        }

        return json.dumps(data)

    def destroy_configuration(self):
        """Destroys the configuration in the simulation."""

        if not self.connected:
            raise ValueError("Physics engine not connected.")

        if self.configuration is None:
            raise ValueError("No configuration is in the engine.")

        bodies = (
            [self.configuration.surface]
            + self.configuration.obstacles
            + self.configuration.originals
            + self.configuration.news
        )

        for bid in [x.bid for x in bodies]:
            p.removeBody(bid, physicsClientId=self.eid)

        self.configuration = None

    def push_bodies(self, configuration, batch_size=10):
        """Modifies the physics engine's internal collision resolution to act as a
        potential field."""

        def static_equilibrium(history, s, e):
            """We say that the configuration is in static equilibrium when its
            moving average is less than some error e."""

            if len(history) >= s:
                cumsum_vec = np.cumsum(np.insert(history, 0, 0))
                ma_vec = (cumsum_vec[s:] - cumsum_vec[:-s]) / s
                return ma_vec[-1] < e
            return False

        if not self.__connected:
            raise ValueError("Physics engine not connected.")

        self.configuration = configuration

        history = []
        s = 20
        e = 0.5 * len(configuration.collidable)
        while not static_equilibrium(history, s, e):
            for _ in range(batch_size):
                p.stepSimulation(physicsClientId=self.eid)
            configuration.update_configuration()
            history.append(self.get_collision_info(configuration)["severity"])

    def get_collision_info(self, configuration):
        """Returns a dictionary with information containing:
        1) A boolean depicting the existence of at least one collision,
        2) the number of collisions,
        3) the cumulative penetration depth of all collisions, and
        4) the colliding Body objects."""

        def pairwaise_collision_check(body1, body2):
            """Returns the presence of a collision between two bodies and their
            penetration depth (0 if no collision)."""

            try:
                penetration = p.getClosestPoints(
                    body1.bid, body2.bid, 0, physicsClientId=self.eid
                )[0][8]
                collision = True

            except IndexError:
                penetration = 0
                collision = False

            return collision, -penetration

        self.configuration = configuration
        collision = False
        cum_penetration = 0
        colliding = set()
        collisions = 0

        for i, j in itertools.combinations(configuration.collidable, 2):
            _, penetration = pairwaise_collision_check(i, j)
            if penetration > self.collision_threshold:
                if i not in configuration.obstacles:
                    i.color = [1.0, 1.0, 0.0, 1.0]
                    j.color = [1.0, 1.0, 0.0, 1.0]
                collisions += 1
                cum_penetration += penetration
                collision = True
                colliding.add(i)
                colliding.add(j)

        for body in configuration.collidable:
            if body not in colliding:
                body.reset_color()

        collision_info = {
            "status": collision,
            "number": collisions,
            "severity": cum_penetration,
            "list": list(colliding),
        }
        return collision_info
