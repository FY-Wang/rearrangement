#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pytest
import math
from rearrangement.physics import (
    Body,
    Configuration,
    Constraint,
    Engine,
    to_euler,
    to_quaternion,
)


def test_to_euler():
    ans = [round(x, 5) for x in to_euler([0.5, 0.5, 0.5, 0.5])]
    assert ans == [round(math.pi / 2, 5), 0, round(math.pi / 2, 5)]


def test_to_quaternion():
    ans = [round(x, 5) for x in to_quaternion([math.pi / 2, 0, math.pi / 2])]
    assert ans == [0.5, 0.5, 0.5, 0.5]
