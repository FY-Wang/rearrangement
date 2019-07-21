#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pytest
import random
from rearrangement.search import pairwise_comparison, multiple_comparison


def test_pairwise_comparison_minimize():
    assert pairwise_comparison((2, 3, 4, 9), (2, 3, 5, 7), find_min=False)


def test_pairwise_comparison_maximize():
    assert pairwise_comparison((2, 3, 5, 7), (2, 3, 4, 9), find_min=True)


def test_multiple_comparison_minimize():
    tuples = [(3, 3, 4, 9), (2, 4, 4, 9), (2, 3, 5, 9), (2, 3, 4, 10)]
    random.shuffle(tuples)
    assert multiple_comparison(tuples, find_min=False) == (3, 3, 4, 9)


def test_multiple_comparison_maximize():
    tuples = [(3, 3, 4, 9), (2, 4, 4, 9), (2, 3, 5, 9), (2, 3, 4, 10)]
    random.shuffle(tuples)
    assert multiple_comparison(tuples, find_min=True) == (2, 3, 4, 10)
