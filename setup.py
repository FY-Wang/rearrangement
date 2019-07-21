#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Environment setup. Adapted from https://github.com/kennethreitz/setup.py.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0

    Repository: https://github.com/ardabbour/rearrangement/
"""

import io
import os
import sys
from shutil import rmtree

from setuptools import find_packages, setup, Command

# Package meta-data.
NAME = "rearrangement"
DESCRIPTION = "Placement and Planning for Rearrangement Problems"
URL = "https://github.com/ardabbour/rearrangement"
EMAIL = "dabbour@sabanciuniv.edu"
AUTHOR = "Abdul Rahman Dabbour"
REQUIRES_PYTHON = ">=2.7.0"
VERSION = None

REQUIRED = ["matplotlib", "numpy", "pandas", "pillow", "pybullet"]

here = os.path.abspath(os.path.dirname(__file__))

try:
    with io.open(os.path.join(here, "README.rst"), encoding="utf-8") as f:
        long_description = "\n" + f.read()
except IOError:
    long_description = DESCRIPTION

# Load the package's __version__.py module as a dictionary.
about = {}
if not VERSION:
    with open(os.path.join(here, NAME, "__version__.py")) as f:
        exec (f.read(), about)
else:
    about["__version__"] = VERSION

# Where the magic happens:
setup(
    name=NAME,
    version=about["__version__"],
    description=DESCRIPTION,
    long_description=long_description,
    long_description_content_type="text/markdown",
    author=AUTHOR,
    author_email=EMAIL,
    python_requires=REQUIRES_PYTHON,
    url=URL,
    packages=find_packages(exclude=("tests",)),
    install_requires=REQUIRED,
    test_requires=["pytest"],
    include_package_data=True,
    license="GNU AGPLv3",
    classifiers=[
        "License :: OSI Approved :: GNU Affero General Public License v3 or later (AGPLv3+)",
        "Natural Language :: English",
        "Programming Language :: Python :: 2.7",
    ],
)
