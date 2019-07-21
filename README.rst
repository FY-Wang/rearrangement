Placement and Planning for Rearrangement Problems
--------------------------------------------------------------------------------

Implementation of the methods described in `[1]`_.

Dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

+---------------+-------------------+
| Dependency    | Version Tested On |
+===============+===================+
| `clingo`_     |            5.2.2  |
+---------------+-------------------+
| `DLVHEX`_     |            2.5.0  |
+---------------+-------------------+
| `matplotlib`_ |            2.2.4  |
+---------------+-------------------+
| `numpy`_      |            1.16.4 |
+---------------+-------------------+
| `pandas`_     |           0.24.2  |
+---------------+-------------------+
| `Pillow`_     |            6.0.0  |
+---------------+-------------------+
| `pybullet`_   |            2.5.0  |
+---------------+-------------------+
| `Python`_     |           2.7.15  |
+---------------+-------------------+
| `Ubuntu`_     |            18.04  |
+---------------+-------------------+


Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#.  install `clingo`_ and `DLVHEX`_ such that they are callable with ``clingo``
    and ``dlvhex``, respectively.
#.  ``cd [your virtual environments directory]``
#.  ``virtualenv rearrangement``
#.  ``source [your virtual environments directory]/rearrangement/bin/activate``
#.  ``cd [rearrangement directory]``
#.  ``pip install .``

About
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Given a surface with movable objects (clutter) and immovable objects
(obstacles), the goal is to add another set of (new) objects to the surface.
The overall approach is explained in `[1]`_.


#.  The **placement** stage finds a collision-free final
    configuration for all objects (all the new objects together with all other
    objects in the clutter) while also trying to minimize the number of object
    relocations, and the amount of movement each object is relocated. At this
    stage, the number, type, order, and feasibility of the move actions
    required to achieve this goal configuration are not considered.

#.  The **planning** stage is divided into two:

    #.  The **discretization** stage takes, as input, the initial
        configurations and the final configurations of all objects on
        the cluttered surface, and divides the surface into a minimum number of
        non-uniform grid cells. During gridization of the continuous plane, an
        object is allowed to span multiple grid cells as long as each grid cell
        contains the centroid of a single object.
    #.  The **planning** stage aims to find a sequence of feasible move actions
        to achieve the final placement of the objects in the clutter from their
        initial discrete placement, while simultaneously minimizing the number
        of actions and object relocations.

This package contains:

-   the ``physics`` module: a wrapper of the `pybullet`_ implementation of the
    `Bullet`_ physics engine specifically for the problem at hand,
-   the ``search`` module: a generic search framework with lexicographic
    comparison,
-   the ``placement`` module uses local searches to find a goal
    placement for the problem:

    -   the ``inner`` submodule: a local search wrapper of `pybullet`_'s
        collision resolver,
    -   the ``middle`` submodule: a local search that uses a grid-based
        heuristic on top of ``inner`` to minimize the number of objects in
        collision,
    -   the ``outer`` submodule: a local search that uses circular constraints
        on top of ``middle`` to minimize the number of original objects moved
        and their movement,
    -   the ``baselines`` submodule: naive local search baselines for
        comparison,

-   the ``planning`` module uses the hybrid planning architecture `[2]`_ to
    compute a feasible plan of actions to realize the rearrangement:

    -   the ``discretization`` submodule: an optimal discretizer
        using `clingo`_,
    -   the ``dlvhex`` submodule: the planning domain, as well as tools to link
        `DLVHEX`_ to the `Python`_ code,
    -   the ``feasibility`` submodule: a local search that uses ``middle`` to
        act as the low-level feasibility checker for the hybrid planner,

-   and some example instances.

All modules are written to be very general, and can be easily extended/wrapped
with new features.


Notes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-   When resolving collisions, the threshold of penetration depth at which a
    collision is recognized can be assigned using the
    ``physics.engine.set_collision_threshold()`` method, and can drastically
    improve performance without sacrificing robustness if set appropriately.

-   ``middle`` uses a bit of stochasticity when re-placing. This is the only
    difference from the methods in `[1]`_.


Usage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To see a demo of the PGHP algorithm, run ``python main.py -v True``

``main.py`` is available as a sample code of how to use the software. The way
to input a problem is by defining a query JSON file, which defines the shapes
and sizes (by linking to URDF files which can link to obj or stl files), and
poses of each object involved in the problem.

The query JSON file also describes the structure of problem by defining which
objects are originals, obstacles, etc. The result can be output as a JSON file,
structured identically to the query JSON. Example JSON queries are given in the
`data`_ folder. `Pillow`_ is used to save images of the state at any time, and
`matplotlib`_ is used to plot discrete configurations and performance measures.

References
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-   `[1]`_: Dabbour, Abdul Rahman, Esra Erdem, and Volkan Patoglu. "Object
    Placement on Cluttered Surfaces: A Nested Local Search Approach."
    *arXiv preprint arXiv:1906.08494* (2019).
-   `[2]`_: Erdem, Esra, Volkan Patoglu, and Peter Schüller. "A systematic
    analysis of levels of integration between high-level task planning and
    low-level feasibility checks." *AI Communications 29.2* (2016): 319-349.

License
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

`GNU Affero General Public License v3.0`_

.. _Ubuntu: https://www.ubuntu.com
.. _Python: https://www.python.org
.. _pybullet: https://pybullet.org
.. _Bullet: https://pybullet.org
.. _numpy: https://www.numpy.org
.. _pandas: https://pandas.pydata.org
.. _matplotlib: https://matplotlib.org
.. _clingo: https://potassco.org/clingo
.. _DLVHEX: http://www.kr.tuwien.ac.at/research/systems/dlvhex/index.html
.. _data: /data/
.. _Pillow: https://pillow.readthedocs.io
.. _GNU Affero General Public License v3.0: /LICENSE
.. _[1]: https://arxiv.org/abs/1906.08494
.. _[2]: https://content.iospress.com/articles/ai-communications/aic697
