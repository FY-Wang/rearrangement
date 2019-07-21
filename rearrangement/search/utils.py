import operator
import copy

import numpy as np


def pairwise_comparison(old_tuple, new_tuple, find_min=True):
    """Performs a lexicographic comparison to see if new_tuple is better
    than old_tuple, provided they are the same length.

    find_min=True  ==> smaller is better
    find_min=False ==> bigger is better"""

    def get_best(old_tuple, new_tuple, comparator, i):
        if i == len(old_tuple):
            return False
        if comparator(new_tuple[i], old_tuple[i]):
            return True
        elif new_tuple[i] == old_tuple[i]:
            return get_best(old_tuple, new_tuple, comparator, i + 1)

    if len(old_tuple) != len(new_tuple):
        raise ValueError("The tuples don't match sizes.")

    if not isinstance(find_min, bool):
        raise ValueError("Only 'True' or 'False' allowed for find_min.")

    op = operator.lt if find_min else operator.gt

    return get_best(old_tuple, new_tuple, op, 0)


def multiple_comparison(tuples, find_min=True):
    """Compares a list of tuples lexicographically, returns the best.

    find_min=True  ==> smaller is better
    find_min=False ==> bigger is better"""

    length = len(tuples[0])

    if find_min:
        best = (np.inf,) * length
    else:
        best = (0,) * length

    for i in tuples:
        if pairwise_comparison(best, i, find_min):
            best = copy.deepcopy(i)

    return best
