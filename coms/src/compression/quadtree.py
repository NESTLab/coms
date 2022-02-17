import numpy as np
from mapmerge.constants import *

SPLIT_IND = 0
FREE_IND = 1
UNKNOWN_IND = 2
OCCUPIED_IND = 3

IND_MAP = {
    FREE_IND : FREE,
    UNKNOWN_IND : UNKNOWN,
    OCCUPIED_IND : OCCUPIED
}

def to_quadtree(arr):
    def split(arr):
        """
        Split array into four equal quadrants
        """
        mid_x, mid_y = arr.shape[0] // 2, arr.shape[1] // 2
        q1 = arr[0:mid_x, 0:mid_y]
        q2 = arr[0:mid_x, mid_y::]
        q3 = arr[mid_x::, 0:mid_y]
        q4 = arr[mid_x::, mid_y::]
        return [q1, q2, q3, q4]
    
    quadtree_stream = []
    stack = [arr]
    while (len(stack) > 0):
        curr = stack.pop(0)
        if len(np.unique(curr)) > 1:
            stack = split(curr) + stack  # prepend
            quadtree_stream.append(SPLIT_IND)
        else:
            cell = np.unique(curr)
            if cell == UNKNOWN:
                quadtree_stream.append(UNKNOWN_IND)
            elif cell == FREE:
                quadtree_stream.append(FREE_IND)
            else:
                quadtree_stream.append(OCCUPIED_IND)
    quadtree_arr = np.asarray(quadtree_stream, dtype=np.uint16)
    return quadtree_arr

def decode_quadtree(quadtree_arr, map_shape=(512, 512)):
    decoded = np.zeros(shape=map_shape, dtype=np.uint16)
    # keep track of where we will be putting in values
    offset_stack = [(0, 0, map_shape[0], map_shape[1])]
    for val in quadtree_arr:
        start_x, start_y, end_x, end_y = offset_stack.pop(0)
        if val != SPLIT_IND:
            decoded[start_x:end_x, start_y:end_y] = IND_MAP[val]
        else:
            # append new offset indices for four splits
            mid_x, mid_y = (end_x + start_x) // 2, (end_y + start_y) // 2
            q1_coord = (start_x, start_y, mid_x,  mid_y)
            q2_coord = (start_x, mid_y, mid_x, end_y)
            q3_coord = (mid_x, start_y, end_x, mid_y)
            q4_coord = (mid_x, mid_y, end_x, end_y)

            offset_stack = [q1_coord, q2_coord, q3_coord, q4_coord] + offset_stack
    return decoded

def split(arr):
    """
    Split array into four equal quadrants
    """
    mid_x, mid_y = arr.shape[0] // 2, arr.shape[1] // 2
    q1 = arr[0:mid_x, 0:mid_y]
    q2 = arr[0:mid_x, mid_y::]
    q3 = arr[mid_x::, 0:mid_y]
    q4 = arr[mid_x::, mid_y::]
    return q1, q2, q3, q4