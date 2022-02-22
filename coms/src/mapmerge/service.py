import numpy as np
from keypoint_merge import sift_mapmerge, orb_mapmerge
from merge_utils import resize_map, combine_aligned_maps, acceptance_index

def mapmerge_pipeline(map1, map2, method="sift", scale_fix=True):
    """
    end-to-end map merge pipeline for testing
    
    args: 
    - map1 (original map)
    - map2 (foreign map, the one to be transformed onto map1)
    - method: one of ["sift", "orb"]
    - scale_fix: boolean, whether or not to attempt scale-adjusted merge
    """
    merge_fn = sift_mapmerge if method == "sift" else orb_mapmerge
    if scale_fix:
        for scale in [0.95, 1, 1.05]:
            ious = []
            merges = []
            map2_scale = resize_map(map2, dsize=(int(map2.shape[0]*scale), int(map2.shape[1]*scale)))
            transformed_map2 = merge_fn(map1, map2_scale)
            merged_map = combine_aligned_maps(transformed_map2, map1)
            ious.append(acceptance_index(map1, merged_map))
            merges.append(merged_map)
        return merges[np.argmax(ious)]
    else:
        transformed_map2 = merge_fn(map1, map2)
        merged_map = combine_aligned_maps(transformed_map2, map1)
        return merged_map
