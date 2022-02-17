import numpy as np

def to_rle(map):
    prev = int(map[0][0])
    count = 1
    rle = []
    for i in range(1, len(map.ravel())):
        val = map.ravel()[i]
        if val == prev and i != (len(map.ravel()) - 1):
            count += 1
        if val != prev and i != (len(map.ravel()) - 1):
            rle.append(count)
            rle.append(prev)
            count = 1
            prev = int(val)
        if i == len(map.ravel()) - 1:
            if val != prev:
                rle.append(count)
                rle.append(prev)
                rle.append(1)
                rle.append(val)
            else:
                rle.append(count+1)
                rle.append(prev)
    return np.asarray(rle, dtype=np.uint32)
    
def decode_rle(rle, map_shape=(512, 512)):
    decoded = np.ones((map_shape[0] * map_shape[1]), dtype=np.uint32)
    i = 0
    for idx in range(0, len(rle), 2):
        count, val = rle[idx], rle[idx+1]
        end = min(i+count, len(decoded))
        decoded[i:end] = val
        i += count
    return decoded.reshape(map_shape)