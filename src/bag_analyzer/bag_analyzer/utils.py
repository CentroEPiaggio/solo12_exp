import numpy as np


def interp(x, xp, fp):
    if len(fp.shape) == 1:
        return np.interp(x, xp, fp)
    
    if len(fp.shape) == 2:
        return np.array([
            np.interp(x, xp, fp[:, i]) for i in range(fp.shape[1])
        ]).transpose()
