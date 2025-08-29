import numpy as np

def wn_zeta(arr):
    wn = np.abs(arr)
    zeta = -np.real(arr) / wn
    return wn, zeta