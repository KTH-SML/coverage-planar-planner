import numpy as np
import warnings as wrn


def normalize(array):
    array = np.array(array)
    norm = np.linalg.norm(array)
    if norm < 0.99 or norm > 1.01:
            wrn.warn('This does not seem to be a unit vector.')
            wrn.warn('The vecotr will be normalized.')
    if norm <= 0.01:
        wrn.warn('The norm is too small.')
        wrn.warn('Setting the vector to (1,0).')
        array = np.array([1.0, 0.0])
        norm = 1.0
    return array/norm


def saturate(array, ths):
    norm = np.linalg.norm(array)
    if norm > ths:
        array *= ths/norm
    return array
