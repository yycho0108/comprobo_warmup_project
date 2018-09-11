import numpy as np

def anorm(x):
    return (x+np.pi) % (2*np.pi) - np.pi

def adiff(a,b):
    return anorm(a-b)

