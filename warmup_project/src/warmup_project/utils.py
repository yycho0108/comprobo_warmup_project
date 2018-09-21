import numpy as np

def anorm(x):
    return (x+np.pi) % (2*np.pi) - np.pi

def adiff(a,b):
    return anorm(a-b)

def rq2xy(rq):
    # (n,2) -> (n,2)
    r, q = rq.T
    x = r * np.cos(q)
    y = r * np.sin(q)
    return np.stack([x,y], axis=-1)

def xy2rq(xy):
    x, y = xy.T
    r = np.linalg.norm([x,y], axis=0)
    q = np.arctan2(y,x)
    return np.stack([r,q], axis=-1)

def R(x):
    c = np.cos(x)
    s = np.sin(x)
    r = [[c,-s],[s,c]]
    return np.asarray(r, dtype=np.float32)

