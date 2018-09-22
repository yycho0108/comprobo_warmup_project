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

def inv_22(A):
    a = A[0,0]
    b = A[0,0]
    c = A[0,0]
    d = A[0,0]
    D = np.linalg.det(A)

def uvec(x, axis=-1):
    return x / np.linalg.norm(x, axis=axis)

def R(x):
    c = np.cos(x)
    s = np.sin(x)
    r = [[c,-s],[s,c]]
    return np.asarray(r, dtype=np.float32)

#def seg_ix(seg0, seg1):
#    (x1,y1), (x2,y2) = seg0
#    (x3,y3), (x4,y4) = seg1
#
#    ta_n = (y3-y4)*(x1-x3)+(x4-x3)*(y1-y3)
#    ta_d = (x4-x3)*(y1-y2)-(x1-x2)*(y4-y3)
#    ta = (ta_n / ta_d)
#
#    tb_n=(y1-y2)*(x1-x3)+(x2-x1)*(y1-y3)
#    tb_d=(x4-x3)*(y1-y2)-(x1-x2)*(y4-y3)
#    tb = (tb_n / tb_d)
#
#    return ta, tb

def lineRayIntersectionPoint(rayOrigin, rayDirection, point1, point2):
    """
    from https://stackoverflow.com/a/29020182
    """

    # Convert to numpy arrays
    rayOrigin = np.array(rayOrigin, dtype=np.float)
    rayDirection = np.array(uvec(rayDirection), dtype=np.float32)
    point1 = np.array(point1, dtype=np.float)
    point2 = np.array(point2, dtype=np.float)

    # Ray-Line Segment Intersection Test in 2D
    # http://bit.ly/1CoxdrG
    v1 = rayOrigin - point1
    v2 = point2 - point1
    v3 = np.array([-rayDirection[1], rayDirection[0]])
    if np.dot(v2,v3) == 0:
        return None
    t1 = np.cross(v2, v1) / np.dot(v2, v3)
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
        return rayOrigin + t1 * rayDirection
    return None

class ScanFootprint(object):
    """
    Transform scan to represent offset from robot border.
    """
    def __init__(self, fpt, ang):
        self.fpt_ = fpt
        self.ang_ = ang
        self.dr_ = np.zeros_like(self.ang_)

        n = len(fpt)
        for i in range(-1,n-1):
            pa, pb = self.fpt_[i], self.fpt_[i+1]
            for ai, h in enumerate(ang):
                c,s = np.cos(h), np.sin(h)
                ix = lineRayIntersectionPoint([0,0], [c,s], pa, pb)
                if ix is not None:
                    self.dr_[ai] = np.linalg.norm(ix)

    def show(self):
        from matplotlib import pyplot as plt
        c = np.cos(self.ang_)
        s = np.sin(self.ang_)
        x = self.dr_ * c
        y = self.dr_ * s
        plt.scatter(x,y)
        plt.show()

    def __call__(self, scan, scale=1.0):
        return scan - (self.dr_ * scale)

def main():
    fpt = np.asarray([[ 0.25649121, -0.17213032],
        [ 0.17213032, -0.17468672],
        [ 0.0562406 , -0.17468672],
        [-0.00596491, -0.14315788],
        [-0.05027569, -0.09117794],
        [-0.06987468, -0.00426065],
        [-0.05027569,  0.07243107],
        [ 0.00170426,  0.13804510],
        [ 0.05624060,  0.16616541],
        [ 0.16957393,  0.16957393],
        [ 0.25478697,  0.16786967]], dtype=np.float32)
    ang = anorm(np.linspace(0,2*np.pi,360,endpoint=True))
    sfp = ScanFootprint(fpt,ang)
    sfp.show()

if __name__ == "__main__":
    main()



