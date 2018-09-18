import cv2
import numpy as np
from utils import anorm

def xy2rt(line):
    """ (x1,y1,x2,y2) -> (rho,theta) """
    x0,y0 = 0,0
    x1,y1,x2,y2 = line
    rho = -x2*y1 + y2*x1
    rho /= np.sqrt((x2-x1)**2 + (y2-y1)**2)
    theta = np.arctan2(x1-x2, y2-y1)
    return rho, theta

def rt2xy(rt, scale=50):
    """ (rho,theta) -> (x1,y1,x2,y2) """
    rho, theta = rt
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = (x0 + scale*(-b))
    y1 = (y0 + scale*(a))
    x2 = (x0 - scale*(-b))
    y2 = (y0 - scale*(a))
    return x1,y1,x2,y2

class WallFinder(object):
    def __init__(self):
        pass
    def __call__(self, points):
        pass

class WallFinderHough(object):
    def __init__(self, dr, dt, mx_r=5.0, mx_t=np.pi):
        self.dr_ = dr
        self.dt_ = dt
        self.mx_r_ = mx_r
        self.mx_t_ = mx_t
        self.n_r_ = n_r = int(np.ceil(mx_r / dr))
        self.n_t_ = n_t = int(np.ceil(mx_t / dt))
        self.theta_ = np.linspace(0, mx_t, n_t)
        self.ix_t_ = np.arange(n_t)

        # trig cache
        self.sin_ = np.sin(self.theta_)
        self.cos_ = np.cos(self.theta_)
        self.acc_ = np.zeros((n_r, n_t), dtype=np.float32)

    def __call__(self, points, thresh=4.0):
        self.acc_.fill(0)
        for (x,y) in points:
            ix_r = (x * self.cos_ + y * self.sin_) / (self.mx_r_ / self.n_r_)
            ix_r = np.round(ix_r).astype(np.int32)
            #ix_r = np.round(ix_r + (self.n_r_- 1) / 2.0).astype(np.int32)
            np.add.at(self.acc_, (ix_r, self.ix_t_), 1.0)
        ri, ti = np.where(self.acc_ > thresh)
        print np.max(self.acc_)
        return self.dr_*ri, self.dt_*ti

        #print np.where(self.acc_)
        #self.acc_[ np.where(self.acc_) ] = 1.0
        #self.acc_ = cv2.dilate(self.acc_, np.ones((3,3),np.float32), iterations=1)

        #self.acc_ = (255 * self.acc_).astype(np.uint8)
            #for r_, t_ in (r,self.theta_):
            #    r_ = r_ + (self.n_r_- 1) / 2;
            #    acc_[(n+1) * (self.n_r_+2) + r+1]++;_
            #print self.acc_

class WallFinderRANSAC(object):
    def __init__(self):
        pass
    def __call__(self, points):
        pass

def random_line(n=50):
    p0 = np.random.normal(size=(1,2))
    t  = np.random.uniform(-np.pi, np.pi)
    print 'angle : {}'.format( anorm(t + np.pi))
    u  = np.asarray([np.cos(t), np.sin(t)], dtype=np.float32)
    print p0, u
    return p0 + u * np.linspace(-1.0, 1.0, num=n).reshape(-1,1)

def line_from_hough(r, t, n=5):
    p0 = np.reshape([r * np.cos(t), r * np.sin(t)], (1,2))
    u  = np.cos(t + np.pi/2), np.sin(t + np.pi/2) # or swap... whatever
    u  = np.reshape(u, (1,2))
    return p0 + u * np.linspace(-1.0, 1.0, num=n).reshape(-1,1)

def main():
    wall_finder = WallFinderHough(dr=0.05, dt=np.deg2rad(5.0))
    #pts = [(1.0, 0.37), (1.0, 0.87)]
    pts = random_line(n=10)
    print 'line', pts
    rs,ts = hough = wall_finder(pts)
    print hough
    print '?', rt2xy( (rs[0], ts[0]) )
    print 'line-rec', line_from_hough(hough[0][0], hough[1][0])
    cv2.namedWindow('acc', cv2.WINDOW_NORMAL)
    cv2.imshow('acc', wall_finder.acc_)
    while True:
        k = cv2.waitKey(10)
        if k == 27:
            break

if __name__ == "__main__":
    main()
