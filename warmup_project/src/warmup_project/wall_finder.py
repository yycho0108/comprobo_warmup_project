import cv2
import numpy as np
from utils import anorm
import scipy.ndimage as ndi
import utils as U

#from skimage.feature import peak_local_max
def _get_high_intensity_peaks(image, mask, num_peaks):
    """
    Return the highest intensity peak coordinates.
    """
    # get coordinates of peaks
    coord = np.nonzero(mask)
    # select num_peaks peaks
    if len(coord[0]) > num_peaks:
        intensities = image[coord]
        idx_maxsort = np.argsort(intensities)
        coord = np.transpose(coord)[idx_maxsort][-num_peaks:]
    else:
        coord = np.column_stack(coord)
    # Higest peak first
    return coord[::-1]

def peak_local_max_wrap(image,
        min_distance=1,
        threshold_abs=None,
        threshold_rel=None,
        num_peaks=np.inf,
        ):
    
    if np.all(image == image.flat[0]):
        return np.empty((0, 2), np.int)

    # Non maximum filter
    # if footprint is not None:
    #     image_max = ndi.maximum_filter(image, footprint=footprint,
    #                                    mode='wrap')
    # else:
    #     size = 2 * min_distance + 1
    #     image_max = ndi.maximum_filter(image, size=size, mode='wrap')

    # maximum filter with gaussian kernel
    ker = cv2.getGaussianKernel(U.rint(2*min_distance+1),sigma=2.0)
    ker = ker*ker.T
    image_max = ndi.maximum_filter(image, footprint=ker, mode='wrap')

    # alternatively,
    # size = 2 * min_distance + 1
    # image_max = ndi.maximum_filter(image, size=size, mode='wrap')

    mask = image == image_max

    # find top peak candidates above a threshold
    thresholds = []
    if threshold_abs is None:
        threshold_abs = image.min()
    thresholds.append(threshold_abs)
    if threshold_rel is not None:
        thresholds.append(threshold_rel * image.max())
    if thresholds:
        mask &= image > max(thresholds)

    # Select highest intensities (num_peaks)
    coordinates = _get_high_intensity_peaks(image, mask, num_peaks)
    return coordinates


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
    def __init__(self, dr=0.03, dt=0.02, mx_r=5.0, mx_t=np.pi):
        self.dr_ = dr
        self.dt_ = dt
        self.mx_r_ = mx_r
        self.mx_t_ = mx_t
        self.n_r_ = n_r = int(np.ceil(mx_r / dr))
        self.n_t_ = n_t = int(np.ceil(mx_t / dt))
        self.rad_   = np.linspace(0, mx_r, n_t) - (mx_r / 2.0)
        self.theta_ = np.linspace(0, mx_t, n_t)
        self.ix_t_ = np.arange(n_t)

        # trig cache
        self.sin_ = np.sin(self.theta_)
        self.cos_ = np.cos(self.theta_)
        self.pad_ = 10 # prevent wrap - TODO: base this on physical quantity parameter
        self.acc_ = np.zeros((n_r, 2*n_t), dtype=np.float32)

    def accumulate(self, points, acc):
        acc.fill(0)
        for (x,y) in points:
            if not (np.isfinite(x) and np.isfinite(y)): continue
            r = np.linalg.norm([x,y])
            # opt1
            ix_r = (x * self.cos_ + y * self.sin_) / (self.mx_r_ / self.n_r_)
            ix_t = self.ix_t_.copy().astype(np.float32)

            # deal with negative radius
            ix_t[ix_r < 0] += (np.pi / self.dt_)
            ix_t %= (2*self.n_t_)
            ix_r = np.abs(ix_r) # NOTE : abs(ix_r) must come AFTER ix_t corrections
            ix_r %= self.n_r_

            # convert to index
            ix_r = U.rint(ix_r)
            ix_t = U.rint(ix_t)

            # filter by max radius
            sel = (ix_r < self.n_r_)
            ix_r = ix_r[sel]
            ix_t = ix_t[sel]

            try:
                np.add.at(acc, (ix_r, ix_t), r) # TODO : weighted addition based on radius?
            except Exception as e:
                print x
                print y
                raise e

    def detect(self, thresh, max_lines=np.inf):
        print self.acc_.max()
        peak_idx = peak_local_max_wrap(self.acc_, min_distance=4.0,
                threshold_abs=thresh,
                #threshold_rel=0.25,
                num_peaks=max_lines
                )
        #peak_idx = np.asarray(np.where(self.acc_ > thresh)).T
        print peak_idx.shape
        if len(peak_idx) > 0:
            #print 'indices', peak_idx
            ri, ti = peak_idx.T
            r, t = self.dr_*ri, U.anorm(self.dt_*ti)
            #print 'r', r, 't', np.rad2deg(t)
            return np.stack([r,t], axis=-1)
            #for r_,t_ in zip(r,t):
            #    print('r-t', r_, t_)
            #return r, t 

            #(np.abs(r - r.T) < self.dr_) & (U.adiff(t-t.T) < self.dt_):
            #print 'r-t', self.dr_*ri, U.anorm(self.dt_*ti)
            #return self.dr_*ri, self.dt_*ti
        else:
            return None

    def merge(self, rt, r_thresh, t_thresh):
        if rt is None:
            return rt
        #print('r_thresh : {}, t_thresh : {}'.format(r_thresh, t_thresh))

        rt2 = []
        n = len(rt)
        src = range(n)

        while src:
            # ref
            i0 = src.pop(0)
            r0, t0 = rt[i0]
            rs, ts = [r0], [t0]

            # merge
            for i1 in src:
                r1, t1 = rt[i1]
                if np.abs(r0-r1)<=r_thresh and np.abs(U.adiff(t0,t1))<=t_thresh:
                    src.remove(i1)
                    rs.append(r1)
                    ts.append(t1)
            # add
            rt2.append( [np.mean(rs), U.amean(ts)] )

        return np.asarray(rt2)

    def cut(self, lines, points):
        pass


    def __call__(self, points, thresh=50.0):
        self.accumulate(points, self.acc_)

        cv2.namedWindow('acc', cv2.WINDOW_NORMAL)
        cv2.imshow('acc', self.acc_ / np.max(self.acc_))# / np.max(wall_finder.acc_))
        cv2.waitKey(10)

        rt = self.detect(thresh)
        #if rt is None:
        #    return None

        #rt0 = rt
        #n0 = len(rt)

        rt = self.merge(rt, 0.03, np.deg2rad(3.0))
        if rt is None:
            return None

        #print rt
        #rt1 = rt
        #n1 = len(rt)
        #if n0 != n1:
        #    print 'merge'
        #    print rt0
        #    print rt1
        if len(rt) > 0:
            return rt.T # returned as (r,t)
        else:
            return None

class WallFinderRANSAC(object):
    def __init__(self):
        pass
    def __call__(self, points):
        pass

def random_line(n=50):
    r = np.random.uniform(5)
    t = np.random.uniform(-np.pi,np.pi)
    print('r-t (orig)', r, t)
    p0 = [r * np.cos(t), r * np.sin(t)]
    p0 = np.reshape(p0, [1,2])
    u = [np.sin(t), -np.cos(t)]
    u = np.reshape(u, [1,2])
    return p0 + u * np.linspace(-1.0, 1.0, num=n).reshape(-1,1)
    #p0 = 5.0 * np.random.uniform(size=(1,2))
    #t  = np.random.uniform(-np.pi, np.pi)
    #print 'angle : {}'.format( anorm(t + np.pi))
    #u  = np.asarray([np.cos(t), np.sin(t)], dtype=np.float32)
    #print p0, u
    #return p0 + u * np.linspace(-1.0, 1.0, num=n).reshape(-1,1)

def line_from_hough(r, t, n=5):
    p0 = np.reshape([r * np.cos(t), r * np.sin(t)], (1,2))
    u  = np.cos(t + np.pi/2), np.sin(t + np.pi/2) # or swap... whatever
    u  = np.reshape(u, (1,2))
    return p0 + u * np.linspace(-1.0, 1.0, num=n).reshape(-1,1)

def main():
    wall_finder = WallFinderHough(dr=0.05, dt=np.deg2rad(1.8))
    #pts = [(1.0, 0.37), (1.0, 0.87)]
    pts = random_line(n=10)
    pts = np.concatenate([pts, random_line(n=10)], axis=0)
    pts = np.concatenate([pts, random_line(n=10)], axis=0)
    #print 'line', pts
    rs, ts = hough = wall_finder(pts)
    print rs, ts
    cv2.namedWindow('acc', cv2.WINDOW_NORMAL)
    cv2.imshow('acc', wall_finder.acc_)# / np.max(wall_finder.acc_))
    while True:
        k = cv2.waitKey(10)
        if k == 27:
            break

if __name__ == "__main__":
    main()
