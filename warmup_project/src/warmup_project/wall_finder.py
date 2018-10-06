import cv2
import numpy as np
from utils import anorm
import scipy.ndimage as ndi
import utils as U

from matplotlib import pyplot as plt

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
    """ adapted from skimage to support theta wrap """
    
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

class WallFider(object):
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

        # params
        self.detect_args_ = {'dmin':10.0}
        self.merge_args_ = {'max_dr':0.05, 'max_dh':np.deg2rad(3.0)}
        self.cut_args_ = {'max_dr':0.1, 'max_dh':0.06, 'max_gap':0.3}

        # trig cache
        self.sin_ = np.sin(self.theta_)
        self.cos_ = np.cos(self.theta_)
        self.pad_ = 10 # prevent wrap - TODO: base this on physical quantity parameter
        self.acc_ = np.zeros((n_r, 2*n_t), dtype=np.float32)

    @staticmethod
    def hough(x,y,h,s=None,c=None):
        # handle trig cache
        if s is None:
            s=np.sin(h)
        if c is None:
            c=np.cos(h)

        # apply hough
        r=x*c+y*s
        r,h=np.broadcast_arrays(r,h)
        h=h.copy()

        # deal with negative radius
        h[r<0] += np.pi
        h %= (2*np.pi)
        r=np.abs(r)

        return r,h

    def accumulate(self, points, acc):
        acc.fill(0)
        for (x,y) in points:
            if not (np.isfinite(x) and np.isfinite(y)): continue
            r = np.linalg.norm([x,y])
            ix_r, ix_t = self.hough(x,y,self.theta_,s=self.sin_,c=self.cos_)
            ix_r *= (self.n_r_/self.mx_r_)
            ix_t *= (self.n_t_/self.mx_t_)

            # convert to index
            ix_r = U.rint(ix_r)
            ix_t = U.rint(ix_t)

            # filter by max radius
            sel = (ix_r < self.n_r_)
            ix_r = ix_r[sel]
            ix_t = ix_t[sel]

            try:
                # weighted addition based on radius
                # TODO : is this a better method?
                np.add.at(acc, (ix_r, ix_t), r)
            except Exception as e:
                print x
                print y
                raise e

    def detect(self, thresh, max_lines=np.inf):
        #print self.acc_.max()
        peak_idx = peak_local_max_wrap(self.acc_,
                min_distance=self.detect_args_['dmin'],
                threshold_abs=thresh,
                #threshold_rel=0.25,
                num_peaks=max_lines
                )
        #peak_idx = np.asarray(np.where(self.acc_ > thresh)).T
        #print peak_idx.shape
        if len(peak_idx) > 0:
            #print 'indices', peak_idx
            ri, ti = peak_idx.T
            r, t = self.dr_*ri, U.anorm(self.dt_*ti)
            #print 'r', r, 't', np.rad2deg(t)
            return np.stack([r,t], axis=-1), self.acc_[ri,ti]
            #for r_,t_ in zip(r,t):
            #    print('r-t', r_, t_)
            #return r, t 

            #(np.abs(r - r.T) < self.dr_) & (U.adiff(t-t.T) < self.dt_):
            #print 'r-t', self.dr_*ri, U.anorm(self.dt_*ti)
            #return self.dr_*ri, self.dt_*ti
        else:
            return None

    def merge(self, rt, v, max_dr=None, max_dh=None):
        if rt is None:
            return None

        # supply default arguments
        if max_dr is None:
            max_dr = self.merge_args_['max_dr']
        if max_dh is None:
            max_dh = self.merge_args_['max_dh']

        rt2 = []
        v2=[]
        n = len(rt)
        src = range(n)

        while src:
            # ref
            i0 = src.pop(0)
            r0, t0 = rt[i0]
            rs, ts, vs = [r0], [t0], [v[i0]]

            # merge
            for i1 in src:
                r1, t1 = rt[i1]
                v1 = v[i1]
                if np.abs(r0-r1)<=max_dr and np.abs(U.adiff(t0,t1)) <= max_dh:
                    src.remove(i1)
                    rs.append(r1)
                    ts.append(t1)
                    vs.append(v1)
            # add
            rt2.append( [np.mean(rs), U.amean(ts)] )
            v2.append(np.sum(vs))

        return np.asarray(rt2), np.asarray(v2)

    def cut(self, lines, points, weights,
            max_dr=0.1, max_dh=0.06, max_gap=0.3):
        # TODO : evaluate non-exclusive + merge vs. exclusive
        # TODO : reformat code to be actually readible and sensible

        #search = [2.83783878, -3.28601199]
        #pidx = np.argmin(np.linalg.norm(points - np.reshape(search,[-1,2]),axis=-1))
        #print 'pidx', pidx 
        #print '# lines : {}'.format(len(lines))
        #print 'points'
        #print points

        # lines  = [L, 2] -- (r,t)
        # points = [P, 2] -- (x,y)
        nax = np.newaxis

        rl, hl = lines.T
        c, s = np.cos(hl), np.sin(hl)
        x, y = points.T
        #rp, hp = (x[:,nax]*c[nax,:] + y[:,nax]*s[nax,:])
        rp, hp = self.hough(x[:,nax], y[:,nax],
                hl[nax,:], s[nax,:], c[nax,:])

        # global dr
        dr = np.abs(rp - rl[nax,:])
        dh = np.abs(U.adiff(hp, hl[nax,:]))

        # local (true) assignment dr
        dr_asn = np.min(dr, axis=1)
        dh_asn = np.min(dh, axis=1)

        #print 'r-h', dr_asn[pidx], dh_asn[pidx]

        # filter by assignment "cost"
        sel = np.logical_and(dr_asn<max_dr, dh_asn<max_dh)

        #p = points[sel]
        #a = np.argmin(dr[sel] / weights[nax,:], axis=1)
        #print 'asgn', a

        #l = [[] for _ in lines]
        #cnt = np.zeros(lines.shape[0])
        #for p_,a_ in zip(p,a):
        #    x,y=p_
        #    l[a_].append(-s[a_]*x+c[a_]*y)
        #    cnt[a_] += 1

        #sel = (cnt > 1)

        # non-exclusive assignment
        segs = []
        lp_sel = np.logical_and(dr < max_dr, dh < max_dh)
        sel_mask = np.zeros_like(lp_sel[:,0])

        # prioritize large line
        idx = np.argsort(weights)[::-1]

        for i in idx:

            r_ = rl[i]
            c_ = c[i]
            s_ = s[i]

            pmask = lp_sel[:,i]

            # exclusive version
            pmask &= (~sel_mask)
            sel_mask |= pmask

            p0 = np.asarray([r_*c_, r_*s_])
            tv = np.asarray([-s_,c_])

            cand = points[pmask] # candidate
            x, y = cand.T
            t = -s_*x+c_*y
            t.sort()
            s_idx = 1+np.where(np.diff(t) > max_gap)[0] # TODO : apply max_gap param
            for seg in np.split(t, s_idx):
                if len(seg) <= 1:
                    continue
                seglen = np.max(seg) - np.min(seg)
                if seglen <= max_gap:
                    continue
                pa = p0 + tv * np.min(seg)
                pb = p0 + tv * np.max(seg)
                segs.append( [pa, pb] )
        return np.asarray(segs, dtype=np.float32)

    def __call__(self, points, thresh=10.0):
        self.accumulate(points, self.acc_)
        rt, v = self.detect(thresh)
        rt, v = self.merge(rt, v)
        ls = self.cut(rt, points, v)
        return ls

        #plt.scatter(points[:,0], points[:,1], alpha=0.5)
        #for l in ls:
        #    plt.plot(l[:,0], l[:,1])
        #plt.axis('equal')
        #plt.show()

        if rt is None:
            return None, None

        if len(rt) > 0:
            return rt.T # returned as (r,t)
        else:
            return None, None

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
    #np.random.seed(1245)
    #np.random.seed(1234)
    #np.random.seed(12308)
    wall_finder = WallFinderHough(dr=0.05, dt=np.deg2rad(1.8))
    #pts = [(1.0, 0.37), (1.0, 0.87)]
    pts = random_line(n=10)
    pts = np.concatenate([pts, random_line(n=10)], axis=0)
    pts = np.concatenate([pts, random_line(n=10)], axis=0)
    #print 'line', pts
    rs, ts = hough = wall_finder(pts)
    print rs, ts
    #cv2.namedWindow('acc', cv2.WINDOW_NORMAL)
    #cv2.imshow('acc', wall_finder.acc_/wall_finder.acc_.max())# / np.max(wall_finder.acc_))
    #while True:
    #    k = cv2.waitKey(10)
    #    if k == 27:
    #        break

if __name__ == "__main__":
    main()
