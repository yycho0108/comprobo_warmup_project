import numpy as np
#from sklearn.cluster import KMeans
#from sklearn.preprocessing import scale 
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA

import utils as U

class PersonDetector(object):
    def __init__(self,
            min_x, max_x,
            min_y, max_y):
        self.min_x_ = min_x
        self.max_x_ = max_x
        self.min_y_ = min_y
        self.max_y_ = max_y

        # TODO : hardcoded & not used
        self.min_c_ = 0.15 * (2.0/3.0) # min cluster size
        self.max_c_ = 0.45 * (3.0/2.0) # max cluster size

    def __call__(self, rq):

        r, q = rq.T

        # format data & apply range limits
        xy = U.rq2xy(rq)
        x, y = xy.T
        x_mask = (self.min_x_ <= x) & (x <= self.max_x_)
        y_mask = (self.min_y_ <= y) & (y <= self.max_y_)
        xy = xy[x_mask & y_mask]

        if np.size(xy) <= 0:
            return False, None

        # clustering
        #xy_f = StandardScaler().fit_transform(xy)
        db = DBSCAN(eps=0.1, min_samples=3).fit(xy)

        labels = db.labels_
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        if n_clusters > 0:
            # simple filter by max variance
            # (avoid detecting walls, etc.)
            pca = PCA(n_components=2)
            cls = []
            for i in range(n_clusters):
                cl = xy[labels==i]
                pca.fit(cl)
                smax = np.sqrt(np.max(pca.explained_variance_))
                if smax < 0.1:
                    cls.append(cl)

            n_clusters = len(cls)
            print n_clusters
            if n_clusters == 1:
                # handle 1-case specially
                c0 = cls[0]
                p0 = np.mean(c0, axis=0)

                # add artificial division
                p_l = p0 + [0, 0.01] 
                p_r = p0 - [0, 0.01]

                return True, (p0,p0)
            elif n_clusters >= 2:
                # sort by distance
                cls = [xy[labels==i] for i in range(n_clusters)]
                pts = [np.mean(e, axis=0) for e in cls]
                pts.sort(key=np.linalg.norm)
                p0, p1 = pts[:2] # closest 2

                if np.linalg.norm(np.subtract(p0, p1)) > 1.0:
                    return False, None
                else:
                    return True, (p0, p1)

        #    return True, (c0,c1)

        #if n_clusters == 2:
        #    # left-right feet
        #    c0 = xy[labels == 0]
        #    c1 = xy[labels == 1]
        #    return True, (c0,c1)
        #elif n_clusters == 1:
        #    # no left-right distinction, but try initialization anyways
        #    c0 = xy[labels == 0]
        #    return True, (c0,c0)
        ## failure
        return False, None




