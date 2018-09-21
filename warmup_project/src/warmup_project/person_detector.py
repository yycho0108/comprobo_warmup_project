import numpy as np
#from sklearn.cluster import KMeans
#from sklearn.preprocessing import scale 
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

class PersonDetector(object):
    def __init__(self,
            min_x, max_x,
            min_y, max_y):
        self.min_x_ = min_x
        self.max_x_ = max_x
        self.min_y_ = min_y
        self.max_y_ = max_y

        # TODO : hardcoded
        self.min_c_ = 0.15 * (2.0/3.0) # min cluster size
        self.max_c_ = 0.45 * (3.0/2.0) # max cluster size

    @staticmethod
    def rq2xy(rq):
        # (n,2) -> (n,2)
        r, q = rq.T
        x = r * np.cos(q)
        y = r * np.sin(q)
        return np.stack([x,y], axis=-1)

    def __call__(self, rq):
        r, q = rq.T

        # format data & apply range limits
        xy = self.rq2xy(rq)
        x, y = xy.T
        x_mask = (self.min_x_ <= x) & (x <= self.max_x_)
        y_mask = (self.min_y_ <= y) & (y <= self.max_y_)
        xy = xy[x_mask & y_mask]

        # clustering
        xy_f = StandardScaler().fit_transform(xy)
        db = DBSCAN(eps=0.2, min_samples=3).fit(xy_f)

        labels = db.labels_
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        if n_clusters == 2:
            # left-right feet
            c0 = xy[labels == 0]
            c1 = xy[labels == 1]
            return True, (c0,c1)
        elif n_clusters == 1:
            # no left-right distinction, but try initialization anyways
            c0 = xy[labels == 0]
            return True, (c0,c0)

        return False, None
