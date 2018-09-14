import numpy as np

class PID(object):
    """ Simple PID controller """
    def __init__(self,
            kp=1.0, ki=0.0, kd=0.0,
            max_u=2.6, max_i=None):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        if max_i is None:
            if np.abs(ki) < 1e-6:
                max_i = 0.0
            else:
                max_i = np.abs(max_u / ki)
        self._max_u = max_u
        self._max_i = max_i
        self._prv_err = None
        self._net_err = 0.0

    def reset(self):
        self._prv_err = None
        self._net_err = 0.0

    def __call__(self, err, dt=0.0):
        if dt <= 0:
            return 0

        # compute control output
        if self._prv_err is None:
            # protection against initial large d
            self._prv_err = err
        p = self._kp * err
        i = self._ki * self._net_err
        d = self._kd * (self._prv_err - err) / dt

        # control output
        u = (p + i + d)
        u = np.clip(u, -self._max_u, self._max_u)

        # save persistent data
        self._prv_err = err
        self._net_err += err * dt
        self._net_err = np.clip(self._net_err, -self._max_i, self._max_i)
        return u
