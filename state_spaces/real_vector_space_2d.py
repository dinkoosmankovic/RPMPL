import random
import numpy as np
from .state_space import StateSpace

class RealVectorSpace2D(StateSpace):
    def __init__(self, obstacles = None) -> None:
        super().__init__(obstacles)
        self.num_checks = 10

    def get_qrand(self, xmin, xmax, ymin, ymax):
        return [random.uniform(xmin, xmax), random.uniform(ymin, ymax)]

    def get_qnew(self, qnear, qrand, eps):
        a = qnear
        b = qrand
        return a + eps*(b-a)/np.linalg.norm(b-a)

    def collide(self, q, obs):
        p = obs["p"]
        r = obs["r"]
        return np.linalg.norm(p-q) < r

    def is_valid(self, q1, q2):
        if self.obs is None:
            return True
        for i in range(self.num_checks):
            q_e = self.get_qnew(q1, q2, i/self.num_checks)
            for o in self.obs:
                if self.collide(q_e, o):
                    return False
        return True