
class StateSpace:
    def __init__(self, obstacles = None) -> None:
        self.obs = obstacles

    def update_obstacles(self, obstacles) -> None:
        self.obs = obstacles

    def is_valid(self, q):
        pass

    def get_qrand(self):
        pass

    def get_qnew(self):
        pass