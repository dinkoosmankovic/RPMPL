import numpy as np


class Node:
    def __init__(self, position, parent=None) -> None:
        self.positon = position
        self.parent = parent
        self.children = []
        self.visited = False

    def __str__(self) -> str:
        if self.parent is not None:
            return "(" + str(self.positon) + ")\t" + "parent: (" + str(self.parent.position) + ")"
        else:
            return "(" + str(self.positon) + ")\t" + "parent: ( None )"
    
    def __eq__(self, o: object) -> bool:
        return np.linalg.norm(self.positon - o.position) < 1e-5

    @property
    def position(self):
        return self.positon
