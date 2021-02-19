class Event:
    ENV_CHANGE = 1
    KEY_PRESSED = 2

    def __init__(self, _type, key=None):
        self.type = _type
        self.key = key


class Key:
    UP = 1
    RIGHT = 2
    DOWN = 3
    LEFT = 4
