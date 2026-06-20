import numpy as np


class Agent:
    def __init__(self,
                 pos: float,
                 goal: float,
                 radius: float,
                 max_speed: float,
                 pref_speed: float,
                 static: bool):
        
        self.pos = np.asarray(pos, dtype=float)
        self.goal = np.asarray(goal, dtype=float)
        self.radius = radius
        self.max_speed = max_speed
        self.pref_speed = pref_speed
        self.static = static
        self.vel = np.zeros(2)
        self.new_vel = np.zeros(2)

    def preferred_velocity(self):
        if self.static:
            return np.zeros(2)
        
        to_goal = self.goal - self.pos
        d = np.linalg.norm(to_goal)
        if d < 1e-3:
            return np.zeros(2)
        
        return to_goal/d * min(self.pref_speed, d)