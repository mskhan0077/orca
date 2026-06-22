import numpy as np
from .agent import Agent
from .base import Simulator


def crossing(dt=0.1,
             time_horizon=5.0,
             max_speed=2.0,
             pref_speed=1.4):
    sim = Simulator(dt, time_horizon)
    for i in range(5):
        y = i * 1.5 - 3
        sim.add(Agent([-9, y], [9, y], radius=0.5, max_speed=max_speed, pref_speed=pref_speed, static=False))
        sim.add(Agent([9, y + 0.75], [-9, y + 0.75], radius=0.5, max_speed=max_speed, pref_speed=pref_speed, static=False))
    return sim