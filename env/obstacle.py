import numpy as np
from .base import Simulator
from .agent import Agent


def obstacle(dt,
             time_horizon,
             max_speed,
             pref_speed):
    sim = Simulator(dt, time_horizon)
    for ox in (-2.0, 2.0):
        sim.add(Agent([ox, 0.0], [ox, 0.0], radius=1.2, max_speed=0.0, pref_speed=0.0, static=True))
    for i in range(6):
        y = i * 1.3 - 3.2
        sim.add(Agent([-9, y], [9, -y], radius=0.5, max_speed=max_speed, pref_speed=pref_speed, static=False))
    return sim
