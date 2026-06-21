import numpy as np
from agent import Agent
from base import Simulator


def crossing(dt, 
             time_horizon):
    sim = Simulator(dt, time_horizon)
    for i in range(5):
        y = i * 1.5 - 3
        sim.add(Agent([-9, y], [9, y], radius=0.5))
        sim.add(Agent([9, y + 0.75], [-9, y + 0.75], radius=0.5))
    return sim