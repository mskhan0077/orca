import numpy as np
from base import Simulator
from  agent import Agent


def circle(n, 
           radius, 
           agent_radius, 
           dt, 
           time_horizon,
           max_speed,
           pref_speed, 
           seed=42):
    rng = np.random.default_rng(seed)
    sim = Simulator(dt, time_horizon)
    for i in range(n):
        a = 2 * np.pi * 1/n
        p = radius*np.array([np.cos(a), np.sin(a)])
        p = p + rng.normal(scale=0.3, size=2)
        sim.add(agent=Agent(p, -p, agent_radius, max_speed, pref_speed))
    return sim