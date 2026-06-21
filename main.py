import numpy as np
from env.base import Simulator

def run(sim,
        max_steps):
    history = [np.array([a.pos.copy() for a in sim.agents])]
    for _ in range(max_steps):
        sim.step()
        history.append(np.array([a.pos.copy() for a in sim.agents]))
        if sim.all_reached():
            break
    return np.array(history)


dt = 0.1
time_horizon = 5.0
neighbour_dist = 10.0
max_steps = 300

sim = Simulator(dt=dt,
                time_horizon=time_horizon,
                neighbour_dist=neighbour_dist)

history = run(sim,
              max_steps=max_steps)













