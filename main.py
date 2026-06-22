import numpy as np
from env.base import Simulator
from env.utils import animate, plot_traj
from env import circle, crossing, obstacle


def run(sim,
        max_steps,
        dt,
        time_horizon):
    history = [np.array([a.pos.copy() for a in sim.agents])]
    for _ in range(max_steps):
        sim.step()
        history.append(np.array([a.pos.copy() for a in sim.agents]))
        if sim.all_reached():
            break
    return np.array(history)


dt = 0.1
time_horizon = 10.0
max_steps = 350
n = 5


# sim = circle.circle(n=n,
#                     radius=10,
#                     agent_radius=1.0)
sim = crossing.crossing()
# sim = obstacle.obstacle()

history = run(sim,
              max_steps=max_steps,
              dt=dt,
              time_horizon=time_horizon)

animate.animate(sim=sim,
                history=history,
                path="plots/crossing.gif")

plot_traj.plot_trajectories(sim=sim,
                            history=history,
                            path="plots/crossing.png")















