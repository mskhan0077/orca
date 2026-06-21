import numpy as np
from env.base import Simulator
from env.utils import animate, plot_traj
from env import circle, crossing, obstacle

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
time_horizon = 1.0
max_steps = 350
n = 5


# sim = circle.circle(n=n,
#                     radius=12,
#                     agent_radius=1.0,
#                     dt=dt,
#                     time_horizon=time_horizon,
#                     max_speed=2.0,
#                     pref_speed=1.5)
# sim = crossing.crossing(dt=dt,
#                         time_horizon=time_horizon,
#                         max_speed=1.0,
#                         pref_speed=0.5)
sim = obstacle.obstacle(dt=dt,
                        time_horizon=time_horizon,
                        max_speed=1.0,
                        pref_speed=0.7)

history = run(sim,
              max_steps=max_steps)

animate.animate(sim=sim,
                history=history,
                path="obstacle.gif")
plot_traj.plot_trajectories(sim=sim,
                            history=history,
                            path="obstacle.png")















