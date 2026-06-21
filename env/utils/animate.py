import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

def animate(sim, history, path):
    fig, ax = plt.subplots(figsize=(7, 7))
    lim = np.abs(history).max() + 1.5
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
    ax.set_aspect("equal"); ax.grid(alpha=0.2)
    cmap = plt.cm.viridis(np.linspace(0, 1, len(sim.agents)))
    circles = []
    for i, agent in enumerate(sim.agents):
        c = plt.Circle(history[0, i], agent.radius,
                       color="0.3" if agent.static else cmap[i],
                       alpha=1.0 if agent.static else 0.85)
        ax.add_patch(c); circles.append(c)
 
    def update(f):
        for i, c in enumerate(circles):
            c.center = history[f, i]
        ax.set_title(f"ORCA  —  step {f}")
        return circles
 
    anim = FuncAnimation(fig, update, frames=len(history), interval=50, blit=False)
    anim.save(path, writer=PillowWriter(fps=20))
    plt.close(fig)