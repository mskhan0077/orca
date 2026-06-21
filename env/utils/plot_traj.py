import matplotlib.pyplot as plt
import numpy as np

def plot_trajectories(sim, history, path):
    fig, ax = plt.subplots(figsize=(7, 7))
    cmap = plt.cm.viridis(np.linspace(0, 1, len(sim.agents)))
    for i, agent in enumerate(sim.agents):
        if agent.static:
            ax.add_patch(plt.Circle(history[0, i], agent.radius,
                                    color="0.3", zorder=3))
            continue
        ax.plot(history[:, i, 0], history[:, i, 1], color=cmap[i], lw=1.2, alpha=0.8)
        ax.add_patch(plt.Circle(history[0, i], agent.radius * 0.5,
                                color=cmap[i], alpha=0.4))
        ax.add_patch(plt.Circle(history[-1, i], agent.radius,
                                color=cmap[i], zorder=4))
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.set_title("ORCA agent trajectories")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)