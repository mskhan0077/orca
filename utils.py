import numpy as np
from dataclasses import dataclass


@dataclass
class Line:
    point: np.ndarray
    direction: np.ndarray

def det(a, b):
    return a[0]*b[1] - a[1]*b[0]

def normalize(v):
    n = np.linalg.norm(v)
    return v/n if n>1e-5 else np.zeros(2)

def compute_orca_lines(agent, others, time_horizon, dt):
    lines = []
    inv_th = 1/time_horizon
    num_static = sum(1 for o in others if o.static)

    for other in others:
        rel_pos = other.pos - agent.pos
        rel_vel = agent.vel - other.vel
        dist_sq = rel_pos @ rel_pos
        r = agent.radius + other.radius
        r_sq = r*r
        responsibility = 1 if other.static else 0.5

        if dist_sq > r_sq:
            w = rel_vel - inv_th*rel_pos
            w_len_sq = w @ w
            dot1 = w @ rel_pos

            if dot1 < 0 and dot1*dot1 > r_sq*w_len_sq:
                w_len = np.sqrt(w_len_sq)
                unit_w = w/w_len
                direction = np.array([unit_w[1], -unit_w[0]])
                u = (r * inv_th - w_len)*unit_w
            else:
                leg = np.sqrt(max(dist_sq - r_sq, 0.0))
                if det(rel_pos, w) > 0:
                    direction = np.array([rel_pos[0] * leg - rel_pos[1] * r,
                                          rel_pos[0] * r + rel_pos[1] * leg]) / dist_sq
                else:
                    direction = np.array([rel_pos[0] * leg - rel_pos[1] * r,
                                          -rel_pos[0] * r + rel_pos[1] * leg]) / dist_sq
                u = (rel_vel @ direction) * direction - rel_vel
        
        else:
            inv_dt = 1/dt
            w = rel_vel - inv_dt*rel_pos
            w_len = np.linalg.norm(w)
            unit_w = w/w_len if w_len>1e-5 else np.array([1.0, 0.0])
            direction = np.array([unit_w[1], -unit_w[0]])
            u = (r*inv_dt - w_len)*unit_w
        
        lines.append(Line(agent.vel + responsibility*u, direction))
    return lines, num_static
