import numpy as np
from dataclasses import dataclass

EPS = 1e-5


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
                    direction = -np.array([rel_pos[0] * leg + rel_pos[1] * r,
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


def linear_program_1(lines, line_no, radius, opt_velocity, direction_opt):
    """Optimise along a single line `line_no`, subject to lines[0:line_no] and the
    max-speed circle. Returns (success, result)."""
    line = lines[line_no]
    dot = line.point @ line.direction
    discriminant = dot * dot + radius * radius - (line.point @ line.point)
    if discriminant < 0.0:
        return False, None                 # max-speed circle misses this line entirely
 
    sqrt_d = np.sqrt(discriminant)
    t_left = -dot - sqrt_d
    t_right = -dot + sqrt_d
 
    for i in range(line_no):
        denom = det(line.direction, lines[i].direction)
        numer = det(lines[i].direction, line.point - lines[i].point)
        if abs(denom) <= EPS:              # parallel lines
            if numer < 0.0:
                return False, None
            continue
        t = numer / denom
        if denom >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return False, None
 
    if direction_opt:
        t = t_right if (opt_velocity @ line.direction) > 0 else t_left
    else:
        t = line.direction @ (opt_velocity - line.point)
        t = max(t_left, min(t_right, t))
    return True, line.point + t * line.direction
 
 
def linear_program_2(lines, radius, opt_velocity, direction_opt):
    """Find the velocity in all half-planes closest to opt_velocity (or, if
    direction_opt, farthest in that direction). Returns (fail_index, result):
    fail_index == len(lines) means full success."""
    if direction_opt:
        result = opt_velocity * radius
    elif (opt_velocity @ opt_velocity) > radius * radius:
        result = normalize(opt_velocity) * radius
    else:
        result = opt_velocity.copy()
 
    for i, line in enumerate(lines):
        if det(line.direction, line.point - result) > 0.0:   # constraint i violated
            temp = result.copy()
            ok, result = linear_program_1(lines, i, radius, opt_velocity, direction_opt)
            if not ok:
                return i, temp
    return len(lines), result
 
 
def linear_program_3(lines, num_obst_lines, begin_line, radius, result):
    """Infeasible fallback: relax agent constraints to minimise the maximum
    penetration, keeping the first `num_obst_lines` (static obstacles) hard."""
    distance = 0.0
    for i in range(begin_line, len(lines)):
        if det(lines[i].direction, lines[i].point - result) > distance:
            proj = list(lines[:num_obst_lines])
            for j in range(num_obst_lines, i):
                determinant = det(lines[i].direction, lines[j].direction)
                if abs(determinant) <= EPS:
                    if lines[i].direction @ lines[j].direction > 0:
                        continue           # same direction, redundant
                    point = 0.5 * (lines[i].point + lines[j].point)
                else:
                    point = lines[i].point + (
                        det(lines[j].direction, lines[i].point - lines[j].point)
                        / determinant) * lines[i].direction
                proj.append(Line(point, normalize(lines[j].direction - lines[i].direction)))
 
            temp = result.copy()
            opt = np.array([-lines[i].direction[1], lines[i].direction[0]])
            fail, result = linear_program_2(proj, radius, opt, True)
            if fail < len(proj):
                result = temp              # numerical safety: keep previous best
            distance = det(lines[i].direction, lines[i].point - result)
    return result

def compute_new_velocity(agent, lines, num_static):
    pref = agent.preferred_velocity()
    fail, result = linear_program_2(lines, agent.max_speed, pref, False)
    if fail < len(lines):
        result = linear_program_3(lines, num_static, fail, agent.max_speed, result)
    return result