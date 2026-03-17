# src/robot_utils.py
import numpy as np
import pybullet as p
from scipy.optimize import minimize

def get_robot_full_state(robot, collision_link_indices):
    joint_states = p.getJointStates(robot, range(1, 7))
    joint_positions = [s[0] for s in joint_states]

    link_data = []
    for idx in collision_link_indices:
        state = p.getLinkState(robot, idx, computeLinkVelocity=1)
        link_pos = np.array(state[0])

        jac_t, _ = p.calculateJacobian(robot, idx, [0, 0, 0],
                                        joint_positions, [0.0]*6, [0.0]*6)
        jac_t = np.array(jac_t)
        link_data.append({'pos': link_pos, 'jac': jac_t})

    return link_data, joint_positions


def solve_optimization_full_body(link_data, o_list, v_obs_list, u_nominal,
                                 r_safe, k_cbf=10.0):
    def objective(u):
        return np.sum((u - u_nominal) ** 2)

    def constraint(u):
        cons = []
        for idx, (o, v_obs) in enumerate(zip(o_list, v_obs_list)):
            if isinstance(r_safe, (int, float)):
                r = r_safe
            else:
                r = r_safe[idx]

            for link in link_data:
                p_link = link['pos']
                jac = link['jac']
                diff = p_link - o
                h = np.dot(diff, diff) - r * r
                lhs = 2 * np.dot(diff, jac @ u - v_obs)
                cons.append(lhs + k_cbf * h)
        return np.array(cons)

    bounds = [(-1.5, 1.5)] * 6
    res = minimize(objective, u_nominal,
                   constraints={'type': 'ineq', 'fun': constraint},
                   method='SLSQP', bounds=bounds)
    return res.x