# src/simulation.py
import pybullet as p
import pybullet_data
import time
import numpy as np
from . import config
from .camera import StaticTripodCamera
from .kalman import KalmanFilter3D
from .robot_utils import get_robot_full_state, solve_optimization_full_body

def setup_simulation(obs_radius=config.OBS_RADIUS):
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    robot = p.loadURDF("../../UR3/urdf/UR3.urdf", [0, 0, 0], useFixedBase=True)
    collision_link_indices = [3, 4, 5, 6]
    initial_positions = [1.0, -1.57, 1.57, -1.57, -1.57, 0]
    for i in range(6):
        p.resetJointState(robot, i + 1, initial_positions[i])

    obstacle_ids = []
    obstacle_init_pos = []
    viz_obs_ids = []

    for cfg in config.obs_configs:
        center = cfg["center"]
        color = cfg["color"]

        col = p.createCollisionShape(p.GEOM_SPHERE, radius=obs_radius)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=obs_radius, rgbaColor=color)
        obs_id = p.createMultiBody(baseMass=1.0,
                                   baseCollisionShapeIndex=col,
                                   baseVisualShapeIndex=vis,
                                   basePosition=center)
        obstacle_ids.append(obs_id)
        obstacle_init_pos.append(np.array(center))

        viz_col = p.createVisualShape(p.GEOM_SPHERE, radius=obs_radius, rgbaColor=[1, 1, 0, 0.5])
        viz_id = p.createMultiBody(baseMass=0,
                                   baseVisualShapeIndex=viz_col,
                                   basePosition=[0, 0, -1])
        viz_obs_ids.append(viz_id)

    for i in range(p.getNumJoints(robot)):
        for obs_id in obstacle_ids:
            p.setCollisionFilterPair(robot, obs_id, i, -1, enableCollision=1)

    target_pose = np.array([0.5, -0.2, 0.25])
    target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 1, 0, 0.7])
    target_id = p.createMultiBody(baseVisualShapeIndex=target_visual, basePosition=target_pose)

    viz_target_id = p.createMultiBody(
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 0.8]),
        basePosition=[0, 0, -1])

    return {
        'robot': robot,
        'collision_link_indices': collision_link_indices,
        'obstacle_ids': obstacle_ids,
        'obstacle_init_pos': obstacle_init_pos,
        'target_pos': target_pose,
        'viz_obs_ids': viz_obs_ids,
        'viz_target_id': viz_target_id,
        'obs_radius': obs_radius,
        'obs_configs': config.obs_configs,
    }


def run_experiment(step_size, vision_noise, use_dr=False, **kwargs):
    max_steps = kwargs.get('max_steps', config.MAX_STEPS)
    success_threshold = kwargs.get('success_threshold', config.SUCCESS_THRESHOLD)
    collision_threshold = kwargs.get('collision_threshold', config.COLLISION_THRESHOLD)
    gain = kwargs.get('gain', config.GAIN)
    dt = kwargs.get('dt', config.DT)
    k_cbf = kwargs.get('k_cbf', config.K_CBF)
    obs_radius = kwargs.get('obs_radius', config.OBS_RADIUS)

    sim_data = setup_simulation(obs_radius=obs_radius)

    robot = sim_data['robot']
    collision_link_indices = sim_data['collision_link_indices']
    obstacle_ids = sim_data['obstacle_ids']
    obstacle_init_pos = sim_data['obstacle_init_pos']
    target_pos = sim_data['target_pos']
    viz_obs_ids = sim_data['viz_obs_ids']
    viz_target_id = sim_data['viz_target_id']
    obs_configs = sim_data['obs_configs']

    num_obs = len(obstacle_ids)

    cam = StaticTripodCamera(obs_radius,
                             *config.CAMERA_POS,
                             noise_std=vision_noise)

    kf_list = [KalmanFilter3D(dt=dt*5, pos_std=vision_noise) for _ in range(num_obs)]

    obs_true = [pos.copy() for pos in obstacle_init_pos]
    obs_est_pos = [pos.copy() for pos in obstacle_init_pos]
    obs_est_vel = [np.zeros(3) for _ in range(num_obs)]
    obs_uncertainty_radius = [0.0] * num_obs

    step_counter = 0

    while True:
        if step_counter >= max_steps:
            return 'timeout'

        for i in range(num_obs):
            center = np.array(obs_configs[i]["center"])
            walk_range = obs_configs[i]["range"]
            obs_true[i] += np.random.uniform(-step_size, step_size, size=3)
            lower = center - walk_range
            upper = center + walk_range
            obs_true[i] = np.clip(obs_true[i], lower, upper)
            p.resetBasePositionAndOrientation(obstacle_ids[i], obs_true[i], [0, 0, 0, 1])

        collision_occurred = False
        for obs_id in obstacle_ids:
            closest = p.getClosestPoints(bodyA=robot, bodyB=obs_id, distance=collision_threshold)
            if len(closest) > 0:
                collision_occurred = True
                break
        if collision_occurred:
            return 'collision'

        link_data, _ = get_robot_full_state(robot, collision_link_indices)
        ee_pos = link_data[-1]['pos']

        if np.linalg.norm(ee_pos - target_pos) < success_threshold:
            return 'success'

        if step_counter % 5 == 0:
            for i in range(num_obs):
                z = cam.get_object_pos_vision(obstacle_ids[i])
                if z is not None:
                    kf_list[i].predict()
                    kf_state = kf_list[i].update(z)
                    obs_est_pos[i] = kf_state[:3]
                    obs_est_vel[i] = kf_state[3:]

                if use_dr:
                    cov = kf_list[i].get_pos_cov()
                    sigma = np.sqrt(np.trace(cov))
                    obs_uncertainty_radius[i] = 1.2 * sigma

                p.resetBasePositionAndOrientation(viz_obs_ids[i], obs_est_pos[i], [0, 0, 0, 1])

            p.resetBasePositionAndOrientation(viz_target_id, target_pos, [0, 0, 0, 1])

        if use_dr:
            r_safe = [obs_radius + 0.05 + unc for unc in obs_uncertainty_radius]
        else:
            r_safe = obs_radius + 0.05

        v_task = gain * (target_pos - ee_pos)
        u_nom = np.linalg.pinv(link_data[-1]['jac']) @ v_task
        q_vel = solve_optimization_full_body(link_data,
                                             obs_est_pos,
                                             obs_est_vel,
                                             u_nom,
                                             r_safe,
                                             k_cbf=k_cbf)

        for i in range(6):
            p.setJointMotorControl2(robot, i + 1,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=q_vel[i],
                                    force=150)

        p.stepSimulation()
        time.sleep(dt)
        step_counter += 1