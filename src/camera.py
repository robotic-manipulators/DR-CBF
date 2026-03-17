# src/camera.py
import numpy as np
import pybullet as p

class StaticTripodCamera:
    def __init__(self, obs_radius, x=0.3, y=-0.8, z=0.3, noise_std=0.001):
        self.camera_pos = np.array([x, y, z])
        self.noise_std = noise_std
        self.obs_radius = obs_radius
        actual_cam_pos = self.camera_pos + np.array([0, 0, 0.04])
        self.width, self.height = 320, 240
        target_view_pos = actual_cam_pos + np.array([0, 1, 0])
        self.view_matrix = p.computeViewMatrix(actual_cam_pos, target_view_pos, [0, 0, 1])
        self.proj_matrix = p.computeProjectionMatrixFOV(60, self.width / self.height, 0.05, 5.0)

    def get_object_pos_vision(self, target_body_id):
        _, _, _, depth_img, seg_img = p.getCameraImage(
            self.width, self.height, self.view_matrix, self.proj_matrix,
            renderer=p.ER_TINY_RENDERER)

        if seg_img is None or seg_img.ndim != 2:
            return None

        mask = np.where(seg_img == target_body_id)
        if len(mask[0]) == 0:
            return None

        u, v = int(np.mean(mask[1])), int(np.mean(mask[0]))
        z_buf = depth_img[v, u]

        z_dist = (2.0 * 0.05 * 5.0) / (5.0 + 0.05 - (2.0 * z_buf - 1.0) * (5.0 - 0.05))

        tan_fov = np.tan(np.deg2rad(30))
        x_c = (2.0 * u / self.width - 1.0) * z_dist * (self.width / self.height) * tan_fov
        y_c = z_dist
        z_c = -(2.0 * v / self.height - 1.0) * z_dist * tan_fov

        surface_point = self.camera_pos + np.array([x_c, y_c, z_c + 0.04])

        direction = surface_point - self.camera_pos
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction = direction / norm
            sphere_center = surface_point + direction * self.obs_radius
        else:
            sphere_center = surface_point

        noisy_center = sphere_center + np.random.normal(0, self.noise_std, 3)
        return noisy_center