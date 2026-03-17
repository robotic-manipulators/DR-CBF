# src/kalman.py
import numpy as np

class KalmanFilter3D:
    def __init__(self, dt, pos_std=0.05, vel_std=0.015):
        self.x = np.zeros(6)
        self.F = np.eye(6)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = dt
        self.H = np.zeros((3, 6))
        self.H[:3, :3] = np.eye(3)
        self.P = np.eye(6) * 1.0
        self.Q = np.eye(6) * (vel_std ** 2)
        self.R = np.eye(3) * (pos_std ** 2)

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def update(self, measurement):
        y = measurement - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = self.P - K @ self.H @ self.P
        return self.x

    def get_pos_cov(self):
        return self.P[:3, :3].copy()