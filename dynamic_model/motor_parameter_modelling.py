import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import lsq_linear
import utils

class MotorModelFitter:
    def __init__(self, filename, C0, C1, C2, mass=3.3185, mode='EVAL'):
        self.filename = filename
        self.C0 = C0
        self.C1 = C1
        self.C2 = C2
        self.mass = mass
        self.mode = mode  # 'fit' or 'test'

        # Load and preprocess data
        self.data = self._load_data()
        self.time = self.data['time'] - self.data['time'][0]
        self.dt = np.mean(np.diff(self.time))
        self.vx_body = self._convert_to_body_frame()

        if self.mode == 'TEST':
            self.Cm0 = utils.Cm0
            self.Cm1 = utils.Cm1

    def _load_data(self):
        df = pd.read_csv(self.filename)
        if 'duty' not in df.columns:
            raise ValueError("CSV must contain a 'duty' column when using time-varying duty cycle.")
        return df

    def _convert_to_body_frame(self):
        vx = self.data['vx'].values
        vy = self.data['vy'].values
        yaw = self.data['yaw'].values
        return vx * np.cos(yaw) + vy * np.sin(yaw)

    def fit(self):
        if self.mode != 'EVAL':
            print("Skipping fitting — running in 'test' mode using known Cm0, Cm1.")
            return self.Cm0, self.Cm1

        vx = self.vx_body
        ax = np.gradient(vx, self.dt)
        D = self.data['duty'].values

        # Build linear system A @ [Cm0, Cm1] = b
        A = np.vstack((D, -D * vx)).T
        b = self.mass * ax + self.C0 * vx + self.C1 + 0.5 * self.C2 * vx**2

        result = lsq_linear(A, b, bounds=([-np.inf, -np.inf], [np.inf, np.inf]))
        self.Cm0, self.Cm1 = result.x
        self.success = result.success
        self.message = result.message
        return self.Cm0, self.Cm1

    def simulate(self):
        Cm0, Cm1 = self.Cm0, self.Cm1
        D = self.data['duty'].values
        vx_sim = np.zeros_like(self.vx_body)
        vx_sim[0] = self.vx_body[0]

        for i in range(1, len(self.time)):
            vx = vx_sim[i - 1]
            duty = D[i]
            Frx = (Cm0 - Cm1 * vx) * duty - self.C0 * vx - self.C1 - 0.5 * self.C2 * vx**2
            ax = Frx / self.mass
            vx_sim[i] = vx + ax * self.dt

        return vx_sim

    def plot_fit(self):
        vx_sim = self.simulate()
        plt.figure(figsize=(10, 5))
        plt.plot(np.array(self.time), np.array(self.vx_body), label='Measured vx (body)')
        plt.plot(np.array(self.time), np.array(vx_sim), '--', label='Simulated vx')
        plt.xlabel("Time (s)")
        plt.ylabel("vx (m/s)")
        plt.title(f"Mode: {self.mode.upper()} — Measured vs Simulated vx")
        plt.legend()
        plt.grid()
        plt.show()

    def summary(self):
        print(f"File: {self.filename}")
        print(f"Known: C0 = {self.C0}, C1 = {self.C1}, C2 = {self.C2}")
        print(f"Mode: {self.mode}")
        print(f"Cm0 = {self.Cm0:.6f}, Cm1 = {self.Cm1:.6f}")
        if self.mode == 'fit':
            print(f"Optimization success: {self.success}")
            print(f"Message: {self.message}")
        print("----------")

# Example usage:
if __name__ == "__main__":
    C0 = utils.C0
    C1 = utils.C1
    C2 = utils.C2
    filename = "duty_04.csv"

    # To fit parameters:

    fitter = MotorModelFitter(filename, C0, C1, C2, mode='EVAL')
    fitter.fit()
    fitter.summary()
    fitter.plot_fit()
