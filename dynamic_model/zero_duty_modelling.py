import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import pandas as pd
import utils

class VehicleDynamicsEstimator:
    def __init__(self, mass=1.0, dt=0.01):
        self.mass = mass
        self.dt = dt
        self.vmax = utils.v_max  # vmax for constraint

    def unpack_params(self, params):
        """
        Convert optimizer params [C0, C1, offset] into [C0, C1, C2]
        with C2 constrained as:
          C2 = - (C0 * vmax + C1) / vmax^2 + offset^2
        Ensures C2 >= -(C0*vmax + C1)/vmax^2 strictly.
        """
        C0, C1, offset = params
        C2 = - 2*(C0 * self.vmax + C1) / (self.vmax ** 2) + offset ** 2
        return C0, C1, C2

    def global_to_vehicle_frame(self, X_dot, Y_dot, theta):
        vx = X_dot * np.cos(theta) + Y_dot * np.sin(theta)
        vy = -X_dot * np.sin(theta) + Y_dot * np.cos(theta)
        return vx, vy
    
    def longitudinal_force_model(self, vx, coeffs):
        C0, C1, CdA_rho = coeffs
        Frx = -C0 * vx - C1 - (CdA_rho * vx**2) / 2
        return Frx
    
    def simulate_longitudinal_dynamics(self, vx0, coeffs, time_steps):
        vx_sim = np.zeros(time_steps)
        vx_sim[0] = vx0
        for i in range(1, time_steps):
            Frx = self.longitudinal_force_model(vx_sim[i-1], coeffs)
            vx_dot = Frx / self.mass
            vx_sim[i] = vx_sim[i-1] + vx_dot * self.dt
        return vx_sim
    
    def objective_function(self, params, vx_measured, vx0, time_steps):
        C0, C1, C2 = self.unpack_params(params)
        coeffs = [C0, C1, C2]
        vx_sim = self.simulate_longitudinal_dynamics(vx0, coeffs, time_steps)
        min_len = min(len(vx_sim), len(vx_measured))
        error = np.mean((vx_sim[:min_len] - vx_measured[:min_len])**2)
        return error
    
    def estimate_coefficients(self, X_dot, Y_dot, theta, initial_guess= utils.initial_guess_duty0):
        vx_measured = []
        vy_measured = []
        for i in range(len(X_dot)):
            vx, vy = self.global_to_vehicle_frame(X_dot[i], Y_dot[i], theta[i])
            vx_measured.append(vx)
            vy_measured.append(vy)
        vx_measured = np.array(vx_measured)
        
        time_steps = len(vx_measured)
        vx0 = vx_measured[0]
        
        # Bounds: C0, C1 positive; offset >= 0
        bounds = [(0.001, 1000), (-0.01, 0.01), (0, 10)]
        
        result = minimize(
            self.objective_function,
            initial_guess,
            args=(vx_measured, vx0, time_steps),
            method='L-BFGS-B',
            bounds=bounds
        )
        
        return result, vx_measured, vy_measured

    def plot_results(self, vx_measured, vy_measured, params, time_array=None):
        if time_array is None:
            time_array = np.arange(len(vx_measured)) * self.dt
        
        C0, C1, C2 = self.unpack_params(params)
        coeffs_estimated = [C0, C1, C2]
        
        vx_sim = self.simulate_longitudinal_dynamics(
            vx_measured[0], coeffs_estimated, len(vx_measured)
        )
        
        print(f"Estimated Coefficients:")
        print(f"C0 (Linear drag): {C0:.6f}")
        print(f"C1 (Constant resistance): {C1:.6f}")
        print(f"C2 (Aerodynamic drag, constrained): {C2:.6f}")
        print('----------')
        print(f"Final RMSE: {np.sqrt(np.mean((vx_measured - vx_sim)**2)):.6f}")
        
        
        plt.figure(figsize=(12, 6))
        
        plt.subplot(1, 2, 1)
        plt.plot(time_array, vx_measured, 'b-', label='Measured', linewidth=2)
        plt.plot(time_array, vx_sim, 'r--', label='Simulated', linewidth=2)
        plt.xlabel('Time (s)')
        plt.ylabel('Longitudinal Velocity (m/s)')
        plt.title('Velocity Comparison')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(1, 2, 2)
        plt.plot(time_array, vx_measured - vx_sim, 'g-', linewidth=2)
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity Error (m/s)')
        plt.title('Simulation Error')
        plt.grid(True)
        
        plt.tight_layout()
        
        plt.figure(figsize=(12, 6))
        plt.plot(time_array, vy_measured)
        plt.xlabel('Time (s)')
        plt.ylabel('Lateral Velocity (m/s)')
        plt.title('Lateral Velocity')
        plt.grid()
        plt.show()
        
    def coeffs_to_params(self, C0, C1, C2):
        """
        Given C0, C1, C2, find the corresponding params [C0, C1, offset]
        where offset = sqrt(C2 + (C0*vmax + C1)/vmax^2)
        This converts physical coefficients back to optimizer params.
        """
        inside_sqrt = C2 + 2*(C0 * self.vmax + C1) / (self.vmax ** 2)
        if inside_sqrt < 0:
            raise ValueError("Given C2 violates constrainqt and cannot be represented.")
        offset = np.sqrt(inside_sqrt)
        print("The value of offset is: ", offset)
        print('----------')
        return [C0, C1, offset]

if __name__ == "__main__":
    try:
        MODE = 'TEST'

        data = pd.read_csv('test_2.csv')
        
        t_raw = data['time'].values
        vx_global = data['vx'].values
        vy_global = data['vy'].values
        yaw = data['yaw'].values
        
        time = t_raw - t_raw[0]
        dt_calculated = np.mean(np.diff(time))
        print(f"Calculated dt: {dt_calculated:.6f} seconds")
        
        estimator = VehicleDynamicsEstimator(mass=utils.mass, dt=dt_calculated)
        
        if MODE == 'EVAL':
            result, vx_measured, vy_measured = estimator.estimate_coefficients(
                vx_global, vy_global, yaw
            )
            
            if result.success:
                print(f"\nOptimization successful!")
                estimator.plot_results(vx_measured, vy_measured, result.x, time)
            else:
                print(f"Optimization failed: {result.message}")
        elif MODE == 'TEST':

            C0_test, C1_test, C2_test = utils.C0, utils.C1, utils.C2
            params = estimator.coeffs_to_params(C0_test, C1_test, C2_test)
            vx_measured, vy_measured = estimator.global_to_vehicle_frame(vx_global, vy_global, yaw)
            estimator.plot_results(vx_measured=vx_measured, vy_measured=vy_measured, params=params)

        else:
            print("ERROR -- check mode")
            
    except FileNotFoundError:
        print("Error: 'test_2.csv' file not found!")
        print("Please ensure the CSV file is in the same directory as this script.")
    except KeyError as e:
        print(f"Error: Missing column {e} in CSV file!")
        print("Expected columns: 'time', 'vx', 'vy', 'yaw'")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
