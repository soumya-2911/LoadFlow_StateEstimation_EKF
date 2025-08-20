# Dynamic State Estimator for a 3‑Bus System (EKF)
import numpy as np
from scipy.linalg import inv
import warnings
import os
import Newton_Raphson
warnings.filterwarnings('ignore')

class DynamicStateEstimator_nBus:
    def __init__(self, system_data: dict):
        self.n_bus = int(system_data['n_bus'])
        self.n_gen = int(system_data['n_gen'])
        self.gen_buses = np.array(system_data['gen_buses'], dtype=int)
        self.Y_bus = system_data['Y_bus'].astype(complex)
        self.dt = float(system_data.get('dt', 0.01))

        self.H = np.array(system_data.get('H', [5.0]*self.n_gen), dtype=float)
        self.D = np.array(system_data.get('D', [2.0]*self.n_gen), dtype=float)
        self.Xd = np.array(system_data.get('Xd', [0.3]*self.n_gen), dtype=float)
        self.E_internal_mag = np.array(system_data.get('E_internal_mag', [1.05]*self.n_gen), dtype=float)

        self.V_nr_mag = np.array(system_data.get('V_nr_mag', [1.04, 1.01, 0.98]), dtype=float)
        self.V_nr_ang = np.array(system_data.get('V_nr_ang', [0.0, -5.0, -7.0]), dtype=float) * np.pi/180.0

        self.P_load_base = np.array(system_data.get('P_load_base', [0.0, 0.0, 0.90]), dtype=float)
        self.Q_load_base = np.array(system_data.get('Q_load_base', [0.0, 0.0, 0.30]), dtype=float)

        self.n_states = 2*self.n_gen
        self.n_meas = 2*self.n_bus + 2*self.n_gen

        self.f0 = float(system_data.get('f0', 60.0))
        self.omega_s = 2*np.pi*self.f0
        self.M = 2 * self.H / self.omega_s

        sigma_process = float(system_data.get('sigma_process', 1e-4))
        sigma_meas = float(system_data.get('sigma_measurement', 1e-3))
        sigma_init = float(system_data.get('sigma_initial', 1e-2))
        self.Q = (sigma_process**2) * np.eye(self.n_states)
        self.R = (sigma_meas**2) * np.eye(self.n_meas)
        self.P = (sigma_init**2) * np.eye(self.n_states)

        self.x_hat = np.zeros(self.n_states)
        self.x_pred = self.x_hat.copy()
        self.load_scale = np.ones(self.n_bus)

    def set_load_scale(self, new_scale):
        new_scale = np.asarray(new_scale, dtype=float)
        if new_scale.shape != (self.n_bus,):
            raise ValueError(f"load_scale must be shape ({self.n_bus},), got {new_scale.shape}")
        self.load_scale = np.clip(new_scale, 0.0, 10.0)

    def init_from_nr(self, bus_angles_deg=None):
        """
        Initialize EKF states from Newton-Raphson power flow solution.
        """
        if bus_angles_deg is None:
            bus_angles_deg = np.degrees(self.V_nr_ang)
        
        # Initialize rotor angles from generator bus voltage angles
        for i, bus_idx in enumerate(self.gen_buses):
            self.x_hat[i] = np.radians(bus_angles_deg[bus_idx - 1])
        
        # Initialize rotor speed deviations to zero
        self.x_hat[self.n_gen:] = np.zeros(self.n_gen)
        
        # Update prediction state
        self.x_pred = self.x_hat.copy()
        
        # Option 1: Convert to list for formatting
        rotor_angles_deg = np.degrees(self.x_hat[:self.n_gen]).tolist()
        speed_deviations = self.x_hat[self.n_gen:].tolist()
        
        print(f"Initialized from NR:")
        print(f"  Rotor angles: {[f'{angle:.2f}' for angle in rotor_angles_deg]} deg")
        print(f"  Speed deviations: {[f'{speed:.4f}' for speed in speed_deviations]} pu")
        
        # Option 2: Use numpy array2string for controlled formatting
        print(f"Initialized from NR:")
        print(f"  Rotor angles: {np.array2string(np.degrees(self.x_hat[:self.n_gen]), precision=2, separator=', ')} deg")
        print(f"  Speed deviations: {np.array2string(self.x_hat[self.n_gen:], precision=4, separator=', ')} pu")



    def state_transition_model(self, x, u, pm):
        delta = x[:self.n_gen]
        omega = x[self.n_gen:]
        Pe = self._compute_electrical_power(delta)
        delta_dot = omega - self.omega_s
        omega_dot = (1.0 / self.M) * (pm - Pe - self.D*(omega - self.omega_s))
        x_next = np.zeros_like(x)
        x_next[:self.n_gen] = delta + delta_dot * self.dt
        x_next[self.n_gen:] = omega + omega_dot * self.dt
        return x_next

    def _compute_electrical_power(self, delta):
        E_internal = self.E_internal_mag * np.exp(1j*delta)
        Y_mod = self.Y_bus.copy().astype(complex)
        for i, b in enumerate(self.gen_buses):
            Y_mod[b-1, b-1] += 1.0 / (0.01 + 1j*self.Xd[i])
        I_inj = np.zeros(self.n_bus, dtype=complex)
        for i, b in enumerate(self.gen_buses):
            Yg = 1.0 / (0.01 + 1j*self.Xd[i])
            I_inj[b-1] += E_internal[i] * Yg
        I_load = self._compute_load_currents()
        I_total = I_inj + I_load
        V_bus = np.linalg.solve(Y_mod, I_total)
        Pe = np.zeros(self.n_gen)
        for i, b in enumerate(self.gen_buses):
            Yg = 1.0 / (0.01 + 1j*self.Xd[i])
            I_g = E_internal[i]*Yg - V_bus[b-1]*Yg
            S_g = V_bus[b-1] * np.conj(I_g)
            Pe[i] = np.real(S_g)
        return Pe

    def compute_bus_voltages_from_states(self, x):
        
        delta = x[:self.n_gen]
        # Compute internal generator voltages
        E_internal = self.E_internal_mag * np.exp(1j*delta)
        # Compute modified Y bus matrix
        Y_mod = self.Y_bus.copy().astype(complex)
        # Add generator admittances
        for i, b in enumerate(self.gen_buses):
            Y_mod[b-1, b-1] += 1.0 / (0.01 + 1j*self.Xd[i])
        # Compute generator currents
        I_gen = np.zeros(self.n_bus, dtype=complex)
        # Compute admittance for each generator
        for i, b in enumerate(self.gen_buses):
            Yg = 1.0 / (0.01 + 1j*self.Xd[i])
            I_gen[b-1] = E_internal[i]*Yg
        I_load = self._compute_load_currents()
        I_total = I_gen + I_load
        V_bus = np.linalg.solve(Y_mod, I_total)
        return np.abs(V_bus), np.angle(V_bus)

    def _compute_load_currents(self):
        V_nr = self.V_nr_mag * np.exp(1j*self.V_nr_ang) # load voltages given by NR
        S_load = (self.P_load_base*self.load_scale) + 1j*(self.Q_load_base*self.load_scale) # load complex power
        with np.errstate(divide='ignore', invalid='ignore'):
            I_load = np.conj(S_load / V_nr)
        I_load[~np.isfinite(I_load)] = 0.0
        return I_load

    def measurement_model(self, x):
        vmag, vang = self.compute_bus_voltages_from_states(x)
        Pe = self._compute_electrical_power(x[:self.n_gen])
        Qe = np.zeros(self.n_gen)
        h = np.zeros(self.n_meas)
        h[:self.n_bus] = vmag
        h[self.n_bus:2*self.n_bus] = vang
        h[2*self.n_bus:2*self.n_bus+self.n_gen] = Pe
        h[2*self.n_bus+self.n_gen:] = Qe
        return h

    def _dPe_ddelta_num(self, delta):
        Pe0 = self._compute_electrical_power(delta)
        J = np.zeros((self.n_gen, self.n_gen))
        eps = 1e-5
        for i in range(self.n_gen):
            d = delta.copy()
            d[i] += eps
            Pe1 = self._compute_electrical_power(d)
            J[:, i] = (Pe1 - Pe0)/eps
        return J

    def compute_jacobian_F(self, x, u, pm):
        F = np.eye(self.n_states)
        F[:self.n_gen, self.n_gen:] = self.dt*np.eye(self.n_gen)
        dPe_ddelta = self._dPe_ddelta_num(x[:self.n_gen])
        F[self.n_gen:, :self.n_gen] = -(self.dt/self.M.reshape(-1,1)) * dPe_ddelta
        F[self.n_gen:, self.n_gen:] = np.eye(self.n_gen) - (self.dt/self.M)*np.diag(self.D)
        return F

    def compute_jacobian_H(self, x):
        H = np.zeros((self.n_meas, self.n_states))
        for i, b in enumerate(self.gen_buses):
            H[self.n_bus + (b-1), i] = 1.0
        dPe_ddelta = self._dPe_ddelta_num(x[:self.n_gen])
        H[2*self.n_bus:2*self.n_bus+self.n_gen, :self.n_gen] = dPe_ddelta
        return H

    def predict(self, u, pm):
        self.x_pred = self.state_transition_model(self.x_hat, u, pm)
        self.x_pred =np.clip(self.x_pred, -1*np.pi, np.pi)

        F = self.compute_jacobian_F(self.x_hat, u, pm) #Jacobian of State Transition model
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        h_pred = self.measurement_model(self.x_pred)
        y = z - h_pred
        H = self.compute_jacobian_H(self.x_pred) #Jacobian of Measurement model
        S = H @ self.P @ H.T + self.R

        # Compute Kalman gain
        try:
            K = self.P @ H.T @ inv(S)
        except Exception:
            K = self.P @ H.T @ np.linalg.pinv(S)

        # Update state estimate and covariance
        self.x_hat = self.x_pred + K @ y
        self.x_hat = np.clip(self.x_hat, -1*np.pi, np.pi) # Ensure angles are wrapped
        I = np.eye(self.n_states)
        # Update process covariance
        self.P = (I - K @ H) @ self.P

    def estimate(self, measurements, mechanical_power, control_input=None):
        if control_input is None:
            control_input = np.zeros(self.n_gen)
        self.predict(control_input, mechanical_power)
        self.update(measurements)
        return {
            'rotor_angles': self.x_hat[:self.n_gen],
            'rotor_speeds': self.x_hat[self.n_gen:],
            'state_vector': self.x_hat,
            'covariance': self.P,
            'predicted_measurements': self.measurement_model(self.x_hat)
        }

def create_3bus_system():
    n_bus = 3; n_gen = 2; gen_buses = np.array([1,2])

    #Y Matrix
    y12 = -1j/0.20; y13 = -1j/0.40; y23 = -1j/0.25
    Y = np.zeros((3,3), dtype=complex)
    Y[0,1] = Y[1,0] = y12
    Y[0,2] = Y[2,0] = y13
    Y[1,2] = Y[2,1] = y23
    for k in range(3):
        Y[k,k] = -(Y[k,:].sum() - Y[k,k])

    #Generator parameters
    H = np.array([1.0, 0.02]); D = np.array([0.02, 0.03]); Xd = np.array([0.30, 0.25])
    Eint = np.array([1.08, 1.05])

    #Newton Raphson Stedy states
    V_nr_mag = np.array([1.04, 1.01, 0.98])
    V_nr_ang_deg = np.array([0.0, -5.0, -7.0])
    P_load = np.array([0.00, 0.00, 0.90]); Q_load = np.array([0.00, 0.00, 0.30])

    return {
        'n_bus': n_bus,
        'n_gen': n_gen,
        'gen_buses': gen_buses,
        'Y_bus': Y,
        'H': H,
        'D': D,
        'Xd': Xd,
        'E_internal_mag': Eint,
        'V_nr_mag': V_nr_mag,
        'V_nr_ang': V_nr_ang_deg,
        'P_load_base': P_load,
        'Q_load_base': Q_load,
        'dt': 0.005,
        'f0': 60.0,
        'sigma_process': 3e-4,
        'sigma_measurement': 1e-3,
        'sigma_initial': 1e-2
    }




def create_14bus_system(NR_obj):
    """
    Create IEEE 14-bus system data for EKF dynamic state estimation
    Based on standard IEEE 14-bus test case with Newton-Raphson results
    """
    n_bus = 14
    n_gen = 5  # Generators at buses 1, 2, 3, 6, 8
    gen_buses = np.array([1, 2, 3, 6, 8])

    Y, V_nr_mag, V_nr_ang_deg = NR_obj.solve_Newton_Raphson()  # Ensure NR solution is computed

    # Load data (converted to per unit on 100 MVA base)
    P_load = NR_obj.bus_matrix[:,6]

    Q_load = NR_obj.bus_matrix[:,7]

    # Generator parameters (realistic values for different generator types)
    H = np.array([0.7, 3.3, 3.8, 2.1, 1.0])  # Inertia constants (s)
    D = np.array([4.5, 5.4, 5.04, 3.3, 3.3])  # Damping coefficients
    Xd = np.array([0.25, 0.22, 0.20, 0.18, 0.16])  # Synchronous reactances
    Eint = np.array([1.08, 1.06, 1.04, 1.09, 1.11])  # Internal voltages
    
   
    
    return {
        'n_bus': n_bus,
        'n_gen': n_gen,
        'gen_buses': gen_buses,
        'Y_bus': Y,
        'H': H,
        'D': D,
        'Xd': Xd,
        'E_internal_mag': Eint,
        'V_nr_mag': V_nr_mag,
        'V_nr_ang': V_nr_ang_deg,
        'P_load_base': P_load,
        'Q_load_base': Q_load,
        'dt': 0.005,
        'f0': 60.0,
        'sigma_process': 2e-4,
        'sigma_measurement': 8e-4,
        'sigma_initial': 5e-3
    }


if __name__ == '__main__':
    data_dir = os.path.join(os.path.dirname(__file__), "state_est_utility")
    obj = Newton_Raphson(data_dir)
    sys = create_14bus_system(obj); est = DynamicStateEstimator_nBus(sys); est.init_from_nr()
    pm = np.array([1.05, 1.03, 1.01, 1.5, 0.5])
    z = est.measurement_model(est.x_hat) + np.random.normal(0, 1e-4, size=2*sys['n_bus']+2*sys['n_gen'])
    res = est.estimate(z, pm)
    print('EKF sanity check ok. Rotor speed (Hz):', res['rotor_speeds']/(2*np.pi))


