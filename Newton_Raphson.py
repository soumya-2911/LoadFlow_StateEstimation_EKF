import json
import numpy as np
import pandas as pd
from scipy.linalg import sqrtm
import os
from dataclasses import dataclass

class Newton_Raphson:
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.bus_matrix1,self.line,self.shunt=self.load_power_system_data()
        self.bus_matrix = np.array(self.bus_matrix1, dtype=float)
        self.bus = self.BusData(
        No   = self.bus_matrix[:, 0].astype(int),
        Type = self.bus_matrix[:, 1].astype(int),
        Vm   = self.bus_matrix[:, 2].astype(float),  # initial prediction
        ang  = self.bus_matrix[:, 3].astype(float),  # initial prediction
        Pg   = self.bus_matrix[:, 4].astype(float),
        Qg   = self.bus_matrix[:, 5].astype(float),
        Pd   = self.bus_matrix[:, 6].astype(float),
        Qd   = self.bus_matrix[:, 7].astype(float)
    )


    @dataclass
    class BusData:
        No:   np.ndarray
        Type: np.ndarray          # 0-PQ, 1-Slack, 2-PV
        Vm:   np.ndarray
        ang:  np.ndarray
        Pd:   np.ndarray
        Qd:   np.ndarray
        Pg:   np.ndarray
        Qg:   np.ndarray
        
        #Shorthand properties
        @property
        def nbus(self) -> int:
            return len(self.No)

    def load_power_system_data(self):
        # Correct way to build path
        bus_json_path = os.path.join(self.data_dir, "bus.json")
        line_json_path = os.path.join(self.data_dir, "data_line.json")
        # shunt_json_path = os.path.join(self.data_dir, "shunt.json")

        # Now open each file
        with open(bus_json_path, 'r', encoding='utf-8') as f:
            bus_matrix = json.load(f)
        with open(line_json_path, 'r', encoding='utf-8') as f:
            line = json.load(f)
        # with open(shunt_json_path, 'r', encoding='utf-8') as f:
        #     shunt = json.load(f)
        shunt = np.empty((0, 2))  # Assuming no shunt data for now

        return bus_matrix, line, shunt

    def form_ybus(self, nbus, line_data, shunt_data=np.empty((0, 2))):
        """Return complex nbus×nbus Y-bus admittance matrix."""
        Y = np.zeros((nbus, nbus), dtype=complex)
        for frm, to, R, X, B2, tap in line_data:
            frm, to, tap = int(frm) - 1, int(to) - 1, float(tap)
            z, y, y_sh = complex(R, X), 1/complex(R, X), 1j*B2
            Y[frm, frm] += y/(tap**2) + y_sh
            Y[to,  to]  += y            + y_sh
            Y[frm, to]  += -y/tap
            Y[to,  frm] += -y/tap
        # shunts
        for bus, jB in shunt_data:
            Y[int(bus) - 1, int(bus) - 1] += jB
        return Y
    
    def power_injections(v, a, G, B):
        """Vectorised real/reactive injections at all buses."""
        # outer products build V_i V_k terms once
        VV  = np.outer(v, v)
        dth = a[:, None] - a                     # Δθ_ik matrix
        cos, sin = np.cos(dth), np.sin(dth)
        P = np.sum(VV*(G*cos + B*sin), axis=1)
        Q = np.sum(VV*(G*sin - B*cos), axis=1)
        return P, Q

    def line_flows(v, a, G, B, frm, to):
        """Real/reactive MW/Mvar from frm→to for every line index."""
        dth = a[frm] - a[to]
        P = -(v[frm]**2)*G[frm, to] + v[frm]*v[to]*(G[frm,to]*np.cos(dth) + B[frm,to]*np.sin(dth))
        Q =  (v[frm]**2)*B[frm, to] + v[frm]*v[to]*(G[frm,to]*np.sin(dth) - B[frm,to]*np.cos(dth))
        return P, Q
    
    # ---------------------------------------------------------------------------
    # Newton-Raphson power flow
    # ---------------------------------------------------------------------------
    def newton_raphson(self, Y: np.ndarray,
                    tol: float = 1e-4, maxiter: int = 80,
                    basemva: float = 100.0):
        """Full Newton-Raphson solver in polar form."""

        # Flat start if zero magnitude
        

        delta = np.deg2rad(self.bus.ang,dtype=float)
        Vm = np.where(self.bus.Vm <= 0, 1.0, self.bus.Vm)
        V = Vm * (np.cos(delta) + 1j * np.sin(delta))

        P = (self.bus.Pg - self.bus.Pd) / basemva
        Q = (self.bus.Qg - self.bus.Qd) / basemva

        slack = self.bus.Type == 1
        pv    = self.bus.Type == 2
        pq    = self.bus.Type == 0

        def mismatches(Vm, delta):
            V = Vm * (np.cos(delta) + 1j * np.sin(delta))
            S_calc = V * np.conj(Y @ V)
            Pm = P - S_calc.real
            Qm = Q - S_calc.imag
            return Pm, Qm, V

        for it in range(1, maxiter + 1):
            Pm, Qm, V = mismatches(Vm, delta)

            # Assemble Jacobian in sparse blocks
            n = self.bus.nbus
            G = Y.real
            B = Y.imag

            # Helper lambdas
            def dP_ddelta(i, j):
                if i == j:
                    return -Q[i] - (Vm[i] ** 2) * B[i, i]
                return Vm[i] * Vm[j] * (G[i, j] * np.sin(delta[i] - delta[j]) -
                                        B[i, j] * np.cos(delta[i] - delta[j]))

            def dP_dV(i, j):
                if i == j:
                    return P[i] / Vm[i] + G[i, i] * Vm[i]
                return Vm[i] * (G[i, j] * np.cos(delta[i] - delta[j]) +
                                B[i, j] * np.sin(delta[i] - delta[j]))

            def dQ_ddelta(i, j):
                if i == j:
                    return P[i] - (Vm[i] ** 2) * G[i, i]
                return -Vm[i] * Vm[j] * (G[i, j] * np.cos(delta[i] - delta[j]) +
                                        B[i, j] * np.sin(delta[i] - delta[j]))

            def dQ_dV(i, j):
                if i == j:
                    return Q[i] / Vm[i] - B[i, i] * Vm[i]
                return Vm[i] * (G[i, j] * np.sin(delta[i] - delta[j]) -
                                B[i, j] * np.cos(delta[i] - delta[j]))

            # Index maps
            pv_pq = np.where(~slack)[0]
            pq_only = np.where(pq)[0]
            mP = len(pv_pq)
            mQ = len(pq_only)

            # Jacobian sub-matrices
            J11 = np.zeros((mP, mP))
            J12 = np.zeros((mP, mQ))
            J21 = np.zeros((mQ, mP))
            J22 = np.zeros((mQ, mQ))

            for a, i in enumerate(pv_pq):
                for b, j in enumerate(pv_pq):
                    J11[a, b] = dP_ddelta(i, j)

            for a, i in enumerate(pv_pq):
                for b, j in enumerate(pq_only):
                    J12[a, b] = dP_dV(i, j)

            for a, i in enumerate(pq_only):
                for b, j in enumerate(pv_pq):
                    J21[a, b] = dQ_ddelta(i, j)

            for a, i in enumerate(pq_only):
                for b, j in enumerate(pq_only):
                    J22[a, b] = dQ_dV(i, j)

            # Compose and solve
            mismatch = np.hstack((Pm[pv_pq], Qm[pq_only]))
            J = np.block([[J11, J12],
                        [J21, J22]])

            DX = np.linalg.solve(J, mismatch)

            # Update
            delta[pv_pq] += DX[:mP]
            Vm[pq_only]  += DX[mP:]

            maxerr = abs(mismatch).max()
            if maxerr < tol:
                break
        else:
            print("WARNING: Newton-Raphson did not converge.")

        Vm_final = Vm
        ang_final = np.rad2deg(delta)
        return Vm_final, ang_final, it, V

    def print_bus_table(self, Vm: np.ndarray, ang: np.ndarray,
                    method: str, iters: int):
        head = (
            f"\nPower-flow solution – {method} ({iters} iterations)\n"
            " Bus |   |V|   |  Angle  |   Pd   |   Qd   |   Pg   |   Qg   |\n"
            "---------------------------------------------------------------"
        )
        print(head)
        for k in range(self.bus.nbus):
            print(f"{self.bus.No[k]:>4} | {Vm[k]:6.3f} | {ang[k]:8.3f} |"
                f"{self.bus.Pd[k]:7.2f} |{self.bus.Qd[k]:8.2f} |"
                f"{self.bus.Pg[k]:7.2f} |{self.bus.Qg[k]:8.2f} |")
        print("")

    def solve_Newton_Raphson(self):
        Ybus=self.form_ybus(self.bus.nbus,self.line,self.shunt)
        Vm, ang, iters, V = self.newton_raphson( Ybus,basemva=100)
        # Update bus matrix
        self.bus_matrix[:,2]=Vm
        self.bus_matrix[:,3]=ang
        self.bus_matrix[:,4]=self.bus.Pg
        self.bus_matrix[:,5]=self.bus.Qg
        # Update bus data
        self.print_bus_table(Vm,ang,"Newton Raphson",iters)
        return Ybus, Vm, ang


if __name__ == "__main__":
    data_dir = os.path.join(os.path.dirname(__file__), "state_est_utility")
    obj = Newton_Raphson(data_dir)
    obj.solve_Newton_Raphson()


# YOUR PV and Slack bus are the Generator Busses

# Bus and line data json format

''' 
    Bus labels

1) Bus number
    A unique integer identifier for the bus (1–14).

2.) Bus type

    0 = PQ (load) bus
    1 = Slack (swing) bus

    2 = PV (generator) bus



3) Voltage magnitude |V| (p.u.)
    initial guess of The per-unit voltage magnitude at that bus .

4) Voltage angle θ (degrees)
    initial Guess of The bus voltage phase angle in degrees (0 ° for the slack bus) .

5) P<sub>G> (p.u.)
    Real power generation injection at the bus (zero for pure loads).

6) Q<sub>G> (p.u.)
    Reactive power generation injection (zero for pure loads).

7) P<sub>L> (p.u.)
    Real power load demand at the bus (zero for pure generators).

8) Q<sub>L> (p.u.)
    Reactive power load demand at the bus.'''

'''
    Line Labels

    1) From
    2) To
    3) resistance
    4) reactance
    5) susceptance
    6) tap
    '''