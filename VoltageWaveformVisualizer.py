
"""
Voltage Waveform Visualization for Dynamic State Estimation
===========================================================

This script creates realistic AC voltage waveforms from dynamic state estimation results,
showing how voltage magnitudes and phases change during system transients.

Author: AI Assistant  
Date: 2025
"""

import numpy as np
import matplotlib.pyplot as plt

import warnings
warnings.filterwarnings('ignore')

class VoltageWaveformVisualizer:
    def __init__(self, system_frequency=60.0, base_voltage_kV=230.0):
        self.f0 = system_frequency; self.w0 = 2*np.pi*self.f0; self.Vbase = base_voltage_kV

    def create_ac_waveforms(self, Vmag, Vang, t_sim, buses_to_plot=None, cycles_per_step=3):
        nT, nB = Vmag.shape
        if buses_to_plot is None: buses_to_plot = [1,2,3]
        dt_sim = t_sim[1]-t_sim[0] if len(t_sim)>1 else 1.0/self.f0
        dt_ac = 1/(self.f0*120); n_ac = max(60, int(cycles_per_step*self.f0*dt_sim/dt_ac))
        t_blocks=[]; wave_blocks=[]; env_hi=[]; env_lo=[]
        for k in range(nT-1):
            t0, t1 = t_sim[k], t_sim[k+1]; tac = np.linspace(t0, t1, n_ac); block={}
            for b in buses_to_plot:
                i=b-1; vm=np.linspace(Vmag[k,i], Vmag[k+1,i], n_ac)*self.Vbase; va=np.linspace(Vang[k,i], Vang[k+1,i], n_ac)
                block[f'Bus_{b}']= vm*np.cos(self.w0*tac+va)
            t_blocks.append(tac); wave_blocks.append(block)
            up={}; lo={}
            for b in buses_to_plot:
                i=b-1; vmax=max(Vmag[k,i], Vmag[k+1,i])*self.Vbase; up[f'Bus_{b}']=vmax; lo[f'Bus_{b}']=-vmax
            env_hi.append(up); env_lo.append(lo)
        return {'buses':buses_to_plot,'sim_time':t_sim,'ac_time_arrays':t_blocks,'voltage_waveforms':wave_blocks,'envelope_upper':env_hi,'envelope_lower':env_lo}

    def plot_voltage_waveforms_2d(self, wf, figsize=(12,8)):
        buses = wf['buses']; fig, axes = plt.subplots(len(buses),1,figsize=figsize,sharex=True)
        if len(buses)==1: axes=[axes]
        for idx,b in enumerate(buses):
            ax=axes[idx]
            for tac,blk in zip(wf['ac_time_arrays'], wf['voltage_waveforms']): ax.plot(tac, blk[f'Bus_{b}'], linewidth=0.8)
            envt=wf['sim_time'][:-1]; up=[e[f'Bus_{b}'] for e in wf['envelope_upper']]; lo=[e[f'Bus_{b}'] for e in wf['envelope_lower']]
            ax.plot(envt, up, '--', linewidth=1.5); ax.plot(envt, lo, '--', linewidth=1.5)
            ax.set_ylabel(f'Bus {b} (kV)'); ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)'); fig.suptitle('AC Voltage Waveforms (3‑Bus)', fontsize=12); fig.tight_layout(); return fig, axes
