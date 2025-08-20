# Disturbance-Enabled Dynamic Analysis (3‑Bus)
import numpy as np
import matplotlib.pyplot as plt
import os
import Newton_Raphson
from dynamic_state_estimator_nbus import DynamicStateEstimator_nBus, create_14bus_system
import VoltageWaveformVisualizer

class run_dynamic_EKF:
    def __init__(self):
        self.NR_obj = Newton_Raphson.Newton_Raphson(os.path.join(os.path.dirname(__file__), "state_est_utility"))
        self.system = create_14bus_system(self.NR_obj); self.est = DynamicStateEstimator_nBus(self.system); self.est.init_from_nr()
        self.visualizer = VoltageWaveformVisualizer(system_frequency=self.system.get('f0',60.0))
        self.f0=self.system.get('f0',60.0)

    # def apply_disturbance(self, t, dist_type, pm_base):
    #     pm = pm_base.copy(); scale = self.est.load_scale.copy(); desc = 'No Disturbance'
    #     if dist_type=='step_load':
    #         if 0.8 <= t <= 1.8: scale[2]*=1.10; desc = '+10% Load @ Bus3'
    #     elif dist_type=='generator_trip':
    #         if t >= 1.2: pm[1]=0.0; desc='Trip Gen2 (Pm2=0)'
    #     elif dist_type=='oscillatory_load':
    #         if t >= 0.8:
    #             f=2.0; amp=0.10*np.exp(-(t-0.8)/2.0); scale[2]*=1.0+amp*np.sin(2*np.pi*f*(t-0.8)); desc='Oscillatory Load @ Bus3'
    #     elif dist_type=='three_phase_fault':
    #         if 1.0 <= t <= 1.10: scale[1]*=1.50; scale[2]*=1.35; desc='3Φ Fault Emulation @ Bus2 (100 ms)'
    #         elif 1.10 < t <= 2.0: rec=1.0-0.5*np.exp(-(t-1.10)/0.4); scale*=rec; desc='Post‑Fault Recovery'
    #     elif dist_type=='pm_pulse':
    #         if 1.0 <= t <= 1.3: pm[0]+=0.15; desc='Pm Pulse on Gen1 (+0.15 pu, 300 ms)'
    #     return pm, scale, desc

    def apply_disturbance(self, t, dist_type, pm_base):
        pm = pm_base.copy()
        scale = self.est.load_scale.copy()
        desc = 'No Disturbance'
        
        if dist_type == 'step_load':
            if 0.8 <= t <= 1.8: 
                scale[2] *= 1.10
                desc = '+10% Load @ Bus3'
        
        elif dist_type == 'generator_trip':
            if t >= 1.2: 
                pm[24] = 0.0
                desc = 'Trip Gen2 (Pm2=0)'
        
        elif dist_type == 'oscillatory_load':
            if t >= 0.8:
                f = 2.0
                amp = 0.23 * np.exp(-(t-0.8)/2.0)
                scale[2] *= 1.0 + amp * np.sin(2*np.pi*f*(t-0.8))
                scale[1] *= 1.0 + amp * np.sin(2*np.pi*f*(t-0.8))
                desc = 'Oscillatory Load @ Bus3'
        
        elif dist_type == 'three_phase_fault':
            if 1.0 <= t <= 1.10: 
                scale[1] *= 1.50
                scale[2] *= 1.35
                desc = '3Φ Fault Emulation @ Bus2 (100 ms)'
            elif 1.10 < t <= 2.0: 
                rec = 1.0 - 0.5 * np.exp(-(t-1.10)/0.4)
                scale *= rec
                desc = 'Post‑Fault Recovery'
        
        elif dist_type == 'pm_pulse':
            if 1.0 <= t <= 1.3: 
                pm += 0.65
                desc = 'Pm Pulse on Gen1 (+0.65 pu, 300 ms)'

        # NEW HARMONIC DISTORTION CASES
        elif dist_type == 'nonlinear_load_harmonics':
            if t >= 0.8:
                # Typical nonlinear load harmonics: 3rd, 5th, 7th harmonics
                h3_mag = 0.15  # 15% of fundamental
                h5_mag = 0.10  # 10% of fundamental  
                h7_mag = 0.05  # 5% of fundamental
                
                # Triple-frequency (3rd harmonic) - zero sequence
                h3_component = h3_mag * np.sin(3 * 2*np.pi*self.f0 * t)
                
                # 5th harmonic - negative sequence
                h5_component = h5_mag * np.sin(5 * 2*np.pi*self.f0 * t + np.pi/3)
                
                # 7th harmonic - positive sequence  
                h7_component = h7_mag * np.sin(7 * 2*np.pi*self.f0 * t - np.pi/4)
                
                # Apply harmonic load distortion to Bus 2
                harmonic_distortion = 1.0 + h3_component + h5_component + h7_component
                scale[1] = max(0.1, harmonic_distortion)  # Prevent negative loads
                desc = 'Nonlinear Load Harmonics @ Bus2 (3rd,5th,7th)'
        
        elif dist_type == 'vfd_harmonics':
            # Variable Frequency Drive harmonics
            if t >= 0.74:
                # VFDs typically generate 6k±1 harmonics (5th, 7th, 11th, 13th, etc.)
                h5 = 0.010 * np.cos(5 * 2*np.pi*self.f0 * t + np.pi/6)
                h7 = 0.04 * np.sin(7 * 2*np.pi*self.f0 * t - np.pi/4)
                h11 = 0.06 * np.sin(11 * 2*np.pi*self.f0 * t + np.pi/2)
                h13 = 0.08 * np.sin(13 * 2*np.pi*self.f0 * t - np.pi/3)
                # h3 = 0.080 * np.sin(3 * 2*np.pi*self.f0 * t + np.pi/8)
                vfd_distortion = 1.0 + h5 + h7 + h11 + h13
                scale[2] *= max(0.1, vfd_distortion)
                scale[1] *= max(0.1, vfd_distortion)
                scale[5] *= max(0.1, vfd_distortion)
                desc = 'VFD Harmonics @ Bus3 (5th,7th,11th,13th)'

        # elif dist_type == 'vfd_harmonics_14bus':
        #     # Stronger VFD harmonics distortion for a 14-bus system
        #     if t >= 1:  # introduce distortion after 0.5s
        #         # Typical VFD harmonics (6k±1 order), made stronger
        #         h5  = 0.18 * np.cos(5  * 2*np.pi*self.f0 * t + np.pi/6)     # 5th
        #         h7  = 0.2 * np.sin(7  * 2*np.pi*self.f0 * t - np.pi/4)     # 7th
        #         h11 = 0.15 * np.sin(11 * 2*np.pi*self.f0 * t + np.pi/2)     # 11th
        #         h13 = 0.04 * np.sin(13 * 2*np.pi*self.f0 * t - np.pi/3)     # 13th
        #         h3  = 0.030 * np.sin(3  * 2*np.pi*self.f0 * t + np.pi/8)     # triplen harmonic (3rd)
        #         h9  = 0.08 * np.sin(9  * 2*np.pi*self.f0 * t - np.pi/6)     # 9th harmonic

        #         # Composite distortion signal
        #         vfd_distortion = 1.0 + h3 + h5 + h7 + h9 + h11 + h13

        #         # Apply scaling to multiple buses (choose based on IEEE 14-bus loads)
        #         scale[2]  *= max(0.05, vfd_distortion)   # Bus 3
        #         scale[3]  *= max(0.05, vfd_distortion)   # Bus 4
        #         scale[5]  *= max(0.05, vfd_distortion)   # Bus 6
        #         scale[7]  *= max(0.05, vfd_distortion)   # Bus 8
        #         scale[10] *= max(0.05, vfd_distortion)   # Bus 11

        #         desc = 'Stronger VFD Harmonics @ Buses 3,4,6,8,11 (3rd,5th,7th,9th,11th,13th)'
        elif dist_type == 'vfd_harmonics_14bus':
            # Enhanced VFD harmonics with capacitor switching transient + transient noise for 14-bus system
            
            # Initialize base distortion factors
            switching_distortion = 1.0
            vfd_harmonic_distortion = 1.0
            noise_distortion = 1.0

            # 1) Capacitor switching transient (0.5–2.5s)
            if 0.5 <= t <= 2.5:
                elapsed = t - 0.5
                # Sharp voltage dip
                dip = -0.45 * np.exp(-elapsed/0.1)
                # Oscillatory recovery components
                osc1 = 0.25 * np.exp(-elapsed/0.3) * np.cos(2*np.pi*150*elapsed + np.pi/4)
                osc2 = 0.15 * np.exp(-elapsed/0.5) * np.sin(2*np.pi*300*elapsed - np.pi/6)
                osc3 = 0.35 * np.exp(-elapsed/1.2) * np.cos(2*np.pi*75 *elapsed + np.pi/3)
                # Overshoot
                overshoot = 0.6 * np.exp(-elapsed/0.8) * np.cos(2*np.pi*60*elapsed)
                switching_distortion = 1.0 + dip + osc1 + osc2 + osc3 + overshoot

            # 2) Continuous VFD harmonics (t ≥ 1s)
            if t >= 1.0:
                h5  = 0.40 * np.cos(5  * 2*np.pi*self.f0 * t + np.pi/6)
                h7  = 0.22 * np.sin(7  * 2*np.pi*self.f0 * t - np.pi/4)
                h11 = 0.12 * np.sin(11 * 2*np.pi*self.f0 * t + np.pi/3)
                h13 = 0.08 * np.cos(13 * 2*np.pi*self.f0 * t - np.pi/6)
                h3  = 0.18 * np.sin(3  * 2*np.pi*self.f0 * t + np.pi/4)
                h9  = 0.10 * np.cos(9  * 2*np.pi*self.f0 * t - np.pi/8)
                h17 = 0.04 * np.sin(17 * 2*np.pi*self.f0 * t + np.pi/5)
                h_inter = 0.05 * np.sin(6.5 * 2*np.pi*self.f0 * t + np.pi/9)
                load_var = 1.0 + 0.2 * np.sin(0.3 * 2*np.pi * t)
                vfd_harmonic_distortion = 1.0 + (h3 + h5 + h7 + h9 + h11 + h13 + h17 + h_inter) * load_var

            # 3) Transient noise burst around switching event
            if 0.45 <= t <= 2.55:
                # Gaussian white noise filtered to high frequencies
                noise = 0.02 * np.random.normal(0, 1)  # 2% random
                # Superimpose damped random spikes
                spike = (0.05 * np.exp(-abs(t-1.5)/0.2) *
                        np.sin(2*np.pi*(np.random.uniform(100,300))*t))
                noise_distortion = 1.0 + noise + spike

            # Combined distortion factor
            combined = switching_distortion * vfd_harmonic_distortion * noise_distortion

            # Apply to buses with distance-based severity
            scale[2]  *= max(0.05, combined)           # Bus 3
            scale[3]  *= max(0.05, combined * 0.9)     # Bus 4
            scale[5]  *= max(0.05, combined * 0.85)    # Bus 6
            scale[7]  *= max(0.10, combined * 0.7)     # Bus 8
            scale[10] *= max(0.10, combined * 0.8)     # Bus 11
            scale[8]  *= max(0.10, combined * 0.6)     # Bus 9
            scale[11] *= max(0.15, vfd_harmonic_distortion * 0.5)  # Bus 12
            scale[12] *= max(0.15, vfd_harmonic_distortion * 0.4)  # Bus 13

            desc = (
                'Capacitor switching transient (0.5–2.5s) + VFD harmonics + '
                'transient noise bursts (2% white noise + damped spikes)'
            )

        elif dist_type == 'strong_multi_harmonics_14bus':
            vfd_distortion = 1.0

            # Event 1: Strong VFD harmonics after 0.5s
            if 0.5 <= t < 0.9:
                h5  = 0.023 * np.cos(5  * 2*np.pi*self.f0 * t + np.pi/6)
                h7  = 0.17 * np.sin(7  * 2*np.pi*self.f0 * t - np.pi/4)
                h11 = 0.019 * np.sin(11 * 2*np.pi*self.f0 * t + np.pi/2)
                h13 = 0.032 * np.sin(13 * 2*np.pi*self.f0 * t - np.pi/3)
                vfd_distortion += h5 + h7 + h11 + h13 + np.random.normal(0, 0.7)
                scale[2]  *= vfd_distortion
                scale[5]  *= vfd_distortion
                scale[4]  *= vfd_distortion
                desc = 'Strong VFD Harmonics @ Bus3, Bus6'

            # Event 2: Triplen harmonics after 0.9s
            if 0.9 <= t < 1.3:
                h3  = 0.20 * np.sin(3 * 2*np.pi*self.f0 * t + np.pi/8)
                h9  = 0.15 * np.sin(9 * 2*np.pi*self.f0 * t - np.pi/6)
                vfd_distortion += h3 + h9
                scale[3]  *= vfd_distortion
                scale[10] *= vfd_distortion
                desc = 'Triplen Harmonics @ Bus4, Bus11'

            # Event 3: Interharmonics (non-integer multiples) after 1.3s
            if 1.3 <= t < 1.7:
                h_inter1 = 0.18 * np.sin(5.5 * 2*np.pi*self.f0 * t)   # 5.5th harmonic
                h_inter2 = 0.12 * np.cos(7.2 * 2*np.pi*self.f0 * t)   # 7.2th harmonic
                vfd_distortion += h_inter1 + h_inter2
                scale[8]  *= vfd_distortion
                scale[12] *= vfd_distortion
                desc = 'Interharmonics @ Bus9, Bus13'

            # Event 4: DC offset burst + notching after 1.7s
            if 1.7 <= t < 2.0:
                dc_offset = 0.3
                notch     = 0.15 * np.sign(np.sin(2*np.pi*self.f0 * t))  # square-wave like
                vfd_distortion += dc_offset + notch
                scale[2]  *= vfd_distortion
                scale[13] *= vfd_distortion
                desc = 'DC + Notch Distortion @ Bus7, Bus14'

        elif dist_type == 'arc_furnace_harmonics':
            # Arc furnace: random/chaotic harmonics with interharmonics
            if t >= 0.8:
                # Random amplitude modulation typical of arc furnaces
                random_factor = 1.0 + 0.1 * np.random.normal()
                
                # Interharmonics (non-integer multiples of fundamental)
                ih23 = 0.08 * np.sin(2.3 * 2*np.pi*self.f0 * t) * random_factor
                ih47 = 0.06 * np.sin(4.7 * 2*np.pi*self.f0 * t) * random_factor
                
                # Subharmonics (below fundamental frequency)
                sub_h = 0.05 * np.sin(0.5 * 2*np.pi*self.f0 * t) * random_factor
                
                # Standard harmonics with random variations
                h2 = 0.12 * np.sin(2 * 2*np.pi*self.f0 * t) * random_factor
                h3 = 0.18 * np.sin(3 * 2*np.pi*self.f0 * t) * random_factor
                
                arc_distortion = 1.0 + ih23 + ih47 + sub_h + h2 + h3
                scale[1] *= max(0.2, arc_distortion)
                desc = 'Arc Furnace Harmonics @ Bus2 (chaotic)'
        
        elif dist_type == 'led_lighting_harmonics':
            # LED/SMPS harmonics: odd harmonics dominant
            if t >= 0.8:
                # Time-varying THD to simulate LED driver variations
                thd_variation = 0.15 + 0.05 * np.sin(0.1 * 2*np.pi * t)
                
                h3 = thd_variation * 0.30 * np.sin(3 * 2*np.pi*self.f0 * t)
                h5 = thd_variation * 0.25 * np.sin(5 * 2*np.pi*self.f0 * t)
                h7 = thd_variation * 0.15 * np.sin(7 * 2*np.pi*self.f0 * t)
                h9 = thd_variation * 0.10 * np.sin(9 * 2*np.pi*self.f0 * t)
                
                led_distortion = 1.0 + h3 + h5 + h7 + h9
                scale[2] *= max(0.1, led_distortion)
                desc = f'LED Lighting Harmonics @ Bus3 (THD={thd_variation*100:.1f}%)'
        
        elif dist_type == 'capacitor_switching_harmonics':
            # Capacitor switching transients with harmonic resonance
            if 1.0 <= t <= 1.5:
                # Resonant frequency around 5th harmonic
                resonant_freq = 5 * self.f0
                decay_factor = np.exp(-(t-1.0)/10)  # Fast decay
                
                # Resonant oscillation with harmonics
                resonance = 0.3 * decay_factor * np.sin(2*np.pi*resonant_freq * t)
                h5_resonance = 0.2 * decay_factor * np.sin(5 * 2*np.pi*self.f0 * t)
                h7_resonance = 0.1 * decay_factor * np.sin(7 * 2*np.pi*self.f0 * t)
                
                cap_distortion = 1.0 + resonance + h5_resonance + h7_resonance
                scale[1] *= max(0.1, cap_distortion)
                desc = 'Capacitor Switching Resonance @ Bus2'
        
        elif dist_type == 'thyristor_rectifier_harmonics':
            # 6-pulse rectifier: characteristic harmonics 6k±1
            if t >= 0.8:
                # Classic 6-pulse rectifier harmonics
                h5 = 0.17 * np.sin(5 * 2*np.pi*self.f0 * t + np.pi)     # 17% (theoretical)
                h7 = 0.12 * np.sin(7 * 2*np.pi*self.f0 * t)             # 12%
                h11 = 0.08 * np.sin(11 * 2*np.pi*self.f0 * t + np.pi)   # 8%
                h13 = 0.06 * np.sin(13 * 2*np.pi*self.f0 * t)           # 6%
                
                rectifier_distortion = 1.0 + h5 + h7 + h11 + h13
                scale[2] *= max(0.1, rectifier_distortion)
                desc = '6-Pulse Rectifier Harmonics @ Bus3'
        
        elif dist_type == 'wind_farm_harmonics':
            # Wind farm with DFIG: sub/super synchronous components
            if t >= 0.8:
                # Slip-dependent harmonics (typical for DFIG)
                slip_freq = 2.0  # Hz, typical slip frequency
                
                # Sub-synchronous (f0 - slip_freq)
                sub_sync = 0.05 * np.sin(2*np.pi*(self.f0 - slip_freq) * t)
                
                # Super-synchronous (f0 + slip_freq)  
                super_sync = 0.05 * np.sin(2*np.pi*(self.f0 + slip_freq) * t)
                
                # PWM harmonics from converter (around switching frequency)
                f_switch = 2000  # 2 kHz typical switching frequency
                pwm_harmonics = 0.03 * np.sin(2*np.pi*f_switch * t) * np.sin(2*np.pi*self.f0 * t)
                
                wind_distortion = 1.0 + sub_sync + super_sync + pwm_harmonics
                scale[1] *= max(0.1, wind_distortion)
                desc = 'Wind Farm Harmonics @ Bus2 (DFIG+PWM)'
        
        return pm, scale, desc


    def run(self, T=3.0, dist_type='three_phase_fault'):
        dt=self.system['dt']; n=int(T/dt); t=np.linspace(0, T, n)
        Vmag=np.zeros((n, self.system['n_bus'])); Vang=np.zeros((n, self.system['n_bus'])) #Initialize your starting voltages
        delta=np.zeros((n, self.system['n_gen'])); omega=np.zeros((n, self.system['n_gen']))
        pm_log=np.zeros((n, self.system['n_gen'])); events=[]; pm_base=np.array([1.35, 1.03, 1.01, 1.5, 0.5])
        for k, tk in enumerate(t):
            pm, scale, desc = self.apply_disturbance(tk, dist_type, pm_base)
            self.est.set_load_scale(scale)
            if desc!='No Disturbance' and (len(events)==0 or events[-1][1]!=desc): events.append((tk, desc))
            # Compute voltage measurements
            vm, va = self.est.compute_bus_voltages_from_states(self.est.x_hat)
            z = np.zeros(self.est.n_meas)  # Measurement vector initialize

            # Update measurement vector with current estimates
            z[:self.system['n_bus']] = vm
            z[self.system['n_bus']:2*self.system['n_bus']] = va
            z[2*self.system['n_bus']:2*self.system['n_bus']+self.system['n_gen']] = pm
            z[2*self.system['n_bus']+self.system['n_gen']:] = 0.0
            z += np.random.normal(0, 1e-3, size=z.shape)

            # State estimation
            out = self.est.estimate(z, pm)
            # Update state estimates
            Vmag[k,:]=vm; Vang[k,:]=va; delta[k,:]=out['rotor_angles']; omega[k,:]=out['rotor_speeds']; pm_log[k,:]=pm


        # Plots
        fig, axes = plt.subplots(4,1, figsize=(10,10), sharex=True)
        for i in range(pm_log.shape[1]): axes[0].plot(t, pm_log[:,i]) 
        axes[0].set_ylabel('Pm (p.u.)'); axes[0].grid(True)

        for i in range(delta.shape[1]): axes[1].plot(t, np.degrees(delta[:,i]))
        axes[1].set_ylabel('Rotor Angle (deg)'); axes[1].grid(True)

        for i in range(omega.shape[1]): axes[2].plot(t, omega[:,i]/(2*np.pi))
        axes[2].axhline(60.0, linestyle='--'); axes[2].set_ylabel('Freq (Hz)'); axes[2].grid(True)

        for b in [1,2,3]: axes[3].plot(t, Vmag[:,b-1], label=f'Bus {b}')
        axes[3].axhline(1.0, linestyle='--'); axes[3].set_ylabel('|V| (p.u.)'); axes[3].grid(True); axes[3].legend()

        axes[-1].set_xlabel('Time (s)'); fig.suptitle(f'3‑Bus DSE Response: {dist_type}', fontsize=12); fig.tight_layout()

        # Voltage waveform visualization
        viz = VoltageWaveformVisualizer(system_frequency=self.system.get('f0',60.0))

        wf = viz.create_ac_waveforms(Vmag, Vang, t, buses_to_plot=[1,2,3], cycles_per_step=2)
        viz.plot_voltage_waveforms_2d(wf, figsize=(10,8))
        return {'time':t,'Vmag':Vmag,'Vang':Vang,'delta':delta,'omega':omega,'Pm':pm_log,'events':events,'waveform':wf}

if __name__ == '__main__':
    sim = run_dynamic_EKF() 
    res = sim.run(T=3.0, dist_type='oscillatory_load')
    # res = sim.run(T=3.0, dist_type='nonlinear_load') 
    print('Events:', res['events'])
    plt.show()
