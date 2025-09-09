# Parallel_dynamic_EKF.py
import os, csv, numpy as np
from multiprocessing import Process, Queue
from run_dynamic_EKF import run_dynamic_EKF   # import your EKF simulator class

CSV_FILE = "ekf_results.csv"


def simulator_process(q: Queue, T_total=20.0, T_chunk=2.0, dist_type='oscillatory_load'):
    sim = run_dynamic_EKF()
    dt = sim.system['dt']  # simulation time step
    n_total = int(T_total / dt)
    n_chunk = int(T_chunk / dt)

    print(f"[Simulator] Running {T_total}s in chunks of {T_chunk}s")

    # Prepare arrays
    pm_base = np.array([1.35, 1.03, 1.01, 1.5, 0.5])
    Vmag_log, Vang_log, delta_log, omega_log, pm_log = [], [], [], [], []
    events = []

    # Loop over total duration in steps
    for chunk_start in range(0, n_total, n_chunk):
        t = np.linspace(chunk_start*dt, (chunk_start+n_chunk)*dt, n_chunk)
        for tk in t:
            pm, scale, desc = sim.apply_disturbance(tk, dist_type, pm_base)
            sim.est.set_load_scale(scale)
            if desc != 'No Disturbance' and (len(events) == 0 or events[-1][1] != desc):
                events.append((tk, desc))

            # compute voltages
            vm, va = sim.est.compute_bus_voltages_from_states(sim.est.x_hat)
            z = np.zeros(sim.est.n_meas)
            z[:sim.system['n_bus']] = vm
            z[sim.system['n_bus']:2*sim.system['n_bus']] = va
            z[2*sim.system['n_bus']:2*sim.system['n_bus']+sim.system['n_gen']] = pm
            z[2*sim.system['n_bus']+sim.system['n_gen']:] = 0.0
            z += np.random.normal(0, 1e-3, size=z.shape)

            out = sim.est.estimate(z, pm)

            Vmag_log.append(vm)
            Vang_log.append(va)
            delta_log.append(out['rotor_angles'])
            omega_log.append(out['rotor_speeds'])
            pm_log.append(pm)

        # package 1 chunk result
        result = {
            'time': t,
            'Vmag': np.array(Vmag_log),
            'Vang': np.array(Vang_log),
            'delta': np.array(delta_log),
            'omega': np.array(omega_log),
            'Pm': np.array(pm_log),
            'events': events
        }
        q.put(result) # sending to logger and analyzer
        print(f"[Simulator] Sent chunk {(chunk_start//n_chunk)+1}")

        # reset logs for next chunk
        Vmag_log, Vang_log, delta_log, omega_log, pm_log = [], [], [], [], []

    print("[Simulator] Finished all chunks")
    q.put(None)  # tell logger/analyzer to stop



def logger_process(q: Queue):
    f0 = 60.0   # system frequency
    samples_per_dt = 32   # oversample each EKF step

    while True:
        result = q.get()
        if result is None:
            break

        time = result['time']
        Vmag = result['Vmag']
        Vang = result['Vang']
        omega = result['omega']
        events = result['events']

        file_exists = os.path.isfile(CSV_FILE)
        with open(CSV_FILE, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists or os.stat(CSV_FILE).st_size == 0:
                header = ["t"] + [f"Vbus{i+1}_wave" for i in range(Vmag.shape[1])] + \
                         [f"omega_gen{i+1}" for i in range(omega.shape[1])]
                writer.writerow(header)

            dt = time[1] - time[0]   # EKF step size
            for k, tk in enumerate(time):
                # create finer time grid inside [tk, tk+dt)
                t_fine = np.linspace(tk, tk+dt, samples_per_dt, endpoint=False)
                for t_sub in t_fine:
                    v_instant = [Vmag[k,i]*np.sin(2*np.pi*f0*t_sub + Vang[k,i]) 
                                 for i in range(Vmag.shape[1])]
                    row = [t_sub] + list(v_instant) + list(omega[k,:])
                    writer.writerow(row)

        print(f"[Logger] Logged {len(time)*samples_per_dt} waveform samples. Events={events}")

def analyzer_process(q: Queue):
    while True:
        result = q.get()
        if result is None:
            break
        
        # AI Model Here
        events = result['events']
        omega = result['omega']
        avg_freq = np.mean(omega[:,:]/(2*np.pi), axis=0)
        print(f"[Analyzer] Events={events}")
        print(f"[Analyzer] Avg gen freqs (Hz): {avg_freq}")


if __name__ == "__main__":
    if os.path.exists(CSV_FILE):
        os.remove(CSV_FILE)
        print(f"[Main] Removed old {CSV_FILE}, starting fresh.")

    q1 = Queue()       # for simulation results
    

    p1 = Process(target=simulator_process, args=(q1,))
    p2 = Process(target=logger_process, args=(q1,))
    p3 = Process(target=analyzer_process, args=(q1,))
    

    p1.start()
    p2.start()
    p3.start()


    p1.join()
    q1.put(None)        # stop logger
    q1.put(None)        # stop analyzer

    p2.join()
    p3.join()

