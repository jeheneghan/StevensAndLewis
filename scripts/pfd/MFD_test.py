import sys
sys.path.append('../')
from modeling.params_f16 import F16Params, Controls
import numpy as np

from modeling.sim_f16 import simulate_realtime_decoupled
from tools.lin_f16 import trim_f16 as trimmer
from modeling.eqm import eqm

FPS = 200
END_TIME = 20.0

def _make_controls_and_t(params, controls):
    """Create initial states, time vector, and controls_list for the sim."""
    X0, U0 = trimmer(controls, params)
    from copy import deepcopy
    t = np.linspace(0, END_TIME, int(END_TIME*FPS + 1))
    controls_list = [deepcopy(U0) for _ in t]

    for i, time_val in enumerate(t):
        if 2.0 <= time_val < 3.0:  # 2 to 3 seconds
            controls_list[i].elev_deg = U0.elev_deg + 5.0
        elif 3.0 <= time_val < 4.0:  # 3 to 4 seconds
            controls_list[i].elev_deg = U0.elev_deg - 5.0
    return X0, U0, t, controls_list

def main():
    params = F16Params()
    params.alt_ft = 20000.0
    params.VT_ftps = 502.0
    params.xcg = 0.30

    controls = Controls()
    X0, U0, t, controls_list = _make_controls_and_t(params, controls)

    y, outputs = simulate_realtime_decoupled(func=eqm, X0=X0, t=t, controls_list=controls_list, params=params)
    print(f"Simulation complete: {len(t)} steps")
    return y, outputs

if __name__ == "__main__":
    main()