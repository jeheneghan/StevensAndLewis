import sys
import os
# Add parent directory to path relative to this script's location
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modeling.params_f16 import F16Params, Controls
import numpy as np

from modeling.realtime_sim import simulate_realtime
from tools.lin_f16 import trim_f16 as trimmer
from modeling.eqm import eqm

FPS = 200

def _init_(params, controls):
    """Create initial states, time vector, and controls_list for the sim."""
    X0, U0 = trimmer(controls, params)
    return X0, U0

def main():
    params = F16Params()
    params.alt_ft = 20000.0
    params.VT_ftps = 502.0
    params.xcg = 0.35
    controls = Controls()
    X0, U0 = _init_(params, controls)

    simulate_realtime(func=eqm, X0=X0, controls_state=U0, params=params, SAS='s')

if __name__ == "__main__":
    main()