from model.engine_f16 import tgear
from model.eqm import eqm
from numpy import zeros

def cost_trim_f16_straight_level(S, controls, params):
    X = zeros((13,1))

    # Set values from solver
    controls.throttle = S[0] # throttle 0-1
    controls.elev_deg = S[1] # elev_deg
    X[1] = S[2] # alpha_rad
    controls.ail_deg = S[3] # ail_deg
    controls.rudder_deg = S[4] # rudder_deg
    X[2] = S[5] # beta_rad
    
    # Set other non-zero values
    X[0] = params.VT_ftps # VT_fps
    X[4] = X[1] # theta_rad       
    X[11] = params.alt_ft # alt_ft
    X[12] = tgear(controls.throttle) # power_perc

    XD, _ = eqm(X, controls, params)
    return XD[0]**2 + 100*(XD[1]**2 + XD[2]**2) + 10*(XD[6]**2 + XD[7]**2 + XD[8]**2)

