from model.params_f16 import Controls
from trim_f16 import cost_trim_f16_straight_level
import numpy as np
from model.eqm import eqm
from model.engine_f16 import tgear
from scipy.optimize import minimize
import control as ct

RTOD = 57.29578

def trim_f16(controls, params):
    def costf16(x):
        y = cost_trim_f16_straight_level(x,controls,params)
        return y
    S0 = [
        .4,   #throttle 0-1
        -5.0,  #elev_deg
        0.0,  #alpha_rad
        0.0,  #ail_deg
        0.0,  #rudder_deg
        0.0,  #beta_rad
        ]
    S = minimize(costf16, S0)['x'] #fminsearch is optimisation using nelder-mead

    print('Trim results:')
    print(f"Throttle (0-1): {S[0]:.2f}")
    print(f"Elevator (deg): {S[1]:.2f}")
    print(f"Alpha (deg): {S[2] * RTOD:.2f}")
    print(f"Aileron (deg): {S[3]:.2f}")
    print(f"Rudder (deg): {S[4]:.2f}")
    print(f"Beta (deg): {S[5] * RTOD:.2f}")

    X0 = np.zeros((13,1))
    X0[0] = params.VT_ftps # VT_fps
    X0[1] = S[2] # alpha_rad
    X0[2] = S[5] # beta_rad
    X0[4] = X0[1] # theta_rad
    X0[11] = params.alt_ft # alt_ft
    X0[12] = tgear(S[0]) # power_perc

    controls.throttle = S[0] # throttle 0-1
    controls.elev_deg = S[1] # elev_deg
    controls.ail_deg = S[3] # ail_deg
    controls.rudder_deg = S[4] # rudder_deg

    return X0, controls

def get_lin_f16(controls, params):

    X0, controls = trim_f16(controls, params)

    def linze(y, controls, params, maxiter=20):

        A_height = 16
        A_width = 13
        A = np.zeros((A_height,A_width))
        tol = 1e-4
        dy = 0.1*y
        dy[y<tol] = 1e-4

        # Compute A matrix using central difference method
        for j in range(A_width):
            for i in range(maxiter):
                x1 = np.copy(y)
                x2 = np.copy(y)
                x1[j] = y[j] + dy[j]
                x2[j] = y[j] - dy[j]
                yd1, out1 = eqm(x1, controls, params)
                yd2, out2 = eqm(x2, controls, params)
                Xd1 = np.append(yd1, [out1.nz_g_pilot, out1.ny_g, out1.gamma_deg])
                Xd2 = np.append(yd2, [out2.nz_g_pilot, out2.ny_g, out2.gamma_deg])
                for k in range(A_height):
                    A[k][j] = (Xd1[k]-Xd2[k])/(2*dy[j])

                # check convergence
                if i == 0:
                    A_old = np.copy(A[:,j])
                else:
                    dA = max(np.abs((A[:,j] - A_old)/(A[:,j] + tol)))
                    if np.all(dA < tol):
                        break
                    A_old = np.copy(A[:,j])

                if i == maxiter-1:
                    print(f"Warning: maxiter reached for state {j}")


        long_index = [0, 1, 4, 7, 11, 12] # VT, alpha, theta, q, alt, power
        lat_index = [2, 3, 5, 6, 8] # beta, phi, psi, p, r

        Along = A[long_index][:, long_index]
        Alat = A[lat_index][:, lat_index]

        Clong = np.identity(len(long_index))
        Clat = np.identity(len(lat_index))

        Clong[[1, 2, 3]] = Clong[[1, 2, 3]] * RTOD # alpha_deg, theta_deg, q_dps
        Clat[[0, 1, 2, 3, 4]] = Clat[[0, 1, 2, 3, 4]] * RTOD # beta_deg, phi_deg, psi_deg, p_dps, r_dps

        Clong = np.append(Clong, A[[13, 15]][:, long_index], axis=0) # nz_g_pilot, gamma_deg
        Clat = np.append(Clat, A[[14]][:, lat_index], axis=0) # ny_g

        u = [
            controls.throttle,
            controls.elev_deg,
            controls.ail_deg,
            controls.rudder_deg
            ]
        
        B_width = 4
        B = np.zeros((A_height,B_width))
        du = 0.1*np.array(u)

        for j in range(B_width):
            for i in range(maxiter):
                u1 = np.copy(u)
                u2 = np.copy(u)
                u1[j] = u[j] + du[j]
                u2[j] = u[j] - du[j]
                yd1, out1 = eqm(y, Controls(u1[0],u1[1],u1[2],u1[3]), params)
                yd2, out2 = eqm(y, Controls(u2[0],u2[1],u2[2],u2[3]), params)
                Xd1 = np.append(yd1, [out1.nz_g_pilot, out1.ny_g, out1.gamma_deg])
                Xd2 = np.append(yd2, [out2.nz_g_pilot, out2.ny_g, out2.gamma_deg])
                for k in range(A_height):
                    B[k][j] = (Xd1[k]-Xd2[k])/(2*du[j])

                # check convergence
                if i == 0:
                    B_old = np.copy(B[:,j])
                else:
                    dB = max(np.abs((B[:,j] - B_old)/(B[:,j] + tol)))
                    if np.all(dB < tol):
                        break
                    B_old = np.copy(B[:,j])

                if i == maxiter-1:
                    print(f"Warning: maxiter reached for control {j}")


        long_ctrl_index = [0, 1] # throttle, elevator
        lat_ctrl_index = [2, 3] # aileron, rudder

        Blong = B[long_index][:, long_ctrl_index]
        Blat = B[lat_index][:, lat_ctrl_index]

        Dlong = np.zeros((Blong.shape[0],len(long_ctrl_index)))
        Dlat = np.zeros((Blat.shape[0],len(lat_ctrl_index)))

        Dlong = np.append(Dlong, B[[13, 15]][:, long_ctrl_index], axis=0) # nz_g_pilot
        Dlat = np.append(Dlat, B[[14]][:, lat_ctrl_index], axis=0) # ny_g

        lon_sys = ct.ss(Along,Blong,Clong,Dlong)
        lon_sys.noutputs = ['VT_fps', 'alpha_deg', 'theta_deg', 'q_dps', 'alt_ft', 'power_perc', 'nz_g_pilot', 'gamma_deg']
        lon_sys.ninputs = ['throttle', 'elev_deg']
        lon_sys.nstates = ['VT_fps', 'alpha_rad', 'theta_rad', 'q_rps', 'alt_ft', 'power_perc']

        lat_sys = ct.ss(Alat,Blat,Clat,Dlat)
        lat_sys.noutputs = ['beta_deg', 'phi_deg', 'psi_deg', 'p_dps', 'r_dps', 'ny_g']
        lat_sys.ninputs = ['ail_deg', 'rudder_deg']
        lat_sys.nstates = ['beta_rad', 'phi_rad', 'psi_rad', 'p_rps', 'r_rps']

        return lon_sys, lat_sys

    lon_sys, lat_sys = linze(np.c_[X0], controls, params)
    return lon_sys, lat_sys

