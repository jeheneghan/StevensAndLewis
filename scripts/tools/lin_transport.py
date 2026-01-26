import numpy as np
from model.eqm import transport
from scipy.optimize import minimize
import control as ct

RTOD = 57.296

def get_lin_transport(x, u, gamma, xcg=0.25, land=0):

    def cost_transport(xu):
        x0 = x.copy()
        u0 = u.copy()
        x0[1] = xu[0]
        x0[2] = xu[0] + gamma
        u0[0:2] = xu[1:3]
        xd, _, _ = transport(x0, u0, xcg, land)
        cost = xd[0]**2 + 1e4*xd[1]**2 + 1e4*xd[3]**2
        return cost

    S0 = np.array([
                    x[1],    # Alpha (rad)
                    u[0],    # Throttle (0-1)
                    u[1],    # Elevator (deg)
                    ])
    S = minimize(cost_transport, S0)['x'] #fminsearch is optimisation using nelder-mead

    print(S)

    x0 = x.copy()
    u0 = u.copy()
    x0[1] = S[0]
    x0[2] = S[0] + gamma
    u0[0:2] = S[1:3]
    xd, keas, _ = transport(x0, u0, xcg, land)

    print('Trim results:')
    print(f"Cost = {cost_transport(S)}")
    print(f"Keas (knots): {keas:.2f}")
    print(f"Alpha (deg): {x0[1] * RTOD:.2f}")
    print(f"Theta (deg): {x0[2] * RTOD:.2f}")
    print(f"Throttle (0-1): {u0[0]:.2f}")
    print(f"Elevator (deg): {S[2]:.2f}")

    def linze(x0, u0, xcg=0.25, land=0, maxiter=20):

        A_height = len(x0)
        A_width = len(x0)
        B_height = len(x0)
        B_width = len(u0)
        A = np.zeros((A_height,A_width))
        B = np.zeros((B_height,B_width))
        tol = 1e-4
        dy = 0.1*x0
        dy[x0<tol] = 1e-4

        # Compute A matrix using central difference method
        for j in range(A_width):
            for i in range(maxiter):
                x1 = np.copy(x0)
                x1[j] += dy[j]
                xd1, _, _ = transport(x1, u0, xcg, land)
                x2 = np.copy(x0)
                x2[j] -= dy[j]
                xd2, _, _ = transport(x2, u0, xcg, land)

                for k in range(A_height):
                    A[k][j] = (xd1[k]-xd2[k])/(2*dy[j])
                
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


        du = 0.1*u0
        du[u0<tol] = 1e-4
        # Compute B matrix using central difference method
        for j in range(B_width):
            for i in range(maxiter):
                u1 = np.copy(u0)
                u1[j] += du[j]
                xd1, _, _ = transport(x0, u1, xcg, land)
                u2 = np.copy(u0)
                u2[j] -= du[j]
                xd2, _, _ = transport(x0, u2, xcg, land)

                for k in range(A_height):
                    B[k][j] = (xd1[k]-xd2[k])/(2*du[j])

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

        # C matrix
        C = np.eye(len(x0))
        C[:,[1,2,3]] = C[:,[1,2,3]] * RTOD # alpha_deg, theta_deg, q_dps

        return ct.ss(A, B, C, np.zeros((len(x0),len(u0))))
    
    return linze(np.c_[x0], np.c_[u0], xcg, land)
                    