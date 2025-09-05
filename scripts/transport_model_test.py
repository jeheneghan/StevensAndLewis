from eqm import transport

# Constants
Vt = 500  # ft/s
alpha = 5.43/57.3  # rad
theta = 5.43/57.3  # rad
altitude = 30e3  # ft
x_Earth = 0  # ft
x0 = [Vt, alpha, theta, 0, altitude, x_Earth]
thtl = 0.204  # throttle 0-1
elev = -4.1  # deg
u0 = [thtl, elev]

xd, keas, qbar = transport(x0, u0)

print('Cost =', xd[0]**2 + 100*xd[1]**2 + 10*xd[3]**2)
print('keas =', keas)