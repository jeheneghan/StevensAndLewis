# this code is used in TABLE 3.5-2 of Stevens & Lewis 3rd edition
# to test the implementation of the F-16 model

from params_f16 import F16Params, Controls
from eqm import eqm

params = F16Params()
params.xcg = 0.40
params.mass.weight_pound = 25000.0

controls = Controls()
controls.throttle = 0.9
controls.elev_deg = 20.0
controls.ail_deg = -15.0
controls.rudder_deg = -20.0

x_i = [500, 0.5, -0.2, -1, 1, -1, 0.7, -0.8, 0.9, 1000, 900, 10000, 90]

xd_i, outputs = eqm(x_i, controls, params)

print('xcg =', params.xcg)
print('mass =', params.mass.weight_pound)
print('xd_i =', xd_i)