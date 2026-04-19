from modeling.engine_f16 import tgear, pdot, thrust
from modeling.aerodata_f16 import CX,CY,CZ, CL,CM,CN, DLDA, DLDR, DNDA, DNDR, aerodynamic_damp
from numpy import asarray, sqrt, cos, sin, zeros, arctan2, sqrt, pi

R0 = 2.377e-3 # slug/ft^3

class OutputState:
    """Container for F-16 simulation output values."""
    def __init__(self):
        self.keas = 0.0
        self.alpha_deg = 0.0
        self.beta_deg = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.p_dps = 0.0
        self.q_dps = 0.0
        self.r_dps = 0.0
        self.nz_g = 0.0
        self.nz_g_pilot = 0.0
        self.ny_g = 0.0
        self.nx_g = 0.0
        self.Q_lbfpft2 = 0.0
        self.mach = 0.0
        self.thrust_pound = 0.0
        self.aero_forces = [0.0, 0.0, 0.0]
        self.aero_moments = [0.0, 0.0, 0.0]
        self.gamma_deg = 0.0
        self.heading_deg = 0.0
        self.track_deg = 0.0
        self.altitude_ft = 0.0
        self.vspeed_fpm = 0.0
        self.power = 0.0

def  airdata(Vt_fps, alt_ft):
    TFac = 1 - 0.703e-5 * alt_ft
    T = 519 * TFac

    if (alt_ft >= 35000):
        T = 390

    rho = R0 * TFac**4.14
    aMach = Vt_fps/sqrt(1.4*1716.3*T)
    qBar = 0.5*rho*Vt_fps**2
    return aMach, qBar

def _wrap_angle(angle_deg, range=[0, 2*pi]):
    """Unwrap angle to range, i.e. [0, 360]."""
    if angle_deg < range[0]:
        angle_deg += (range[1] - range[0])
    if angle_deg >= range[1]:
        angle_deg -= (range[1] - range[0])
    return angle_deg

def eqm(X, controls, params):
    X = asarray(X)
    if X.ndim == 1:
        if X.size == 13:
            X = X.reshape((13, 1))
        else:
            raise ValueError(f"eqm requires X with 13 elements, got {X.size}")
    elif X.shape != (13, 1):
        raise ValueError(f"eqm requires X shaped (13,1), got {X.shape}")

    # F-16 model from Stevens And Lewis,second edition, pg 184
    mass = params.mass
    geom = params.geom
    
    g0_ftps2 = 32.17
    rad2deg = 57.29578

    # python script
    XD=zeros([len(X),1])
    
    # Control variables
    throttle_u = controls.throttle
    elev_deg = controls.elev_deg
    ail_deg = controls.ail_deg
    rudder_deg = controls.rudder_deg

    # Wrap angles to avoid discontinuities in display
    X[3,0] = _wrap_angle(X[3,0], range=[-pi, pi])
    X[4,0] = _wrap_angle(X[4,0], range=[-pi/2, pi/2])
    X[5,0] = _wrap_angle(X[5,0], range=[0, 2*pi])
    
    # Assign state & control variables
    VT_ftps = X[0,0]
    alpha_deg = X[1,0]*rad2deg
    beta_deg = X[2,0]*rad2deg
    phi_rad = X[3,0]
    theta_rad = X[4,0]
    psi_rad = X[5,0]
    p_rps = X[6,0]
    q_rps = X[7,0]
    r_rps = X[8,0]
    alt_ft = X[11,0]
    power = X[12,0]
    
    # Air data computer and engine model
    mach, Q_lbfpft2 = airdata(VT_ftps, alt_ft)
    
    # Engine model
    cpow = tgear(throttle_u)
    XD[12,0] = pdot(power, cpow)
    thrust_pound = thrust(power, alt_ft, mach)
    
    # Look-up tables and component buildup
    CXT = CX(alpha_deg, elev_deg)
    CYT = CY(beta_deg, ail_deg, rudder_deg)
    CZT = CZ(alpha_deg, beta_deg, elev_deg)
    dail = ail_deg/20.0
    drdr = rudder_deg/30.0
    CLT = CL(alpha_deg, beta_deg) + DLDA(alpha_deg, beta_deg)*dail + DLDR(alpha_deg, beta_deg)*drdr
    CMT = CM(alpha_deg, elev_deg)
    CNT = CN(alpha_deg, beta_deg) + DNDA(alpha_deg, beta_deg)*dail + DNDR(alpha_deg, beta_deg)*drdr
    
    # Add damping derivatives
    TVT = 0.5/VT_ftps
    B2V = geom.wingspan_ft*TVT
    CQ = geom.chord_ft*q_rps*TVT
    D = aerodynamic_damp(alpha_deg)
    CXT = CXT + CQ*D[0,0]
    CYT = CYT + B2V*(D[1,0]*r_rps + D[2,0]*p_rps)
    CZT = CZT + CQ*D[3,0]
    CLT = CLT + B2V*(D[4,0]*r_rps + D[5,0]*p_rps)
    CMT = CMT + CQ*D[6,0] + CZT*(geom.xcgr_mac - params.xcg)
    CNT = CNT + B2V*(D[7,0]*r_rps + D[8,0]*p_rps) - CYT*(geom.xcgr_mac - params.xcg)*geom.chord_ft/geom.wingspan_ft
    
    # Get ready for state equations
    cos_beta = cos(X[2,0])
    sin_theta = sin(theta_rad)
    cos_theta = cos(theta_rad)
    sin_phi = sin(phi_rad)
    cos_phi = cos(phi_rad)
    sin_psi = sin(psi_rad)
    cos_psi = cos(psi_rad)
    QS = Q_lbfpft2*geom.wing_ft2
    QSb = QS*geom.wingspan_ft
    g0_cos_theta = g0_ftps2*cos_theta
    Q_sin_phi = q_rps*sin_phi
    QS_over_mass = QS/mass.mass_slug
    
    u_ftps = VT_ftps*cos(X[1,0])*cos_beta
    v_ftps = VT_ftps*sin(X[2,0])
    w_ftps = VT_ftps*sin(X[1,0])*cos_beta
    
    ax_ftps2 = (QS*CXT + thrust_pound)/mass.mass_slug
    ay_ftps2 = QS_over_mass*CYT
    az_ftps2 = QS_over_mass*CZT
    
    # Force equations
    udot_ftps2 = r_rps*v_ftps - q_rps*w_ftps - g0_ftps2*sin_theta   + ax_ftps2
    vdot_ftps2 = p_rps*w_ftps - r_rps*u_ftps + g0_cos_theta*sin_phi + ay_ftps2
    wdot_ftps2 = q_rps*u_ftps - p_rps*v_ftps + g0_cos_theta*cos_phi + az_ftps2
    u2_plus_w2 = u_ftps**2 + w_ftps**2
    XD[0,0] = (u_ftps*udot_ftps2 + v_ftps*vdot_ftps2 + w_ftps*wdot_ftps2)/VT_ftps
    XD[1,0] = (u_ftps*wdot_ftps2 - w_ftps*udot_ftps2) / u2_plus_w2
    XD[2,0] = (VT_ftps*vdot_ftps2 - v_ftps*XD[0,0])*cos_beta / u2_plus_w2
    
    # Kinematics
    XD[3,0] = p_rps + (sin_theta/cos_theta)*(Q_sin_phi + r_rps*cos_phi)
    XD[4,0] = q_rps*cos_phi - r_rps*sin_phi
    XD[5,0] = (Q_sin_phi + r_rps*cos_phi)/cos_theta
    
    # Moments
    roll_rps = QSb*CLT
    pitch_rps = QS*geom.chord_ft*CMT
    yaw_rps = QSb*CNT
    p_q = p_rps*q_rps
    q_r = q_rps*r_rps
    q_hx = q_rps*geom.engmomenthx_slugft2ps
    XD[6,0] = (mass.XPQ*p_q - mass.XQR*q_r + mass.AZZ*roll_rps + mass.AXZ*(yaw_rps + q_hx))/mass.GAM
    XD[7,0] = (mass.YPR*p_rps*r_rps - mass.AXZ*(p_rps**2 - r_rps**2) + pitch_rps - r_rps*geom.engmomenthx_slugft2ps)/mass.AYY
    XD[8,0] = (mass.ZPQ*p_q - mass.XPQ*q_r + mass.AXZ*roll_rps + mass.AXX*(yaw_rps + q_hx))/mass.GAM
    
    # Navigation
    T1 = sin_phi*cos_phi
    T2 = cos_phi*sin_theta
    T3 = sin_phi*sin_psi
    S1 = cos_theta*cos_psi
    S2 = cos_theta*sin_psi
    S3 = T1*sin_theta - cos_phi*sin_psi
    S4 = T3*sin_theta + cos_phi*cos_psi
    S5 = sin_phi*cos_theta
    S6 = T2*cos_psi + T3
    S7 = T2*sin_psi - T1
    S8 = cos_phi*cos_theta
    
    XD[9,0] = u_ftps*S1 + v_ftps*S3 + w_ftps*S6        # North speed
    XD[10,0] = u_ftps*S2 + v_ftps*S4 + w_ftps*S7        # East speed
    XD[11,0] = u_ftps*sin_theta - v_ftps*S5 - w_ftps*S8 # Vertical speed
    
    outputs = OutputState()
    outputs.keas = sqrt(2*Q_lbfpft2/R0)*0.5925
    outputs.alpha_deg = alpha_deg
    outputs.beta_deg = beta_deg
    outputs.pitch_deg = theta_rad*rad2deg
    outputs.roll_deg = phi_rad*rad2deg
    outputs.p_dps = p_rps*rad2deg
    outputs.q_dps = q_rps*rad2deg
    outputs.r_dps = r_rps*rad2deg
    outputs.nz_g = -az_ftps2/g0_ftps2 - 1 # remove 1 to account for gravity
    outputs.nz_g_pilot = -(az_ftps2 - XD[7,0]*geom.pilot_station_ft + p_rps*r_rps*geom.pilot_station_ft)/g0_ftps2 - 1
    outputs.ny_g = ay_ftps2/g0_ftps2
    outputs.nx_g = ax_ftps2/g0_ftps2
    outputs.Q_lbfpft2 = Q_lbfpft2
    outputs.mach = mach
    outputs.thrust_pound = thrust_pound
    outputs.aero_forces = [CXT, CYT, CZT]
    outputs.aero_moments = [CLT, CMT, CNT]
    outputs.gamma_deg = theta_rad*rad2deg - alpha_deg
    outputs.heading_deg = psi_rad*rad2deg
    outputs.track_deg = arctan2(XD[10,0], XD[9,0])*rad2deg
    outputs.altitude_ft = alt_ft
    outputs.vspeed_fpm = XD[11,0]*60
    outputs.power = power

    return XD, outputs

def transport(x, u, xcg=0.25, land=0):

    # Constants
    S = 2170.0  # ft^2
    CBAR = 17.5  # ft
    MASS = 5.0e3 # lb
    IYY = 4.1e6 # slug*ft^2
    TSTAT = 6.0e4
    DTDV = -38.0
    ZE = 2.0
    CDCLS = 0.042
    CLA = 0.085 # per deg
    CMA = -0.022 # per deg
    CMDE = -0.016 # per deg
    CMQ = -16.0 # per rad/s
    CMADOT = -6.0 # per rad
    RTOD = 57.296
    GD = 32.17 # ft/s^2

    # State variables
    Vt = x[0,0]
    alpha = x[1,0] * RTOD
    theta = x[2,0]
    q = x[3,0]
    h = x[4,0]

    # Control variables
    thtl = u[0,0]
    elev = u[1,0]
    # Handle glide path input for auto land
    if len(u) > 2:
        gamma_r = u[2,0]

    # Atmosphere
    mach, qbar = airdata(Vt, h)
    rho_0 = 0.00237 # sl/ft^3
    KEAS = sqrt(2*qbar/rho_0) * 0.5925 # knots

    # Define useful variables
    qS = qbar * S
    salp = sin(x[1,0])
    calp = cos(x[1,0])
    gamma = theta - x[1,0]
    sgam = sin(gamma)
    cgam = cos(gamma)

    # Constant coefficients
    if land == 0:
        CL0 = 0.20
        CD0 = 0.016
        CM0 = 0.05
        DCDG = 0.0
        DCMG = 0.0
    else:
        CL0 = 1.0
        CD0 = 0.08
        CM0 = -0.20
        DCDG = 0.02
        DCMG = -0.05

    thrust = (TSTAT + DTDV * Vt) * min(max(thtl,0),1) # thrust
    CL = CL0 + CLA * alpha # nondim lift
    CM = DCMG + CM0 + CMA * alpha + CMDE * elev + CL * (xcg - 0.25) # moment
    CD = DCDG + CD0 + CDCLS * CL**2 # drag polar

    # State equations
    xd = zeros(len(x))
    xd[0,0] = (thrust*calp - qS * CD) / MASS - GD * sgam # Vt dot
    xd[1,0] = (-thrust*salp - qS * CL + MASS * (Vt * q + GD * cgam)) / (MASS * Vt) # alpha dot
    xd[2,0] = q # theta dot
    D = 0.5 * CBAR * (CMQ * q + CMADOT * xd[1,0]) / Vt # pitch damping
    xd[3,0] = (qS * CBAR * (CM + D) + thrust * ZE) / IYY # q dot
    xd[4,0] = Vt * sgam # h dot
    xd[5,0] = Vt * cgam # horizontal speed

    # Handle glide path output for auto land
    if len(x) > 6:
        xd[6,0] = Vt * sin(gamma - gamma_r)

    return xd, KEAS, qbar



    