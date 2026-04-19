def _apply_washout_filter(value, filter):
    """First-order washout filter"""
    filtered_value =  value - filter.u + 0.998 * filter.y
    return filtered_value

class Filters:
    def __init__(self):
        self.u = 0.0
        self.y = 0.0

    def get(self, name):
        return getattr(self, name)

    def set(self, name, value):
        setattr(self, name, value)

yaw_rate_filter = Filters()

def elevon_controller(SAS, outputs, elevon_pilot):
    elevon_sas = 0.0
    if SAS=='sas':
        k_alpha = 0.5
        k_q = 0.25
        pitch_rate_dps = outputs.q_dps
        alpha_deg = outputs.alpha_deg
        elevon_sas = alpha_deg * k_alpha + pitch_rate_dps * k_q

    return elevon_sas

def aileron_controller(SAS, outputs, aileron_pilot):
    aileron_sas = 0.0
    if SAS=='sas':
        k_p = 0.1
        roll_rate_dps = outputs.p_dps
        aileron_sas = roll_rate_dps * k_p

    return aileron_sas

def rudder_controller(SAS, outputs, rudder_pilot):
    rudder_sas = 0.0
    if SAS=='sas':
        k_r = 1.3
        yaw_rate_dps = outputs.r_dps
        yaw_rate_dps_washed = _apply_washout_filter(yaw_rate_dps, yaw_rate_filter)
        rudder_sas = yaw_rate_dps_washed * k_r
        yaw_rate_filter.u = yaw_rate_dps
        yaw_rate_filter.y = yaw_rate_dps_washed

    return rudder_sas

def update_controls_from_fcs(SAS, outputs, controls_state, shared_state):

    # Elevon
    elevon_pilot = controls_state.elev_deg - shared_state['elev_neutral']
    elevon_sas = elevon_controller(SAS, outputs, elevon_pilot)

    # Aileron
    aileron_pilot = controls_state.ail_deg
    aileron_sas = aileron_controller(SAS, outputs, aileron_pilot)

    # Rudder
    rudder_pilot = controls_state.rudder_deg
    rudder_sas = rudder_controller(SAS, outputs, rudder_pilot)

    # Combine pilot and SAS inputs for elevon command
    controls_state.elev_deg = elevon_pilot + elevon_sas + shared_state['elev_neutral']
    controls_state.ail_deg = aileron_pilot + aileron_sas
    controls_state.rudder_deg = rudder_pilot + rudder_sas



