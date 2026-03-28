def _apply_washout_filter(value, filter):
    """First-order washout filter to prevent drift in integrated values."""
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

alpha_filter = Filters()

def elevon_controller(SAS, outputs, elevon_pilot):
    elevon_sas = 0.0
    if SAS=='s':
        k_alpha = 0.5
        k_q = 0.25
        pitch_rate_dps = outputs['q_dps']
        alpha_deg_washed = _apply_washout_filter(outputs['alpha_deg'], alpha_filter)
        elevon_sas = alpha_deg_washed * k_alpha + pitch_rate_dps * k_q
        alpha_filter.u = outputs['alpha_deg']
        alpha_filter.y = alpha_deg_washed

    return elevon_sas

def update_controls_from_fcs(SAS, outputs, controls_state, elev_neutral):

    # Elevon
    elevon_pilot = controls_state.elev_deg - elev_neutral
    elevon_sas = elevon_controller(SAS, outputs, elevon_pilot)

    # Combine pilot and SAS inputs for elevon command
    controls_state.elev_deg = elevon_pilot + elevon_sas + elev_neutral



