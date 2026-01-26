class F16Params:
    class Mass:
        def __init__(self):
            self.AXX = 9496.0
            self.AYY = 55814.0
            self.AZZ = 63100.0
            self.AXZ = 982.0
            self.AXZ2 = self.AXZ ** 2
            self.XPQ = self.AXZ * (self.AXX - self.AYY + self.AZZ)
            self.GAM = self.AXX * self.AZZ - self.AXZ ** 2
            self.XQR = self.AZZ * (self.AZZ - self.AYY) + self.AXZ2
            self.ZPQ = (self.AXX - self.AYY) * self.AXX + self.AXZ2
            self.YPR = self.AZZ - self.AXX
            self.weight_pound = 20500.0
            self.mass_slug = self.weight_pound / 32.17

    class Geom:
        def __init__(self):
            self.wing_ft2 = 300
            self.wingspan_ft = 30
            self.chord_ft = 11.32
            self.xcgr_mac = 0.35
            self.engmomenthx_slugft2ps = 160
            self.pilot_station_ft = 15

    def __init__(self):
        self.mass = self.Mass()
        self.geom = self.Geom()
        self.g0_ftps2 = 32.17
        self.xcg = .35
        self.coordinated_turn = 0
        self.turn_rate_rps = 0.0
        self.roll_rate_rps = 0.0
        self.pitch_rate_rps = 0.0
        self.phi_rad = 0.0
        self.gamma_rad = 0.0
        self.stability_axis_roll = 0
        self.VT_ftps = 502
        self.alt_ft = 0

    def get(self, name):
        return getattr(self, name)

    def set(self, name, value):
        setattr(self, name, value)

class Controls:
    def __init__(self, throttle=0.0, elev_deg=0.0, ail_deg=0.0, rudder_deg=0.0):
        self.throttle = throttle
        self.elev_deg = elev_deg
        self.ail_deg = ail_deg
        self.rudder_deg = rudder_deg

    def get(self, name):
        return getattr(self, name)

    def set(self, name, value):
        setattr(self, name, value)
