import configparser


class SimrateControlConfigError(Exception):
    pass


class SimrateControlConfig:
    def __init__(self, file=None):
        if file is None:
            self.max_vsi = 2000
            self.min_vsi = -2000
            self.max_bank = 20
            self.max_pitch = 10
            self.waypoint_buffer = 40  # seconds
            self.min_agl_cruise = 1000  # ft

            # These values relate to approach detection
            self.min_agl_descent = 3000  # ft
            self.destination_distance = 10  # nm
            self.min_approach_time = 7  # minutes
            # Top of descent is estimated as
            # ((altitude agl)/final_approach)**descent_safety_factor
            # where ** is the exponentiation operator.
            # self.final_approach = 500 # fpm
            self.degrees_of_descent = 3
            self.descent_safety_factor = 1.0
            self.pause_at_tod = False
        else:
            self._config = configparser.ConfigParser()
            self._config.read("config.ini")
            if self._config.sections() == []:
                raise SimrateControlConfigError

            self.annunciation = self._config.getboolean("simrate", "annunciation")
            self.decelerate_on_simconnect_error = self._config.getboolean(
                "simrate", "decelerate_on_simconnect_error"
            )
            self.max_rate = float(self._config["simrate"]["max_rate"])
            self.min_rate = float(self._config["simrate"]["min_rate"])
            self.cautious_rate = float(self._config["simrate"]["cautious_rate"])
            self.set_barometer = self._config.getboolean("simrate", "set_barometer")
            self.set_mixture = self._config.getboolean("simrate", "set_mixture")

            self.min_vsi = int(float(self._config["stability"]["min_vsi"]))
            self.max_vsi = int(float(self._config["stability"]["max_vsi"]))
            self.max_bank = float(self._config["stability"]["max_bank"])
            self.max_pitch = float(self._config["stability"]["max_pitch"])
            self.waypoint_buffer = int(
                self._config["stability"]["waypoint_buffer"]
            )  # seconds
            self.minimum_waypoint_distance = float(
                self._config["stability"]["minimum_waypoint_distance"]
            )  # nm
            self.custom_waypoint_distance = float(
                self._config["stability"]["custom_waypoint_distance"]
            )  # nm
            self.min_agl_cruise = int(self._config["stability"]["min_agl_cruise"])  # ft
            self.min_agl_protection = self._config["stability"].getboolean("min_agl_protection")

            # These values relate to approach detection
            self.min_agl_descent = int(
                self._config["stability"]["min_agl_descent"]
            )  # ft
            self.destination_distance = float(
                self._config["stability"]["destination_distance"]
            )  # nm
            self.min_approach_time = int(
                self._config["stability"]["min_approach_time"]
            )  # minutes
            # Top of descent is estimated as
            # ((altitude agl)/final_approach)**descent_safety_factor
            # where ** is the exponentiation operator.
            # self.final_approach = int(self._config['stability']['final_approach']) # fpm
            self.degrees_of_descent = float(
                self._config["stability"]["degrees_of_descent"]
            )
            self.angle_of_climb = float(self._config["stability"]["angle_of_climb"])
            self.decel_for_climb = self._config["stability"].getboolean(
                "decel_for_climb"
            )
            self.descent_safety_factor = float(
                self._config["stability"]["descent_safety_factor"]
            )
            self.ap_nav_guarded = self._config.getboolean(
                "stability", "nav_mode_guarded"
            )

            self.ap_approach_hold_guarded = self._config.getboolean(
                "stability", "approach_hold_guarded"
            )
            self.check_cruise_configuration = self._config.getboolean(
                "stability", "check_cruise_configuration"
            )

            self.ete_guard = self._config.getboolean(
                "stability", "ete_guarded"
            )

            self.waypoint_vnav = self._config.getboolean("stability", "waypoint_vnav")

            self.pause_at_tod = self._config.getboolean("stability", "pause_at_tod")

            self.altitude_change_tolerance = int(
                self._config["metrics"]["altitude_change_tolerance"]
            )
            self.waypoint_minimum_agl = int(
                self._config["metrics"]["waypoint_minimum_agl"]
            )

            if self.pause_at_tod:
                self.waypoint_vnav = False
