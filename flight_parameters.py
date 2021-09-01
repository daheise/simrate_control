from SimConnect import *
from geopy import distance
from collections import namedtuple
from sys import maxsize
from math import radians, degrees, ceil, tan, sin, asin, floor, log2
from time import sleep


class SimConnectDataError(Exception):
    pass


# A simple structure to hold distance from the previous and next waypoints
WaypointClearance = namedtuple("WaypointClearances", "prev next")


class FlightDataMetrics:
    def __init__(self, simconnect_connection):
        self.sm = simconnect_connection
        self.aq = AircraftRequests(self.sm)
        self.messages = []

    def __del__(self):
        self.sm.exit()

    def _get_value(self, aq_name, retries=maxsize):
        val = self.aq.find(str(aq_name)).value
        i = 0
        while val is None and i < retries:
            val = self.aq.find(str(aq_name)).value
            i += 1
            sleep(0.01)
        if i > 0:
            self.messages.append(f"Warning: Retried {aq_name} {i} times.")
        return val

    def update(self, retries=maxsize):
        # Load these all up for three reasons.
        # 1. These are static items
        # 2. Running them over and over may trigger a memory leak in the game
        # 3. It seems to increase reliability of reading/setting the data
        self.messages = []
        self.aq_prev_wp_lat = self._get_value("GPS_WP_PREV_LAT")
        self.aq_prev_wp_lon = self._get_value("GPS_WP_PREV_LON")
        self.aq_cur_lat = self._get_value("GPS_POSITION_LAT")
        self.aq_cur_long = self._get_value("GPS_POSITION_LON")
        self.aq_next_wp_ident = self._get_value("GPS_WP_NEXT_ID").decode("utf-8")
        self.aq_next_wp_lat = self._get_value("GPS_WP_NEXT_LAT")
        self.aq_next_wp_lon = self._get_value("GPS_WP_NEXT_LON")
        self.aq_next_wp_alt = self._get_value("GPS_WP_NEXT_ALT")
        self.aq_pitch = self._get_value("PLANE_PITCH_DEGREES")
        self.aq_bank = self._get_value("PLANE_BANK_DEGREES")
        self.aq_vsi = self._get_value("VERTICAL_SPEED")
        self.aq_ap_master = self._get_value("AUTOPILOT_MASTER")
        self.aq_is_prev_wp_valid = self._get_value("GPS_WP_PREV_VALID")
        self.aq_cur_waypoint_index = self._get_value("GPS_FLIGHT_PLAN_WP_INDEX")
        self.aq_num_waypoints = self._get_value("GPS_FLIGHT_PLAN_WP_COUNT")
        self.aq_agl = self._get_value("PLANE_ALT_ABOVE_GROUND")
        self.aq_alt_indicated = self._get_value("INDICATED_ALTITUDE")
        self.aq_nav_mode = self._get_value("AUTOPILOT_NAV1_LOCK")
        self.aq_heading_hold = self._get_value("AUTOPILOT_HEADING_LOCK")
        self.aq_approach_hold = self._get_value("AUTOPILOT_APPROACH_HOLD")
        self.aq_approach_active = self._get_value("GPS_IS_APPROACH_ACTIVE")
        self.aq_ete = self._get_value("GPS_ETE")
        # self.aq_nav_has_nav = [self._get_value("NAV_HAS_NAV:1"), self._get_value("NAV_HAS_NAV:2")]
        self.aq_has_localizer = [
            self._get_value("NAV_HAS_LOCALIZER:1"),
            self._get_value("NAV_HAS_LOCALIZER:2"),
        ]
        self.aq_has_glide_scope = [
            self._get_value("NAV_HAS_GLIDE_SLOPE:1"),
            self._get_value("NAV_HAS_GLIDE_SLOPE:2"),
        ]
        self.aq_ground_speed = self._get_value("GPS_GROUND_SPEED")
        self.aq_title = self._get_value("TITLE")

    def next_waypoint_altitude(self):
        next_alt = self.aq_next_wp_alt * 3.28084
        # If the next waypoint altitude is set to zero, try to approximate
        # setting the alt to the ground's altitude + 3000
        if abs(next_alt) < 10:
            next_alt = self.aq_alt_indicated - self.aq_agl
        return next_alt

    def get_waypoint_distances(self):
        """Get the distance to the previous and next FPL waypoints."""
        try:
            prev_wp_lat = self.aq_prev_wp_lat
            prev_wp_lon = self.aq_prev_wp_lon
            cur_lat = self.aq_cur_lat
            cur_long = self.aq_cur_long
            next_wp_lat = self.aq_next_wp_lat
            next_wp_lon = self.aq_next_wp_lon

            prev_clearance = distance.distance(
                (prev_wp_lat, prev_wp_lon), (cur_lat, cur_long)
            ).nm
            next_clearance = distance.distance(
                (next_wp_lat, next_wp_lon), (cur_lat, cur_long)
            ).nm

            return WaypointClearance(prev_clearance, next_clearance)
        except ValueError:
            raise SimConnectDataError()
        except TypeError:
            raise SimConnectDataError()

    def ground_speed(self):
        # Convert m/s to nm/s
        ground_speed = self.aq_ground_speed * 5.4e-4
        return ground_speed

    def target_altitude_change(self, altitude_change_tolerance=100, minimum_agl=0):
        total_descent = (
            max(self.next_waypoint_altitude(), minimum_agl) - self.aq_alt_indicated
        )
        if abs(total_descent) < altitude_change_tolerance:
            return 0
        return total_descent

    def target_fpm(self, angle):
        # https://code7700.com/rot_descent_vvi.htm
        # Convert nm/s to feet/min
        ground_speed = self.ground_speed() * 60 * 6076.118
        # rough calculation. I have also seen (ground_speed(knots)/2)*10
        return ground_speed * sin(radians(angle))

    def required_fpm(self):
        # https://code7700.com/rot_descent_vvi.htm
        # Convert nm/s to feet/min
        ground_speed = self.ground_speed() * 60 * 6076.118
        # rough calculation. I have also seen (ground_speed(knots)/2)*10
        # convert nm to feet
        waypoint_distance = self.get_waypoint_distances().next * 6076.118
        fpm = 0
        try:
            fpm = ground_speed * asin(self.target_altitude_change() / waypoint_distance)
        except (ValueError, ZeroDivisionError):
            pass
        return fpm

    def distance_to_flc(self, angle):
        if self.target_altitude_change() == 0:
            return self.get_waypoint_distances().next
        return self.get_waypoint_distances().next - self.flc_length(angle)

    def time_to_flc(self, angle):
        gspeed = self.ground_speed()
        if gspeed == 0:
            return 0
        if self.target_altitude_change() == 0:
            return self.get_waypoint_distances().next / gspeed
        seconds = self.distance_to_flc(angle) / gspeed
        return seconds if seconds > 0 else 0

    def flc_length(self, angle, altitude_change_tolerance=100):
        # distance in nm
        # https://www.thinkaviation.net/top-of-descent-calculation/
        total_descent = self.target_altitude_change()
        feet_to_nm = 6076.118
        # Solve the triangle to get a chosen degree of of change
        angle = radians(angle)
        distance = (total_descent / tan(angle)) / feet_to_nm
        # safety_distance = self.ground_speed() * self.descent_safety_factor
        if abs(total_descent) < altitude_change_tolerance:
            return 0
        return distance

    def distance_to_destination(self):
        # Distance in nm
        dist = self.ground_speed() * self.aq_ete
        return dist

    def choose_angle(self, climb, descend):
        return float(climb) if self.target_altitude_change() > 0 else float(descend)

    # def get_approach_minutes(self) -> float:
    #     """Estimate the number of minutes needed for a successful arrival
    #     based on AGL.
    #     Parameters
    #     ----------
    #     final_fpm: Estimate of a typical descent speed on final.
    #     performance_exponent: Exponent that represents whether to become more
    #         or less conservative on your descent as altitude increases. 1.0
    #         would indicate that you intend to descend at final_fpm dur the
    #         entire descent. Less than one is higher descent speed at higher
    #         altitude. Greater than 1 would be descending slower at higher
    #         altitudes.
    #     minimum: The minimum minutes this function will return.

    #     Known Issue: This may cause some sim rate rubber banding as altitude
    #     changes.
    #     """
    #     minutes = 0
    #     try:
    #         agl = self.aq_agl.value
    #         minutes = (agl/self.final_fpm)**(self.performance_exponent)

    #     except TypeError:
    #         raise SimConnectDataError()

    #     logging.debug(f"get_approach_minutes(): {agl} ft AGL, {minutes} min")
    #     return max(minutes, self.min_approach_time)


class SimrateDiscriminator:
    def __init__(self, flight_parameters, config=None):
        self.config = config
        self.flight_params: FlightDataMetrics = flight_parameters
        self.messages = []
        if config is None or config.sections() == []:
            self.max_vsi = 2000
            self.min_vsi = -2000
            self.max_bank = 5
            self.max_pitch = 7
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
            self.config = config
            self.min_vsi = int(self.config["stability"]["min_vsi"])
            self.max_vsi = int(self.config["stability"]["max_vsi"])
            self.max_bank = int(self.config["stability"]["max_bank"])
            self.max_pitch = int(self.config["stability"]["max_pitch"])
            self.waypoint_buffer = int(
                self.config["stability"]["waypoint_buffer"]
            )  # seconds
            self.min_agl_cruise = int(self.config["stability"]["min_agl_cruise"])  # ft

            # These values relate to approach detection
            self.min_agl_descent = int(
                self.config["stability"]["min_agl_descent"]
            )  # ft
            self.destination_distance = float(
                self.config["stability"]["destination_distance"]
            )  # nm
            self.min_approach_time = int(
                self.config["stability"]["min_approach_time"]
            )  # minutes
            # Top of descent is estimated as
            # ((altitude agl)/final_approach)**descent_safety_factor
            # where ** is the exponentiation operator.
            # self.final_approach = int(self.config['stability']['final_approach']) # fpm
            self.degrees_of_descent = float(
                self.config["stability"]["degrees_of_descent"]
            )
            self.angle_of_climb = float(self.config["stability"]["angle_of_climb"])
            self.decel_for_climb = self.config["stability"].getboolean(
                "decel_for_climb"
            )
            self.descent_safety_factor = float(
                self.config["stability"]["descent_safety_factor"]
            )
            self.altitude_change_tolerance = int(
                self.config["stability"]["altitude_change_tolerance"]
            )
            self.pause_at_tod = config.getboolean("stability", "pause_at_tod")

        self.have_paused_at_tod = False

    def are_angles_aggressive(self):
        """Check to see if pitch and bank angles are "agressive."

        The default values seem to minimize sim rate induced porpoising and
        waddling
        """
        agressive = True
        try:
            pitch = abs(degrees(self.flight_params.aq_pitch))
            bank = abs(degrees(self.flight_params.aq_bank))
            if pitch > self.max_pitch or bank > self.max_bank:
                self.messages.append(
                    f"Agressive angles detected: {pitch} deg {bank} deg"
                )
                agressive = True
            else:
                agressive = False

        except TypeError:
            raise SimConnectDataError()

        return agressive

    def is_vs_aggressive(self):
        """Check to see if the vertical speed is "aggressively" high or low.

        Values are intended to detect and stop porposing. However,
        `are_angles_aggressive` may be a better proxy.
        """
        agressive = True
        try:
            vsi = self.flight_params.aq_vsi
            if vsi > self.min_vsi and vsi < self.max_vsi:
                agressive = False
            else:
                self.messages.append(f"Agressive VS detected: {vsi} ft/s")
                agressive = True

        except TypeError:
            raise SimConnectDataError()

        return agressive

    def is_ap_active(self) -> bool:
        """Is the autopilot configured an a sane way? Namely, is it turned on.

        Would also like it to detect if nav mode is turned on.
        """
        ap_active = False
        try:
            sc_autopilot_active = int(self.flight_params.aq_ap_master)
            sc_nav_mode = int(self.flight_params.aq_nav_mode)
            sc_title = self.flight_params.aq_title
            if sc_autopilot_active and sc_nav_mode:
                ap_active = True
            elif "A320" in str(sc_title) and sc_autopilot_active:
                # Workaround for A320 not reporting AUTOPILOT_NAV1_LOCK like
                # other planes. A320 always reports
                # AUTOPILOT_HEADING_LOCK==True and
                # AUTOPILOT_NAV1_LOCK==False leaving no way to set the
                # autopilot to disable time acceleration.
                ap_active = True
            else:
                ap_active = False
                self.messages.append("AP not active.")

        except TypeError:
            raise SimConnectDataError()
        if ap_active:
            self.messages.append("AP On")
        else:
            self.messages.append("AP Off")
        return ap_active

    def is_waypoints_valid(self):
        try:
            sc_is_prev_wp_valid = self.flight_params.aq_is_prev_wp_valid
            sc_cur_waypoint_index = self.flight_params.aq_cur_waypoint_index
            sc_num_waypoints = self.flight_params.aq_num_waypoints
            if (
                sc_is_prev_wp_valid
                and sc_num_waypoints > 0
                and sc_cur_waypoint_index <= sc_num_waypoints
            ):
                return True
        except TypeError:
            raise SimConnectDataError()

        self.messages.append("No valid waypoints detected.")
        return False

    def is_waypoint_close(self):
        """Check is a waypoint is close by.

        In the future, the definition of "close" could be parameterized on
        bearing to/out of the waypoint. Greater degrees of turn
        need more buffer.

        IMPORTANT: Buffer size is decided largely by the FMS switching waypoints early
        to cut corners.
        """
        close = True
        try:
            ground_speed = self.flight_params.aq_ground_speed  # units: meters/sec
            mps_to_nmps = 5.4e-4  # one meter per second to 1 nautical mile per second
            nautical_miles_per_second = ground_speed * mps_to_nmps
            previous_dist = nautical_miles_per_second * self.waypoint_buffer
            next_dist = nautical_miles_per_second * self.waypoint_buffer
            clearance = self.flight_params.get_waypoint_distances()

            if clearance.prev > previous_dist and clearance.next > next_dist:
                close = False
            else:
                self.messages.append(
                    f"Close ({previous_dist:.2f}, {next_dist:.2f}) to waypoint: ({clearance.prev:.2f} nm, {clearance.next:.2f} nm)"
                )
        except TypeError:
            raise SimConnectDataError()

        return close

    def is_too_low(self, low):
        """Is the plan below `low` AGL"""
        try:
            agl = self.flight_params.aq_agl
            if agl > low:
                return False
        except TypeError:
            raise SimConnectDataError()

        self.messages.append(f"Plane close to ground: {agl} ft AGL")
        return True

    def is_last_waypoint(self):
        """Is the FMS tagerting the final waypoint?"""
        cur_waypoint_index = self.flight_params.aq_cur_waypoint_index
        num_waypoints = self.flight_params.aq_num_waypoints
        try:
            # Don't really understand the indexing here...
            # Indexes skip by twos, so you get 2, 4, 6...N-1,
            # i.e. the last waypoint will be 7 not 8.
            # I don't know why the last waypoint violates the pattern.
            if cur_waypoint_index < (num_waypoints - 1):
                return False
        except TypeError:
            raise SimConnectDataError()

        return True

    def is_past_leg_flc(self):
        angle = self.flight_params.choose_angle(
            self.angle_of_climb, self.degrees_of_descent
        )
        try:
            if (
                not self.decel_for_climb
                and self.flight_params.target_altitude_change() > 0
            ):
                return False
            if (
                self.flight_params.time_to_flc(angle) < self.descent_safety_factor
                and self.flight_params.target_altitude_change() != 0
            ):
                return True
        except Exception as e:
            pass
        return False

    def is_approaching(self):
        """Checks several items to see if we are "arriving" because there are
        different ways a flight plan may be set up.

        These count as arriving:
        1. Within `close`nm of the final waypoint
        2. AGL < min_agl_descent and last waypoint is active
        3. The aircraft is beyond the top of descent for the next waypoint.
        4. The aircraft is beyond the top of descent for the destination.
        5. "Approach" mode is active on the AP

        More to consider:
        1. NAV has picked up a localizer signal
        2. NAV/GPS has has a glide scope
        """
        approaching = False
        try:
            last = self.is_last_waypoint()
            if last:
                too_low = self.is_too_low(self.min_agl_descent)
            else:
                too_low = False
            autopilot_active = bool(self.flight_params.aq_ap_master)
            approach_hold = bool(self.flight_params.aq_approach_hold)
            # approach_active = bool(self.aq_approach_active.value)

            # has_localizer = [int(l.value) for l in self.aq_has_localizer]
            # has_glide_scope = [int(l.value) for l in self.aq_has_glide_scope]
            # print(approach_active, has_localizer, has_glide_scope)
            # min_distance = max(self.distance_to_flc(), self.destination_distance)
            # if (((last or approach_active) and self.is_past_tod()) or
            if self.is_past_leg_flc():
                self.messages.append("Prepare for FLC.")
                approaching = True

            if last and too_low:
                self.messages.append(f"Last waypoint and low")
                approaching = True

            # seconds = ceil(self.get_approach_minutes(descent_fpm, performance_exponent,
            # minimum_time))*60

            #            if approach_active and self.is_past_tod():
            # logging.info("Plane is descending slow while on approach.")
            # logging.info(f"Target {self.degrees_of_descent} degree FPM: -{self.target_descent_fpm()} fpm")
            # logging.info(f"Current Rate: {self.aq_vsi.value} fpm")
            # approaching = True

            seconds = self.min_approach_time * 60
            if self.flight_params.aq_ete < seconds:
                self.messages.append(f"Less than {seconds/60} minutes from destination")
                approaching = True

            if autopilot_active and approach_hold:
                self.messages.append("Approach hold mode on")
                approaching = True

            # if(has_localizer[0] or has_localizer[1] or
            #    has_glide_scope[0] or has_glide_scope[1]):
            #    logging.info("Localizer or glide scope detected")
            #    approaching = True

        except TypeError:
            raise SimConnectDataError()

        return approaching

    def get_max_sim_rate(self):
        """Returns what is considered the maximum stable sim rate.

        Looks at a lot of factors.
        """
        stable = 1
        self.messages = []
        try:
            if self.is_waypoints_valid():
                if not self.is_ap_active():
                    self.messages.append("Autopilot not enabled.")
                    stable = 1
                elif self.is_approaching():
                    if self.pause_at_tod and not self.have_paused_at_tod:
                        self.have_paused_at_tod = True
                        self.messages.append("Pause at TOD.")
                        stable = 0
                    else:
                        stable = 1
                    self.messages.append("Flight level change needed.")
                elif self.are_angles_aggressive():
                    self.messages.append("Pitch or bank too high")
                    stable = 1
                elif self.is_too_low(self.min_agl_cruise):
                    self.messages.append("Too close to ground.")
                    stable = 1
                elif self.is_vs_aggressive():
                    # pitch/bank may be a better/suffcient proxy
                    self.messages.append("Vertical speed too high.")
                    stable = 2
                elif self.is_waypoint_close():
                    # The AP will switch waypoints several miles away to cut corners,
                    # so we slow down far enough away that we don't enter a turn prior
                    # to slowing down (4nm). To keep from speeding up immediately when
                    # a corner is cut we also give until 2.5nm after the switch
                    self.messages.append("Close to waypoint.")
                    stable = 2
                else:
                    self.messages.append("Flight stable")
                    stable = 16
            else:
                self.messages.append("No valid flight plan. Stability undefined.")
                stable = 1
        except SimConnectDataError as e:
            self.messages.append("DATA ERROR: DECEL")
            stable = 1

        return min(stable, int(self.config["simrate"]["max_rate"]))

    def get_messages(self):
        return self.messages
