from sc_config import SimrateControlConfig
from SimConnect import *
from geopy import distance
from collections import namedtuple
from sys import maxsize
from math import radians, degrees, tan, sin, cos, asin, atan2
from time import sleep


class SimConnectDataError(Exception):
    pass


# A simple structure to hold distance from the previous and next waypoints
WaypointClearance = namedtuple("WaypointClearances", "prev next")


class FlightDataMetrics:
    def __init__(self, simconnect_connection, config: SimrateControlConfig):
        self.sm = simconnect_connection
        self._config = config
        self.aq = AircraftRequests(self.sm)
        self.messages = []
        self._request_sleep = 0.05
        self._max_request_sleep = 0.5
        self._min_request_sleep = 0.01
        self.update()

    def _get_value(self, aq_name, retries=maxsize):
        # PySimConnect seems to crash the sim if requests happen too fast.
        sleep(self._request_sleep)
        val = self.aq.find(str(aq_name)).value
        i = 0
        while val is None and i < retries:
            i += 1
            self._request_sleep = min(
                self._request_sleep + 0.05 * i, self._max_request_sleep
            )
            sleep(self._request_sleep)
            val = self.aq.find(str(aq_name)).value
        if i > 0:
            self.messages.append(f"Warning: Retried {aq_name} {i} times.")
        self._request_sleep = max(self._min_request_sleep, self._request_sleep - 0.01)
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
        self.aq_next_wp_lat = self._get_value("GPS_WP_NEXT_LAT")
        self.aq_next_wp_lon = self._get_value("GPS_WP_NEXT_LON")
        self.aq_next_wp_alt = self._get_value("GPS_WP_NEXT_ALT")
        self.aq_pitch = self._get_value("PLANE_PITCH_DEGREES")
        self.aq_bank = self._get_value("PLANE_BANK_DEGREES")
        self.aq_vsi = self._get_value("VERTICAL_SPEED")
        self.aq_ap_master = self._get_value("AUTOPILOT_MASTER")
        self.aq_cur_waypoint_index = self._get_value("GPS_FLIGHT_PLAN_WP_INDEX")
        self.aq_num_waypoints = self._get_value("GPS_FLIGHT_PLAN_WP_COUNT")
        self.aq_agl = self._get_value("PLANE_ALT_ABOVE_GROUND")
        self.aq_alt_indicated = self._get_value("INDICATED_ALTITUDE")
        self.aq_nav_mode = self._get_value("AUTOPILOT_NAV1_LOCK")
        self.aq_heading_hold = self._get_value("AUTOPILOT_HEADING_LOCK")
        self.aq_approach_hold = self._get_value("AUTOPILOT_APPROACH_HOLD")
        self.aq_approach_active = self._get_value("GPS_IS_APPROACH_ACTIVE")
        self.aq_ete = self._get_value("GPS_ETE")
        self.aq_ground_speed = self._get_value("GPS_GROUND_SPEED")
        self.aq_ground_elevation = self._get_value("GROUND_ALTITUDE")
        self.aq_flaps_percent = max(
            self._get_value("TRAILING_EDGE_FLAPS_LEFT_PERCENT"),
            self._get_value("TRAILING_EDGE_FLAPS_RIGHT_PERCENT"),
        )

        ident = self._get_value("GPS_WP_NEXT_ID").decode("utf-8")
        self.aq_next_wp_ident = (
            ident
            if (
                self.next_waypoint_altitude()
                > (self.get_ground_elevation() + self._config.waypoint_minimum_agl)
                and self._config.waypoint_vnav
            )
            else f"LAND ({ident})"
        )

    def next_waypoint_altitude(self):
        next_alt = self.aq_next_wp_alt * 3.28084
        # If the next waypoint altitude is set to zero, try to approximate
        # setting the alt to the ground's
        if (
            next_alt - self.get_ground_elevation()
        ) < self._config.waypoint_minimum_agl or not self._config.waypoint_vnav:
            next_alt = self.get_ground_elevation()
        return next_alt

    def get_ground_elevation(self):
        # return self.aq_alt_indicated - self.aq_agl
        return self.aq_ground_elevation * 3.28084

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
            if (
                self.next_waypoint_altitude()
                <= (self.get_ground_elevation() + self._config.waypoint_minimum_agl)
                or not self._config.waypoint_vnav
            ):
                return WaypointClearance(
                    prev_clearance, self.ground_speed() * self.aq_ete
                )
            else:
                return WaypointClearance(prev_clearance, next_clearance)
        except ValueError:
            raise SimConnectDataError()
        except TypeError:
            raise SimConnectDataError()

    def _get_bearing(self, lat1, lon1, lat2, lon2):
        dLon = lon2 - lon1
        y = sin(dLon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
        brng = degrees(atan2(y, x))
        brng = (brng + 360) % 360
        return brng

    def get_waypoint_directions(self):
        prev_wp_lat = radians(self.aq_prev_wp_lat)
        prev_wp_lon = radians(self.aq_prev_wp_lon)
        cur_lat = radians(self.aq_cur_lat)
        cur_long = radians(self.aq_cur_long)
        next_wp_lat = radians(self.aq_next_wp_lat)
        next_wp_lon = radians(self.aq_next_wp_lon)

        brng_prev = self._get_bearing(cur_lat, cur_long, prev_wp_lat, prev_wp_lon)
        brng_next = self._get_bearing(cur_lat, cur_long, next_wp_lat, next_wp_lon)

        return (brng_prev, brng_next)

    def ground_speed(self):
        # Convert m/s to nm/s
        ground_speed = self.aq_ground_speed * 5.4e-4
        return ground_speed

    def target_altitude_change(self):
        total_descent = self.next_waypoint_altitude() - self.aq_alt_indicated
        if abs(total_descent) < self._config.altitude_change_tolerance:
            return 0
        return total_descent

    def target_fpm(self):
        angle = self.choose_slope_angle()
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

    def distance_to_flc(self):
        if self.target_altitude_change() == 0:
            return self.get_waypoint_distances().next

        return self.get_waypoint_distances().next - self.flc_length()

    def time_to_flc(self):
        gspeed = self.ground_speed()
        if gspeed == 0:
            return 0
        if self.target_altitude_change() == 0:
            return self.get_waypoint_distances().next / gspeed
        seconds = self.distance_to_flc() / gspeed
        return seconds if seconds > 0 else 0

    def flc_length(self):
        # distance in nm
        # https://www.thinkaviation.net/top-of-descent-calculation/
        angle = self.choose_slope_angle()
        total_descent = self.target_altitude_change()
        feet_to_nm = 6076.118
        # Solve the triangle to get a chosen degree of of change
        angle = radians(angle)
        distance = (total_descent / tan(angle)) / feet_to_nm
        if abs(total_descent) < self._config.altitude_change_tolerance:
            return 0
        return distance

    def distance_to_destination(self):
        # Distance in nm
        dist = self.ground_speed() * self.aq_ete
        return dist

    def choose_slope_angle(self):
        return (
            self._config.angle_of_climb
            if self.target_altitude_change() > 0
            else self._config.degrees_of_descent
        )


class SimrateDiscriminator:
    def __init__(self, flight_parameters, config: SimrateControlConfig):
        self._config = config
        self.flight_params: FlightDataMetrics = flight_parameters
        self.messages = []
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
            if pitch > self._config.max_pitch or bank > self._config.max_bank:
                self.messages.append(
                    f"Agressive angles detected: {int(pitch)} deg {int(bank)} deg"
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
            if vsi > self._config.min_vsi and vsi < self._config.max_vsi:
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
            sc_autopilot_active = bool(self.flight_params.aq_ap_master)
            sc_nav_mode = int(self.flight_params.aq_nav_mode)
            if sc_autopilot_active and sc_nav_mode:
                ap_active = True
            elif not self._config.ap_nav_guarded and sc_autopilot_active:
                # Some planes do not report AUTOPILOT_NAV1_LOCK and give
                # AUTOPILOT_HEADING_LOCK==True and
                # AUTOPILOT_NAV1_LOCK==False leaving no way for the user to
                # set the autopilot (e.g. to heading mode) to disable time
                # acceleration.
                self.messages.append("Simrate not LNAV guarded.")
                ap_active = True
            else:
                ap_active = False

        except TypeError:
            raise SimConnectDataError()
        return ap_active

    def is_waypoints_valid(self):
        try:
            sc_is_prev_wp_valid = (
                self.flight_params.aq_prev_wp_lat is not None
                and self.flight_params.aq_prev_wp_lon is not None
            )
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
            previous_dist = nautical_miles_per_second * self._config.waypoint_buffer
            next_dist = nautical_miles_per_second * self._config.waypoint_buffer
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
        try:
            # Allow acceleration if the VSI is better than the required.
            if (
                self.flight_params.target_altitude_change() > 0
                and self.flight_params.aq_vsi >= self.flight_params.required_fpm()
            ) or (
                self.flight_params.target_altitude_change() < 0
                and self.flight_params.aq_vsi <= self.flight_params.required_fpm()
            ):
                return False

            if (
                not self._config.decel_for_climb
                and self.flight_params.target_altitude_change() > 0
            ):
                return False

            if (
                self.flight_params.time_to_flc() < self._config.descent_safety_factor
                and self.flight_params.target_altitude_change() != 0
            ):
                return True
        except Exception as e:
            return True
        return False

    def is_flc_needed(self):
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
                too_low = self.is_too_low(self._config.min_agl_descent)
            else:
                too_low = False
            autopilot_active = bool(self.flight_params.aq_ap_master)
            approach_hold = bool(self.flight_params.aq_approach_hold)

            if self.is_past_leg_flc():
                self.messages.append("Prepare for FLC.")
                approaching = True

            if last and too_low:
                self.messages.append(f"Last waypoint and low")
                approaching = True

            seconds = self._config.min_approach_time * 60
            if self.flight_params.aq_ete < seconds:
                self.messages.append(f"Less than {seconds/60} minutes from destination")
                approaching = True

            if (
                autopilot_active
                and approach_hold
                and self._config.ap_approach_hold_guarded
            ):
                self.messages.append("Approach hold mode on")
                approaching = True
            elif (
                autopilot_active
                and approach_hold
                and not self._config.ap_approach_hold_guarded
            ):
                self.messages.append("Approach hold unguarded")

        except TypeError:
            raise SimConnectDataError()

        return approaching

    def is_cruise_configured(self):
        cruise_configured = True
        if self.flight_params.aq_flaps_percent > 0:
            cruise_configured = False
            self.messages.append("Flaps extended")

        return cruise_configured

    def get_max_sim_rate(self):
        """Returns what is considered the maximum stable sim rate.

        Looks at a lot of factors.
        """
        stable = 1
        self.messages = []
        try:
            if self.is_waypoints_valid():
                if not self.is_ap_active():
                    stable = 1
                elif (
                    not self.is_cruise_configured()
                    and self._config.check_cruise_configuration
                ):
                    stable = 1
                elif self.is_flc_needed():
                    if self._config.pause_at_tod and not self.have_paused_at_tod:
                        self.have_paused_at_tod = True
                        self.messages.append("Pause at TOD.")
                        stable = 0
                    else:
                        stable = self._config.min_rate
                    self.messages.append("Flight level change needed.")
                elif self.is_too_low(self._config.min_agl_cruise):
                    self.messages.append("Too close to ground.")
                    stable = self._config.min_rate
                elif self.are_angles_aggressive():
                    self.messages.append("Pitch or bank too high")
                    stable = self._config.cautious_rate
                elif self.is_vs_aggressive():
                    # pitch/bank may be a better/suffcient proxy
                    self.messages.append("Vertical speed too high.")
                    stable = self._config.cautious_rate
                elif self.is_waypoint_close():
                    # The AP will switch waypoints several seconds away to cut corners,
                    # so we slow down far enough away that we don't enter a turn prior
                    # to slowing down (4nm). To keep from speeding up immediately when
                    # a corner is cut we also give until 2.5nm after the switch
                    self.messages.append("Close to waypoint.")
                    stable = self._config.cautious_rate
                else:
                    self.messages.append("Flight stable")
                    stable = self._config.max_rate
            else:
                self.messages.append("No valid flight plan. Stability undefined.")
                stable = self._config.min_rate
        except SimConnectDataError as e:
            self.messages.append("DATA ERROR: DECEL")
            stable = self._config.min_rate

        return min(stable, int(self._config.max_rate))

    def get_messages(self):
        return self.messages
