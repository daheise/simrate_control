from tkinter import Y
from sc_config import SimrateControlConfig

# from lib.simconnect_mobiflight import SimConnectMobiFlight
from lib.koseng.mobiflight_variable_requests import MobiFlightVariableRequests
from SimConnect import *
from geopy import distance
from collections import namedtuple, deque
from sys import maxsize
from math import ceil, radians, degrees, sqrt, tan, sin, cos, asin, atan2, log2
from time import sleep
from copy import copy
from os.path import expandvars
import sqlite3 as sql

class SimConnectDataError(Exception):
    pass


# A simple structure to hold distance from the previous and next waypoints
WaypointClearance = namedtuple("WaypointClearances", "prev next")


def rate_steps(rate, safety_margin=1):
    return max(ceil(log2(rate)), 0) + safety_margin


def windowed_collector(collector, variable, smoothing_points=3):
    if variable is not None:
        collector.insert(0, variable)
    return collector[0:smoothing_points]


class FlightDataMetrics:
    def __init__(self, simconnect_connection, config: SimrateControlConfig):
        self.sm = simconnect_connection
        self.vr = MobiFlightVariableRequests(self.sm)
        self.vr.clear_sim_variables()
        self._config = config
        self.aq = AircraftRequests(self.sm)
        self.messages = []
        self._request_sleep = (
            1 / 31
        )  # 31 is the number of simconnect requests as of writing
        self._max_request_sleep = 0.1
        self._min_request_sleep = 1 / 32
        self.first_waypoint = ""
        self.smoothing_points = 10
        # self.wind_samples = []
        self.heading_samples = deque([], maxlen= 30)
        self.vsi_samples = deque([], maxlen = 30)
        self.max_alt_seen = 0
        self.simconnect_error = False
        self.update()

    def _get_value(self, aq_name, retries=maxsize):
        # PySimConnect seems to crash the sim if requests happen too fast.
        val = self.aq.find(str(aq_name)).value
        i = 0
        while val is None and i < retries:
            sleep(self._request_sleep)
            self._request_sleep = min(
                self._request_sleep + (self._min_request_sleep * (i ** 2)),
                # Given the propensity for simconnect to cause crashes, back off agressively
                # self._request_sleep * (i**2),
                self._max_request_sleep,
            )
            val = self.aq.find(str(aq_name)).value
            i += 1
        if i > 0:
            self.messages.append(f"Warning: Retried {aq_name} {i} times.")
        self._request_sleep = max(
            self._min_request_sleep, self._request_sleep - self._min_request_sleep
        )
        return val

    @property
    def ete(self):
        ete1 = self._get_value("GPS_ETE")

        # ete2 is a workaround for the WT avionics framework ETE.
        # See issue #40
        ete2 = 0
        distance = self.vr.get("(L:WT1000_LNav_Destination_Dis)") / 1852
        gspeed = self.ground_speed()
        if distance > 0 and gspeed > 0:
            # meters to nmi
            ete2 = distance / gspeed

        ete = max(ete1, ete2)
        return ete

    def windowed_collector(self, collector, variable):
        if variable is None:
            return collector[0 : self.smoothing_points]
        return collector.append(variable)[0 : self.smoothing_points]

    def update(self, retries=maxsize):
        # Load these all up for three reasons.
        # 1. These are static items
        # 2. Running them over and over may trigger a memory leak in the game
        # 3. It seems to increase reliability of reading/setting the data
        self.messages = []
        self.simconnect_error = False
        self.aq_title = self._get_value("TITLE").decode("utf-8").strip()
        ident = self._get_value("GPS_WP_NEXT_ID").decode("utf-8").strip()
        prev_ident = self._get_value("GPS_WP_PREV_ID").decode("utf-8").strip()
        self.aq_next_wp_alt = self._get_value("GPS_WP_NEXT_ALT")
        self.aq_ground_elevation = self._get_value("GROUND_ALTITUDE")
        if len(self.first_waypoint.replace("(LAND)", "").strip()) == 0:
            self.first_waypoint = prev_ident
        self.aq_prev_wp_ident = (
            prev_ident if not prev_ident.startswith("TIMECLI") else self.first_waypoint
        ).strip()
        self.aq_next_wp_ident = ident.strip()
        self.aq_next_wp_ident = (
            ident
            if (
                self.next_waypoint_altitude()
                > (self.get_ground_elevation() + self._config.waypoint_minimum_agl)
                and self._config.waypoint_vnav
            )
            else f"{ident} (LAND)"
        ).strip()

        self.aq_prev_wp_lat = self._get_value("GPS_WP_PREV_LAT")
        self.aq_prev_wp_lon = self._get_value("GPS_WP_PREV_LON")
        self.aq_cur_lat = self._get_value("GPS_POSITION_LAT")
        self.aq_cur_long = self._get_value("GPS_POSITION_LON")
        self.aq_next_wp_lat = self._get_value("GPS_WP_NEXT_LAT")
        self.aq_next_wp_lon = self._get_value("GPS_WP_NEXT_LON")
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
        self.aq_ground_speed = self._get_value("GPS_GROUND_SPEED")
        self.aq_heading_indicator = self._get_value("HEADING_INDICATOR")
        self.heading_samples.append(self.aq_heading_indicator)
        self.vsi_samples.append(self.aq_vsi)
        self.max_alt_seen = max(self.max_alt_seen, self.aq_alt_indicated)
        #self.heading_samples = deque([], maxlen = 30)
        #windowed_collector(
        #    self.heading_samples, self.aq_heading_indicator, 30
        #)
        #self.vsi_samples = deque([], maxlen = 30) #windowed_collector(self.vsi_samples, self.aq_vsi, 30)
        # self.aq_windx = self._get_value("AIRCRAFT_WIND_X")
        # self.aq_windy = self._get_value("AIRCRAFT_WIND_Y")
        # self.aq_windz = self._get_value("AIRCRAFT_WIND_Z")
        # self.wind_samples = windowed_collector(
        #    self.wind_samples, (self.aq_windx, self.aq_windy, self.aq_windz), 100
        # )
        # self.crosswind_samples = self.windowed_collector(self.crosswind_samples, self.aq_crosswind)
        # self.vsi_samples = self.windowed_collector(self.vsi_samples, self.aq_vsi)
        # self.ground_speed_samples = self.windowed_collector(self.ground_speed_samples, self.aq_ground_elevation)
        # self.agl_samples = self.windowed_collector(self.agl_samples, self.agl_samples)
        # self.crosswind = max(self.crosswind_samples, self.aq_crosswind)
        # self.vsi_min = max(self.vsi_samples, self.aq_vsi)
        # self.vsi_max = min(self.vsi_samples, self.aq_vsi)
        self.aq_ete = self.ete
        self.aq_flaps_percent = max(
            self._get_value("TRAILING_EDGE_FLAPS_LEFT_PERCENT"),
            self._get_value("TRAILING_EDGE_FLAPS_RIGHT_PERCENT"),
        )
        self.aq_landing_lights = self._get_value("LIGHT_LANDING")
        # Not the best way to handle special cases, but I'm just making sure it
        # works at all fight now.
        if (
            "Airbus A320 Neo FlyByWire" in self.aq_title
            or "Airbus A320neo FlyByWire" in self.aq_title
        ):
            self.aq_ap_master = bool(
                self.vr.get("(L:A32NX_AUTOPILOT_1_ACTIVE)")
                + self.vr.get("(L:A32NX_AUTOPILOT_2_ACTIVE)")
            )
            self.aq_nav_mode = bool(
                self.vr.get("(L:A32NX_FCU_HDG_MANAGED_DASHES)")
                + self.vr.get("(L:A32NX_FCU_HDG_MANAGED_DOT)")
            )
        if (
            "Cessna CJ4 Citation" in self.aq_title
            or "Salty Boeing 747-8i" in self.aq_title
        ):
            wt_lnav = self.vr.get("(L:WT_CJ4_NAV_ON, Bool)")
            self.aq_nav_mode = bool(self.aq_nav_mode + wt_lnav)

    def next_waypoint_altitude(self):
        next_alt = self.aq_next_wp_alt * 3.28084
        # If the next waypoint altitude is set to zero, try to approximate
        # setting the alt to the ground's
        # Known phantoms:
        # TIMECLI = Initial climb out of airport. Set to minimum cruise alt.
        # TIMEVER = Seem to be related to arrival. Should be use ground
        # elevation.
        if "TIMECLI" in self.aq_next_wp_ident:
            next_alt = self.get_ground_elevation() + self._config.min_agl_cruise
        elif "TIMEVER" in self.aq_next_wp_ident:
            next_alt = self.get_ground_elevation()

        if (
            next_alt - self.get_ground_elevation()
        ) < self._config.waypoint_minimum_agl or not self._config.waypoint_vnav:
            next_alt = self.get_ground_elevation()
        return next_alt

    def get_ground_elevation(self):
        # return self.aq_alt_indicated - self.aq_agl
        return self.aq_ground_elevation * 3.28084

    def get_destination_distance(self):
        return self.ground_speed() * self.aq_ete

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

    def get_vnav_distance(self):
        if (
            self.next_waypoint_altitude()
            <= (self.get_ground_elevation() + self._config.waypoint_minimum_agl)
            or not self._config.waypoint_vnav
        ):
            return self.ground_speed() * self.aq_ete
        else:
            return self.get_waypoint_distances().next

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
        waypoint_distance = self.get_vnav_distance() * 6076.118
        fpm = 0
        try:
            fpm = ground_speed * asin(self.target_altitude_change() / waypoint_distance)
        except (ValueError, ZeroDivisionError):
            pass
        return fpm

    def min_agl_cruise(self):
        fpm = self._config.min_agl_cruise
        protected_alt = self._config.min_agl_cruise
        if self.target_fpm() < 0 or self.aq_vsi < 0:
            fpm = abs(min(self.target_fpm(), self.aq_vsi))
        if self._config.min_agl_protection:
            protected_alt = abs(fpm) * rate_steps(self._config.max_rate)
        return max(self._config.min_agl_cruise, protected_alt)

    def distance_to_flc(self):
        if self.target_altitude_change() == 0:
            return self.get_vnav_distance()

        return self.get_vnav_distance() - self.flc_length()

    def time_to_flc(self):
        gspeed = self.ground_speed()
        if gspeed == 0:
            return 0
        if self.target_altitude_change() == 0:
            return self.get_vnav_distance() / gspeed
        seconds = self.distance_to_flc() / gspeed
        return seconds if seconds > 0 else 0

    def flc_length(self, change=None):
        # distance in nm
        # https://www.thinkaviation.net/top-of-descent-calculation/
        angle = self.choose_slope_angle()
        if change is None:
            change = self.target_altitude_change()

        feet_to_nm = 6076.118
        # Solve the triangle to get a chosen degree of of change
        angle = radians(angle)
        distance = (change / tan(angle)) / feet_to_nm
        if abs(change) < self._config.altitude_change_tolerance:
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
        self.simrate_samples = []
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
            target = self.flight_params.target_fpm()
            vsi = self.flight_params.aq_vsi
            if target > 0 and vsi > target:
                agressive = True
            elif target < 0 and vsi < target:
                agressive = True
            elif vsi > self._config.min_vsi and vsi < self._config.max_vsi:
                agressive = False
            else:
                self.messages.append(f"Agressive VS detected: {vsi} ft/s")
                agressive = True

            if self.flight_params.aq_vsi < 0:
                if (
                    abs(
                        (self.flight_params.aq_agl + self._config.min_agl_cruise)
                        // self.flight_params.aq_vsi
                    )
                    < self._config.min_approach_time
                ):
                    self.messages.append(
                        f"VSI may impact ground in less than {self._config.min_approach_time} minutes."
                    )
                    agressive = True
        except TypeError as e:
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

    def is_custom_waypoint(self):
        if self.flight_params.aq_agl > 10000:
            return (False, False)
        # This will trigger about 70 false positives around the world
        custom_prev = (
            self.flight_params.aq_prev_wp_ident == self.flight_params.first_waypoint
            or self.flight_params.aq_prev_wp_ident.startswith("HOLD")
            or self.flight_params.aq_prev_wp_ident.startswith("USR")
            or self.flight_params.aq_prev_wp_ident.startswith("WP")
            or self.flight_params.aq_prev_wp_ident.startswith("POI")
            or len(self.flight_params.aq_prev_wp_ident.split(" ")[0]) > 5
            or not self.flight_params.aq_prev_wp_ident.split(" ")[0].isupper()
        )
        custom_next = (
            self.flight_params.aq_next_wp_ident.startswith("HOLD")
            or self.flight_params.aq_next_wp_ident.startswith("USR")
            or self.flight_params.aq_next_wp_ident.startswith("WP")
            or self.flight_params.aq_prev_wp_ident.startswith("POI")
            or len(self.flight_params.aq_next_wp_ident.split(" ")[0]) > 5
            or not self.flight_params.aq_next_wp_ident.split(" ")[0].isupper()
        )

        return (custom_prev, custom_next)

    def is_lnm_userpoint_close(self, threshold=8):
        min_distance = 25000
        con = sql.connect(
            expandvars("%APPDATA%\ABarthel\little_navmap_db\little_navmap_userdata.sqlite")
        )            
        cur = con.cursor()
        q = cur.execute("SELECT name,type,laty,lonx FROM userdata").fetchall()
        for r in q:
            d = distance.distance(
                (self.flight_params.aq_cur_lat, self.flight_params.aq_cur_long), (r[2], r[3])
            ).nm
            min_distance = min(min_distance, d)

        if min_distance < threshold:
            self.messages.append(f"Close to LNM userpoint {min_distance}nmi.")
            return True
        
        return False




    def is_waypoint_close(self, distance=None):
        """Check is a waypoint is close by.

        In the future, the definition of "close" could be parameterized on
        bearing to/out of the waypoint. Greater degrees of turn
        need more buffer.

        IMPORTANT: Buffer size is decided largely by the FMS switching waypoints early
        to cut corners.
        """
        # close = (True, True)
        if distance is None:
            distance = self._config.minimum_waypoint_distance
        try:
            ground_speed = self.flight_params.aq_ground_speed  # units: meters/sec
            mps_to_nmps = 5.4e-4  # one meter per second to 1 nautical mile per second
            nautical_miles_per_second = ground_speed * mps_to_nmps
            previous_dist = max(
                distance,
                nautical_miles_per_second
                * self._config.waypoint_buffer
                * rate_steps(self._config.cautious_rate),
            )
            next_dist = max(
                distance,
                nautical_miles_per_second
                * self._config.waypoint_buffer
                * rate_steps(self._config.max_rate),
            )

            clearance = self.flight_params.get_waypoint_distances()

            close = (clearance.prev < previous_dist, clearance.next < next_dist)
            if close:
                self.messages.append(
                    f"Close ({previous_dist:.2f}, {next_dist:.2f}) to waypoint: ({clearance.prev:.2f} nm, {clearance.next:.2f} nm)"
                )
        except TypeError:
            raise SimConnectDataError()

        return close

    def is_too_low(self, low):
        """Is the plan below `low` AGL"""
        retval = True
        try:
            agl = self.flight_params.aq_agl
            if agl > low:
                retval = False
        except TypeError:
            raise SimConnectDataError()
        
        if (self.flight_params.aq_alt_indicated / self.flight_params.max_alt_seen) < 0.1:
            self.messages.append(f"Less than 1/10 of highest altitude")
            retval = True
        
        if retval:
            self.messages.append(f"Plane close to ground: {agl} ft AGL")
        return retval

    def is_last_waypoint(self):
        """Is the FMS targeting the final waypoint?"""
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
                abs(self.flight_params.target_altitude_change())
                > self._config.altitude_change_tolerance
                and abs(self.flight_params.required_fpm() - self.flight_params.aq_vsi)
                < self.flight_params.required_fpm()*0.25
            ):
                # ) or (
                #     self.flight_params.target_altitude_change() < 0
                #     and abs(self.flight_params.required_fpm() - self.flight_params.aq_vsi) < 100
                # ):
                return False

            if (
                not self._config.decel_for_climb
                and self.flight_params.target_altitude_change() > 0
            ):
                return False

            if (
                self.flight_params.time_to_flc()
                < self._config.descent_safety_factor * rate_steps(self._config.max_rate)
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
            if self.flight_params.aq_ete < seconds and self._config.ete_guard:
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

    def is_cruise_lights(self):
        lights = not self.flight_params.aq_landing_lights
        if not lights:
            self.messages.append("Lights not configured for cruise")
        return lights

    def is_cruise_configured(self):
        cruise_configured = True
        if self.flight_params.aq_flaps_percent > 0:
            cruise_configured = False
            self.messages.append("Flaps extended")

        return cruise_configured

    def is_in_hold(self):
        holding = False
        if "HOLD" in self.flight_params.aq_next_wp_ident:
            holding = True
            self.messages.append("In a hold")
        return holding

    def is_gusty(self, heading_threshold=3, vsi_threshold=950):
        # Heading threshold is in degrees
        # vsi threshold is in fpm
        max_heading = max(self.flight_params.heading_samples)
        min_heading = min(self.flight_params.heading_samples)
        heading_diff = abs(max_heading - min_heading) % 360
        if (heading_diff) > 180:
            heading_diff = abs((2 * 3.14159) - heading_diff)
        
        max_vsi = max(self.flight_params.vsi_samples)
        min_vsi = min(self.flight_params.vsi_samples)
        vsi_diff = abs(max_vsi - min_vsi)

        heading_turbulence = heading_diff > (0.01745329 * heading_threshold)
        vsi_turbulence = vsi_diff > vsi_threshold
        if heading_turbulence:
            self.messages.append(f"Heading turbulence")
        if vsi_turbulence:
            self.messages.append(f"VSI turbulence of {vsi_diff}")
        return (
            heading_turbulence or vsi_turbulence
        )
        # We only care about the relative velocity between max and min
        # local_samples = [
        #     (x, y, z) for (x, y, z) in self.flight_params.wind_samples
        # ]

        # x = [x for (x, _, _) in local_samples]
        # y = [y for (_, y, _) in local_samples]
        # z = [z for (_, _, z) in local_samples]

        # diff_x = abs(max(x) - min(x))
        # diff_y = abs(max(y) - min(y))
        # diff_z = abs(max(z) - min(z))
        # maxx_diff = max(diff_x, diff_y, diff_z)
        # wind_mag = max(max(x), max(y), max(z))#sqrt(max(x)**2 + max(y)**2 + max(z)**2)
        # #minn = min(local_samples)
        # #diff_mag = sqrt(diff_x**2 + diff_y**2 + diff_z**2)
        # #return maxx_diff > sqrt(wind_mag)
        # return False

    def get_max_sim_rate(self):
        """Returns what is considered the maximum stable sim rate.

        Looks at a lot of factors.
        """
        self.messages = []
        stable = self._config.min_rate
        try:
            if self.is_waypoints_valid():
                if self.flight_params.simconnect_error:
                    self.messages("Simconnect error")
                    stable = self._config.min_rate
                elif not self.is_ap_active():
                    stable = self._config.min_rate
                elif (
                    not self.is_cruise_configured() or not self.is_cruise_lights()
                ) and self._config.check_cruise_configuration:
                    stable = self._config.min_rate
                elif self.is_flc_needed():
                    if self._config.pause_at_tod and not self.have_paused_at_tod:
                        self.have_paused_at_tod = True
                        self.messages.append("Pause at TOD.")
                        stable = 0
                    else:
                        stable = self._config.min_rate
                    self.messages.append("Flight level change needed.")
                elif self.is_too_low(self.flight_params.min_agl_cruise()):
                    self.messages.append("Too close to ground.")
                    min_alt = (
                        self.flight_params.get_ground_elevation()
                        + self.flight_params.min_agl_cruise()
                    )
                    self.messages.append(f"Minimum altitude: {min_alt}")
                    stable = self._config.min_rate
                elif self.is_in_hold():
                    self.messages.append(f"Holding at")
                    stable = self._config.min_rate
                elif (
                    (
                        self.is_custom_waypoint()[0]
                        and self.is_waypoint_close(
                            self._config.custom_waypoint_distance
                        )[0]
                    )
                    or (
                        self.is_custom_waypoint()[1]
                        and self.is_waypoint_close(
                            self._config.custom_waypoint_distance
                        )[1]
                    )
                    or (
                        self.flight_params.distance_to_destination()
                        < self._config.custom_waypoint_distance
                    )
                    or self.is_lnm_userpoint_close()
                ):
                    # The AP will switch waypoints several seconds away to cut corners,
                    # so we slow down far enough away that we don't enter a turn prior
                    # to slowing down. To keep from speeding up immediately when
                    # a corner is cut we also give distance after the switch
                    self.messages.append("Close to custom waypoint.")
                    stable = self._config.cautious_rate
                elif (
                    self.is_waypoint_close(self._config.minimum_waypoint_distance)[0]
                    or self.is_waypoint_close(self._config.minimum_waypoint_distance)[1]
                ):
                    self.messages.append("Close to waypoint.")
                    stable = self._config.cautious_rate
                elif self.are_angles_aggressive():
                    self.messages.append("Pitch or bank too high")
                    stable = self._config.cautious_rate
                elif self.is_vs_aggressive():
                    # pitch/bank may be a better/suffcient proxy
                    self.messages.append("Vertical speed too high.")
                    stable = self._config.cautious_rate
                elif self.is_gusty():
                    self.messages.append("Turbulence too high.")
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
        self.simrate_samples = windowed_collector(self.simrate_samples, stable)
        return min(min(self.simrate_samples), int(self._config.max_rate))

    def get_messages(self):
        messages = copy(self.messages)
        return messages
