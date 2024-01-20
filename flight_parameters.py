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
import subprocess as sp
from simrate_exceptions import SimConnectDataError
import os

# A simple structure to hold distance from the previous and next waypoints
WaypointClearance = namedtuple("WaypointClearances", "prev next")

def rate_steps(rate, safety_margin=1):
    return max(ceil(log2(rate)), 0) + safety_margin

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
        self._min_request_sleep = 1 / 33
        self.first_waypoint = ""
        self.smoothing_points = 10
        self.heading_samples = deque([], maxlen=10)
        self.vsi_samples = deque([], maxlen=10)
        self.max_alt_seen = 0
        self.simconnect_error = False
        self.update()

    def get_gpu_memory_utilization():
        command = "nvidia-smi --query-gpu=memory.free --format=csv"
        memory_free_info = sp.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
        memory_free_values = [int(x.split()[0]) for i, x in enumerate(memory_free_info)]
        command = "nvidia-smi --query-gpu=memory.total --format=csv"
        memory_total_info = sp.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
        memory_total_values = [int(x.split()[0]) for i, x in enumerate(memory_total_info)]
        return memory_free_values[0]/memory_total_values[0]

        get_gpu_memory()

    def _get_value(self, aq_name, retries=maxsize):
        # PySimConnect seems to crash the sim if requests happen too fast.
        sleep(self._min_request_sleep)
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
                self.vr.get("(L:A32NX_AUTOPILOT_ACTIVE)")
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
        # https://code7700.com/rot_descent_vvi.htm $ 404'd
        # https://www.code7700.com/60_to_1_engineer_to_pilot.htm#section5
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
