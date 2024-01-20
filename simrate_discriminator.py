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
from flight_parameters import FlightDataMetrics
from system_parameters import SystemDataMetrics
from simrate_exceptions import SimConnectDataError
import os

def rate_steps(rate, safety_margin=1):
    return max(ceil(log2(rate)), 0) + safety_margin

class SimrateDiscriminator:
    def __init__(self, flight_parameters: FlightDataMetrics, system_parameters: SystemDataMetrics, config: SimrateControlConfig):
        self._config = config
        self.flight_params: FlightDataMetrics = flight_parameters
        self.system_params: SystemDataMetrics = system_parameters
        self.messages = []
        self.simrate_samples = deque([], maxlen=10)
        self.have_paused_at_tod = False
        self.lnm_userpoints = []
        try:
            lnm_userdata = sql.connect(
                expandvars(
                    "%APPDATA%\ABarthel\little_navmap_db\little_navmap_userdata.sqlite"
                )
            )
            cur = lnm_userdata.cursor()
            self.lnm_userpoints = cur.execute(
                "SELECT name,type,laty,lonx FROM userdata"
            ).fetchall()
        except:
            pass

    def is_gpu_overloaded(self):
        return self.system_params.memory_utilization > self._config.gpu_memory_utilization_limit

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
            elif (vsi > self._config.min_vsi and vsi < self._config.max_vsi) and (abs(target) < max(abs(self._config.min_vsi), self._config.max_vsi)):
                agressive = False
            else:
                agressive = False
            
            if agressive:
                self.messages.append(f"Agressive VS detected: {vsi} ft/s")

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
        for r in self.lnm_userpoints:
            d = distance.distance(
                (self.flight_params.aq_cur_lat, self.flight_params.aq_cur_long),
                (r[2], r[3]),
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
            if close[0] or close[1]:
                self.heading_samples = deque([], maxlen=10)
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

        # if (self.flight_params.aq_alt_indicated / self.flight_params.max_alt_seen) < 0.1:
        #     self.messages.append(f"Less than 1/10 of highest altitude")
        #     retval = True

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
            # Allow acceleration if the VSI is close to the required.
            if (
                abs(self.flight_params.target_altitude_change())
                > self._config.altitude_change_tolerance
                and abs(self.flight_params.required_fpm() - self.flight_params.aq_vsi)
                < abs(self.flight_params.required_fpm()) * 0.25
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
        return heading_turbulence or vsi_turbulence
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
        stable = []
        try:          
            if self.is_lnm_userpoint_close(7):
                self.messages.append(f"Close to POI")
                stable += [self._config.cautious_rate]

            if self.are_angles_aggressive():
                self.messages.append("Pitch or bank too high")
                stable += [self._config.cautious_rate]
            if self.is_vs_aggressive():
                # pitch/bank may be a better/suffcient proxy
                self.messages.append("Vertical speed too high.")
                stable += [self._config.cautious_rate]
            if self.is_gusty():
                self.messages.append("Turbulence too high.")
                stable += [self._config.cautious_rate]
            
            if self.is_waypoints_valid():
                if self.is_gpu_overloaded():
                    self.messages.append("GPU overloaded")
                    stable += [self._config.min_rate]
                elif self.is_flc_needed():
                    if self._config.pause_at_tod and not self.have_paused_at_tod:
                        self.have_paused_at_tod = True
                        self.messages.append("Pause at TOD.")
                        stable += [self._config.min_rate]
                    else:
                        stable += [self._config.min_rate]
                    self.messages.append("Flight level change needed.")
                elif self.is_in_hold():
                    self.messages.append(f"Holding at")
                    stable += [self._config.min_rate]
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
                ):
                    # The AP will switch waypoints several seconds away to cut corners,
                    # so we slow down far enough away that we don't enter a turn prior
                    # to slowing down. To keep from speeding up immediately when
                    # a corner is cut we also give distance after the switch
                    self.messages.append("Close to custom waypoint.")
                    stable += [self._config.cautious_rate]
                elif (
                    self.is_waypoint_close(self._config.minimum_waypoint_distance)[0]
                    or self.is_waypoint_close(self._config.minimum_waypoint_distance)[1]
                ):
                    self.messages.append("Close to waypoint.")
                    stable += [self._config.cautious_rate]
                else:
                    self.messages.append("Flight stable")
                    stable += [self._config.max_rate]
            else:
                self.messages.append("No valid flight plan. Stability undefined. Choose rate wisely.")
                stable += [self._config.max_rate]

            if self.flight_params.simconnect_error:
                    self.messages.append("Simconnect error")
                    stable += [self._config.min_rate]
            
            # Check things that cause minimum rate last

            if not self.is_ap_active():
                    stable = self._config.min_rate
            
            if (
                    not self.is_cruise_configured() or not self.is_cruise_lights()
                ) and self._config.check_cruise_configuration:
                    stable += [self._config.min_rate]
            
            if self.is_too_low(self.flight_params.min_agl_cruise()):
                    self.messages.append("Too close to ground.")
                    min_alt = (
                        self.flight_params.get_ground_elevation()
                        + self.flight_params.min_agl_cruise()
                    )
                    self.messages.append(f"Minimum altitude: {min_alt}")
                    stable += [self._config.min_rate]
            
            if self.is_lnm_userpoint_close(3):
                    self.messages.append(f"Very close to POI")
                    stable += [self._config.min_rate]

        except SimConnectDataError as e:
            self.messages.append("DATA ERROR: DECEL")
            stable += [self._config.min_rate]
        self.simrate_samples += [min(stable)]
        return min(min(self.simrate_samples), int(self._config.max_rate))

    def get_messages(self):
        messages = copy(self.messages)
        return messages
