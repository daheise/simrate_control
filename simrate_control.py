import logging
import os, sys
import configparser
from time import sleep
from collections import namedtuple
from math import radians, degrees, ceil, tan, sin

import pyttsx3
from SimConnect import *
from geopy import distance

logging.basicConfig(level=logging.INFO)
LOGGER = logging.getLogger(__name__)
LOGGER.info("START")

class SimConnectDataError(Exception):
    pass


# A simple structure to hold distance from the previous and next waypoints
WaypointClearance = namedtuple("WaypointClearances", "prev next")


class FlightStability:
    def __init__(self, sm, config = None):
        self.sm = sm
        self.config = config
        if config is None or config.sections() == []:
            self.max_vsi = 2000
            self.min_vsi = -2000
            self.max_bank = 5
            self.max_pitch = 7
            self.waypoint_buffer = 40 #seconds
            self.min_agl_cruise = 1000 #ft

            # These values relate to approach detection
            self.min_agl_descent = 3000 #ft
            self.destination_distance = 10 #nm
            self.min_approach_time = 7 #minutes
            # Top of descent is estimated as 
            # ((altitude agl)/final_approach)**descent_safety_factor
            # where ** is the exponentiation operator.
            #self.final_approach = 500 # fpm
            self.degrees_of_descent = 3
            self.descent_safety_factor = 1.0
            self.pause_at_tod = False
        else:
            self.config = config
            self.min_vsi = int(self.config['stability']['min_vsi'])
            self.max_vsi = int(self.config['stability']['max_vsi'])
            self.max_bank = int(self.config['stability']['max_bank'])
            self.max_pitch = int(self.config['stability']['max_pitch'])
            self.waypoint_buffer = int(self.config['stability']['waypoint_buffer']) #seconds
            self.min_agl_cruise = int(self.config['stability']['min_agl_cruise']) #ft

            # These values relate to approach detection
            self.min_agl_descent = int(self.config['stability']['min_agl_descent']) #ft
            self.destination_distance = float(self.config['stability']['destination_distance']) #nm
            self.min_approach_time = int(self.config['stability']['min_approach_time']) #minutes
            # Top of descent is estimated as 
            # ((altitude agl)/final_approach)**descent_safety_factor
            # where ** is the exponentiation operator.
            #self.final_approach = int(self.config['stability']['final_approach']) # fpm
            self.degrees_of_descent = float(self.config['stability']['degrees_of_descent'])
            self.descent_safety_factor = float(self.config['stability']['descent_safety_factor'])
            self.pause_at_tod = config.getboolean('stability', 'pause_at_tod')
        
        self.have_paused_at_tod = False
        self.aq = AircraftRequests(self.sm)
        # Load these all up for three reasons.
        # 1. These are static items
        # 2. Running them over and over may trigger a memory leak in the game
        # 3. It seems to increase reliability of reading/setting the data
        self.aq_prev_wp_lat = self.aq.find("GPS_WP_PREV_LAT")
        self.aq_prev_wp_lon = self.aq.find("GPS_WP_PREV_LON")
        self.aq_cur_lat = self.aq.find("GPS_POSITION_LAT")
        self.aq_cur_long = self.aq.find("GPS_POSITION_LON")
        self.aq_next_wp_lat = self.aq.find("GPS_WP_NEXT_LAT")
        self.aq_next_wp_lon = self.aq.find("GPS_WP_NEXT_LON")
        self.aq_next_wp_alt = self.aq.find("GPS_WP_NEXT_ALT")
        self.aq_pitch = self.aq.find("PLANE_PITCH_DEGREES")
        self.aq_bank = self.aq.find("PLANE_BANK_DEGREES")
        self.aq_vsi = self.aq.find("VERTICAL_SPEED")
        self.aq_ap_master = self.aq.find("AUTOPILOT_MASTER")
        self.aq_is_prev_wp_valid = self.aq.find("GPS_WP_PREV_VALID")
        self.aq_cur_waypoint_index = self.aq.find("GPS_FLIGHT_PLAN_WP_INDEX")
        self.aq_num_waypoints = self.aq.find("GPS_FLIGHT_PLAN_WP_COUNT")
        self.aq_agl = self.aq.find("PLANE_ALT_ABOVE_GROUND")
        self.aq_alt = self.aq.find("PLANE_ALTITUDE")
        self.aq_nav_mode = self.aq.find("AUTOPILOT_NAV1_LOCK")
        self.aq_heading_hold = self.aq.find("AUTOPILOT_HEADING_LOCK")
        self.aq_approach_hold = self.aq.find("AUTOPILOT_APPROACH_HOLD")
        self.aq_approach_active = self.aq.find("GPS_IS_APPROACH_ACTIVE")
        self.aq_ete = self.aq.find("GPS_ETE")
        # self.aq_nav_has_nav = [self.aq.find("NAV_HAS_NAV:1"), self.aq.find("NAV_HAS_NAV:2")]
        self.aq_has_localizer = [
            self.aq.find("NAV_HAS_LOCALIZER:1"),
            self.aq.find("NAV_HAS_LOCALIZER:2"),
        ]
        self.aq_has_glide_scope = [
            self.aq.find("NAV_HAS_GLIDE_SLOPE:1"),
            self.aq.find("NAV_HAS_GLIDE_SLOPE:2"),
        ]
        self.aq_ground_speed = self.aq.find("GPS_GROUND_SPEED")
        self.aq_title=self.aq.find("TITLE")

    def get_waypoint_distances(self):
        """Get the distance to the previous and next FPL waypoints."""
        try:
            prev_wp_lat = self.aq_prev_wp_lat.value
            prev_wp_lon = self.aq_prev_wp_lon.value
            cur_lat = self.aq_cur_lat.value
            cur_long = self.aq_cur_long.value
            next_wp_lat = self.aq_next_wp_lat.value
            next_wp_lon = self.aq_next_wp_lon.value

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

    def are_angles_aggressive(self):
        """Check to see if pitch and bank angles are "agressive."

        The default values seem to minimize sim rate induced porpoising and
        waddling
        """
        agressive = True
        try:
            pitch = abs(degrees(self.aq_pitch.value))
            bank = abs(degrees(self.aq_bank.value))
            if pitch > self.max_pitch or bank > self.max_bank:
                logging.info(f"Agressive angles detected: {pitch} deg {bank} deg")
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
            vsi = self.aq_vsi.value
            if vsi > self.min_vsi and vsi < self.max_vsi:
                agressive = False
            else:
                logging.info(f"Agressive VS detected: {vsi} ft/s")
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
            sc_autopilot_active = int(self.aq_ap_master.value)
            sc_nav_mode = int(self.aq_nav_mode.value)
            sc_title = self.aq_title.value
            if sc_autopilot_active and sc_nav_mode:
                ap_active = True
            elif ("A320" in str(sc_title) and sc_autopilot_active):
                # Workaround for A320 not reporting AUTOPILOT_NAV1_LOCK like
                # other planes. A320 always reports
                # AUTOPILOT_HEADING_LOCK==True and
                # AUTOPILOT_NAV1_LOCK==False leaving no way to set the
                # autopilot to disable time acceleration.
                ap_active = True
            else:
                ap_active = False
                logging.info("AP not active.")

        except TypeError:
            raise SimConnectDataError()

        return ap_active

    def is_waypoints_valid(self):
        try:
            sc_is_prev_wp_valid = self.aq_is_prev_wp_valid.value
            sc_cur_waypoint_index = self.aq_cur_waypoint_index.value
            sc_num_waypoints = self.aq_num_waypoints.value
            if (
                sc_is_prev_wp_valid
                and sc_num_waypoints > 0
                and sc_cur_waypoint_index <= sc_num_waypoints
            ):
                return True
        except TypeError:
            raise SimConnectDataError()

        logging.info("No valid waypoints detected.")
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
            ground_speed = self.aq_ground_speed.value  # units: meters/sec
            mps_to_nmps = 5.4e-4  # one meter per second to 1 nautical mile per second
            nautical_miles_per_second = ground_speed * mps_to_nmps
            previous_dist = nautical_miles_per_second * self.waypoint_buffer
            next_dist = nautical_miles_per_second * self.waypoint_buffer
            clearance = self.get_waypoint_distances()

            if clearance.prev > previous_dist and clearance.next > next_dist:
                close = False
            else:
                logging.info(
                    f"Close ({previous_dist}, {next_dist}) to waypoint: ({clearance.prev} nm, {clearance.next} nm)"
                )
        except TypeError:
            raise SimConnectDataError()

        return close

    def is_too_low(self, low):
        """Is the plan below `low` AGL"""
        try:
            agl = self.aq_agl.value
            if agl > low:
                return False
        except TypeError:
            raise SimConnectDataError()

        logging.info(f"Plane close to ground: {agl} ft AGL")
        return True

    def is_last_waypoint(self):
        """Is the FMS tagerting the final waypoint?"""
        cur_waypoint_index = self.aq_cur_waypoint_index.value
        num_waypoints = self.aq_num_waypoints.value
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

    def target_descent_fpm(self):
        # https://code7700.com/rot_descent_vvi.htm
        # Convert m/s to nm/min
        ground_speed = (self.aq_ground_speed.value / 0.51444444444444)/60
        # rough calculation. I have also seen (ground_speed/2)*10
        # TODO: Change this based on descent angle
        return ground_speed * 6076.118 * sin(radians(self.degrees_of_descent))


    def arrival_distance(self):
        # https://www.thinkaviation.net/top-of-descent-calculation/
        # height above pattern altitude
        total_descent = abs(self.aq_alt.value - (self.aq_next_wp_alt.value))
        feet_to_nm = 6076.118
        # Solve the triangle to get a chosen degree of descent
        distance = (total_descent/tan(radians(self.degrees_of_descent)))/feet_to_nm
        return distance*self.descent_safety_factor

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

    def is_past_tod(self):
        try:
            distance_to_dest = self.get_waypoint_distances()[1]
            if distance_to_dest < self.arrival_distance():
                return True
        except:
            pass
        return False

    def is_approaching(self):
        """Checks several items to see if we are "arriving" because there are
        different ways a flight plan may be set up.

        These count as arriving:
        1. Within `close`nm of the final waypoint
        2. AGL < low and last waypoint is active
        2. An "approach" is active
        3. "Approach" mode is active on the AP

        More to consider:
        1. NAV has picked up a localizer signal
        2. NAV/GPS has has a glide scope
        """
        approaching = False
        try:
            clearance = self.get_waypoint_distances()
            last = self.is_last_waypoint()
            if last:
                too_low = self.is_too_low(self.min_agl_descent)
            else:
                too_low = False
            autopilot_active = int(self.aq_ap_master.value)
            approach_hold = int(self.aq_approach_hold.value)
            approach_active = int(self.aq_approach_active.value)

            # has_localizer = [int(l.value) for l in self.aq_has_localizer]
            # has_glide_scope = [int(l.value) for l in self.aq_has_glide_scope]
            # print(approach_active, has_localizer, has_glide_scope)
            min_distance = max(self.arrival_distance(), self.destination_distance)
            if last and clearance.next < min_distance:
                logging.info(f"Descent phase detected: {clearance.next} nm, TOD: {self.arrival_distance()}")
                logging.info(f"Target {self.degrees_of_descent} degree FPM: -{self.target_descent_fpm()} nm")
                approaching = True

            if last and too_low:
                logging.info(f"Last waypoint and low")
                approaching = True

            #seconds = ceil(self.get_approach_minutes(descent_fpm, performance_exponent,
            #minimum_time))*60
            seconds = self.min_approach_time * 60
            if self.aq_ete.value < seconds:
                logging.info(f"Less than {seconds/60} minutes from destination")
                approaching = True

            if autopilot_active and approach_hold:
                logging.info("Approach is active or approach hold mode on")
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
        try:
            if flight_stability.is_waypoints_valid():
                if not self.is_ap_active():
                    logging.info("Autopilot not enabled.")
                    stable = 1
                elif self.is_approaching():
                    if self.pause_at_tod and not self.have_paused_at_tod:
                        self.have_paused_at_tod = True
                        logging.info("Pause at TOD.")
                        stable = 0
                    else:
                        stable = 1
                    logging.info("Arrival imminent.")
                elif self.are_angles_aggressive():
                    logging.info("Pitch or bank too high")
                    stable = 1
                elif self.is_too_low(self.min_agl_cruise):
                    logging.info("Too close to ground.")
                    stable = 1
                elif self.is_waypoint_close():
                    # The AP will switch waypoints several miles away to cut corners,
                    # so we slow down far enough away that we don't enter a turn prior
                    # to slowing down (4nm). To keep from speeding up immediately when
                    # a corner is cut we also give until 2.5nm after the switch
                    logging.info("Close to waypoint.")
                    stable = 2
                elif self.is_vs_aggressive():
                    # pitch/bank may be a better/suffcient proxy
                    logging.info("Vertical speed too high.")
                    stable = 2
                else:
                    logging.info("Flight stable")
                    stable = 16
            else:
                logging.warning("No valid flight plan. Stability undefined.")
                stable = 1
        except SimConnectDataError:
            logging.warning("DATA ERROR: DECEL")
            stable = 1

        return stable


class SimRateManager:
    """Manages the game sim rate, and audible annunciation."""

    def __init__(self, sm, config = None):
        self.sm = sm
        self.config = config
        if config is None or config.sections() == []:
            self.min_rate=1
            self.max_rate=4
            self.annunciation =True
        else:
            self.config = config
            self.min_rate = int(self.config['simrate']['min_rate'])
            self.max_rate = int(self.config['simrate']['max_rate'])
            self.annunciation = config.getboolean('simrate', 'annunciation')
        
        self.aq = AircraftRequests(self.sm)
        self.ae = AircraftEvents(self.sm)
        
        self.sc_sim_rate = self.aq.find("SIMULATION_RATE")
        self.increase_sim_rate = self.ae.find("SIM_RATE_INCR")
        self.decrease_sim_rate = self.ae.find("SIM_RATE_DECR")
        self.ae_pause = self.ae.find("PAUSE_ON")
        self.tts_engine = pyttsx3.init()

    def get_sim_rate(self):
        """Get the current sim rate."""
        return self.sc_sim_rate.value

    def say_sim_rate(self):
        """Speak the current sim rate using text-to-speech"""
        if not self.annunciation:
            return

        try:
            simrate = self.get_sim_rate()
            if simrate >= 1.0:
                self.tts_engine.say(f"Sim rate {str(int(simrate))}")
            else:
                self.tts_engine.say(f"Sim rate {str(simrate)}")
            self.tts_engine.runAndWait()
        except TypeError:
            pass

    def pause(self):
        """Pause the sim"""
        self.ae_pause()
        if self.annunciation:
            self.tts_engine.say(f"Paused at todd")
            self.tts_engine.runAndWait()

    def stop_acceleration(self):
        """Decrease the sim rate to the minimum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        while simrate > self.min_rate:
            sleep(2)
            self.decelerate()
            simrate /= 2

    def decelerate(self):
        """Decrease the sim rate, up to some maximum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        if simrate > self.min_rate:
            self.decrease_sim_rate()
        elif simrate < self.min_rate:
            self.increase_sim_rate()

    def accelerate(self):
        """Increase the sim rate, up to some maximum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        if simrate < self.max_rate:
            self.increase_sim_rate()
        elif simrate > self.max_rate:
            self.decrease_sim_rate()


if __name__ == "__main__":
    config = configparser.ConfigParser()
    config.read('config.ini')

    clear = lambda: os.system("cls")  # on Windows System
    connected = False
    while not connected:
        clear()
        try:
            sm = SimConnect()
            connected = True
            logging.info("Connected to simulator.")
        except KeyboardInterrupt:
            quit()
        except Exception as e:
            print(type(e).__name__, e)
            logging.warning("Connection failed, retrying...")
            sleep(5)

    srm = SimRateManager(sm, config)
    flight_stability = FlightStability(sm, config)

    try:
        have_paused_at_tod = False
        while True:
            clear()
            prev_simrate = srm.get_sim_rate()
            try:
                max_stable_rate = min(flight_stability.get_max_sim_rate(),
                                      int(config['simrate']['max_rate']))
                if max_stable_rate is None:
                    raise SimConnectDataError()
                elif max_stable_rate == 0 and not have_paused_at_tod:
                    have_paused_at_tod = True
                    srm.pause()
                elif max_stable_rate > prev_simrate:
                    logging.info("accelerate")
                    srm.accelerate()
                elif max_stable_rate < prev_simrate:
                    logging.info("decelerate")
                    srm.decelerate()
            except TypeError as e:
                logging.warning(e)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except SimConnectDataError:
                logging.warning("DATA ERROR: DECEL")
                if config.getboolean('simrate', 'decelerate_on_simconnect_error'):
                    srm.decelerate()
            finally:
                sleep(0.1)
                new_simrate = srm.get_sim_rate()
                print("SIM RATE: ", new_simrate)
                if prev_simrate != new_simrate:
                    srm.say_sim_rate()
                sleep(2)
    except KeyboardInterrupt:
        pass
    except OSError:
        print("Flight Simulator exited. Shutting down.")
        sys.exit()
    except Exception as e:
        print(type(e).__name__, e)
    finally:
        simrate = srm.get_sim_rate()
        srm.stop_acceleration()
        sleep(1)
        print(srm.get_sim_rate())
        srm.say_sim_rate()
        sm.exit()

    sys.exit()
