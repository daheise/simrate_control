from SimConnect import *
from geopy import distance
import logging
from time import sleep
import os
import pyttsx3
from collections import namedtuple
from math import radians, degrees

logging.basicConfig(level=logging.DEBUG)
LOGGER = logging.getLogger(__name__)
LOGGER.info("START")

# Go above 4 at your own risk. Lots of porpoising.
MAX_SIM_RATE=4
MIN_SIM_RATE=1

# If VSI crosses these values, slow down
# Units are feet/s
VSI_MAX = 5000 
VSI_MIN = -2000

class SimConnectDataError(Exception):
    pass

# A simple structure to hold distance from the previous and next waypoints
WaypointClearance = namedtuple("WaypointClearances", 'prev next')

class FlightStability():
    def __init__(self, sm):
        self.sm = sm
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
        self.aq_pitch = self.aq.find("PLANE_PITCH_DEGREES")
        self.aq_bank = self.aq.find("PLANE_BANK_DEGREES")
        self.aq_vsi = self.aq.find("VERTICAL_SPEED")
        self.aq_ap_master = self.aq.find("AUTOPILOT_MASTER")
        self.aq_is_prev_wp_valid = self.aq.find("GPS_WP_PREV_VALID")
        self.aq_cur_waypoint_index = self.aq.find("GPS_FLIGHT_PLAN_WP_INDEX")
        self.aq_num_waypoints = self.aq.find("GPS_FLIGHT_PLAN_WP_COUNT")
        self.aq_agl = self.aq.find("PLANE_ALT_ABOVE_GROUND")
        self.aq_approach_hold = self.aq.find("AUTOPILOT_APPROACH_HOLD")
        self.aq_approach_active = self.aq.find("GPS_IS_APPROACH_ACTIVE")
        #self.aq_nav_has_nav = [self.aq.find("NAV_HAS_NAV:1"), self.aq.find("NAV_HAS_NAV:2")]
        self.aq_has_localizer = [self.aq.find("NAV_HAS_LOCALIZER:1"), self.aq.find("NAV_HAS_LOCALIZER:2")]
        self.aq_has_glide_scope = [self.aq.find("NAV_HAS_GLIDE_SLOPE:1"), self.aq.find("NAV_HAS_GLIDE_SLOPE:2")]
        self.aq_nav_mode = self.aq.find("AUTOPILOT_NAV1_LOCK")
        self.aq_ground_speed = self.aq.find("GPS_GROUND_SPEED")

    def get_waypoint_distances(self):
        """Get the distance to the previous and next FPL waypoints.
        """
        try:
            prev_wp_lat = self.aq_prev_wp_lat.value
            prev_wp_lon = self.aq_prev_wp_lon.value
            cur_lat = self.aq_cur_lat.value
            cur_long = self.aq_cur_long.value
            next_wp_lat = self.aq_next_wp_lat.value
            next_wp_lon = self.aq_next_wp_lon.value

            prev_clearance = distance.distance((prev_wp_lat, prev_wp_lon),
                                (cur_lat, cur_long)).nm
            next_clearance = distance.distance((next_wp_lat, next_wp_lon),
                (cur_lat, cur_long)).nm
            
            return(WaypointClearance(prev_clearance, next_clearance))
        except TypeError:
            raise SimConnectDataError()


    def are_angles_aggressive(self, max_pitch = 7, max_bank = 5):
        """Check to see if pitch and bank angles are "agressive."

        The default values seem to minimize sim rate induced porpoising and
        waddling
        """
        agressive = True
        try:
            pitch = abs(degrees(self.aq_pitch.value))
            bank = abs(degrees(self.aq_bank.value))
            if (pitch > max_pitch or bank > max_bank):
                logging.info(f"Agressive angles detected: {pitch} deg {bank} deg")
                agressive = True
            else:
                agressive = False

        except TypeError:
            raise SimConnectDataError()

        return agressive

    def is_vs_aggressive(self, min_vs = -300, max_vs = 500):
        """Check to see if the vertical speed is "aggressively" high or low.

        Values are intended to detect and stop porposing. However, 
        `are_angles_aggressive` may be a better proxy.
        """
        agressive = True
        try:
            vsi = self.aq_vsi.value
            if(vsi > min_vs and vsi < max_vs):
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
            sc_autopilot_active = self.aq_ap_master.value
            sc_nav_mode = self.aq_nav_mode.value
            if(sc_autopilot_active and sc_nav_mode):
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
            if (sc_is_prev_wp_valid and
                sc_num_waypoints > 0 and
                sc_cur_waypoint_index <= sc_num_waypoints):
                return True
        except TypeError:
            raise SimConnectDataError()
        
        logging.info("No valid waypoints detected.")
        return False

    def is_waypoint_close(self, prev_seconds = 60, next_seconds = 60):
        """Check is a waypoint is close by.

        In the future, the definition of "close" could be parameterized on
        bearing to/out of the waypoint. Greater degrees of turn
        need more buffer.

        IMPORTANT: Buffer size is decided largely by the FMS switching waypoints early
        to cut corners.
        """
        close = True
        try:
            ground_speed = self.aq_ground_speed.value # units: meters/sec
            mps_to_nmps = 5.4e-4 # one meter per second to 1 nautical mile per second
            nautical_miles_per_second = ground_speed * mps_to_nmps
            previous_dist = nautical_miles_per_second * prev_seconds
            next_dist = nautical_miles_per_second * next_seconds
            clearance = self.get_waypoint_distances()

            if (clearance.prev > previous_dist and clearance.next > next_dist):
                close = False
            else:
                logging.info(f"Close ({previous_dist}, {next_dist}) to waypoint: ({clearance.prev} nm, {clearance.next} nm)")
        except TypeError:
            raise SimConnectDataError()

        return close

    def is_too_low(self, low:int = 1000):
        """Is the plan below `low` AGL
        """
        try:
            agl = self.aq_agl.value
            if (agl > low):
                return False
        except TypeError:
            raise SimConnectDataError()
        
        logging.info(f"Plane close to ground: {agl} ft AGL")
        return True

    def is_last_waypoint(self):
        """ Is the FMS tagerting the final waypoint?
        """
        cur_waypoint_index = self.aq_cur_waypoint_index.value
        num_waypoints = self.aq_num_waypoints.value
        try:
            # Don't really understand the indexing here...
            # Indexes skip by twos, so you get 2, 4, 6...N-1, 
            # i.e. the last waypoint will be 7 not 8.
            # I don't know why the last waypoint violates the pattern.
            if (cur_waypoint_index < (num_waypoints - 1)):
                return False
        except TypeError:
            raise SimConnectDataError()

        
        return True

    def is_approaching(self, close:float=25.0, low:int=3000):
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
                too_low = self.is_too_low(low)
            else:
                too_low = False
            autopilot_active = int(self.aq_ap_master.value)
            approach_hold = int(self.aq_approach_hold.value)
            approach_active = int(self.aq_approach_active.value)
            
            #has_localizer = [int(l.value) for l in self.aq_has_localizer]
            #has_glide_scope = [int(l.value) for l in self.aq_has_glide_scope]
            #print(approach_active, has_localizer, has_glide_scope)

            if (last and clearance.next < close):
                logging.info(f"Last waypoint and close: {clearance.next} nm")
                approaching = True
            
            if (last and too_low):
                logging.info(f"Last waypoint and low")
                approaching = True

            if(autopilot_active and (approach_active or approach_hold)):
                logging.info("Approach is active or approach hold mode on")
                approaching = True

            #if(has_localizer[0] or has_localizer[1] or
            #    has_glide_scope[0] or has_glide_scope[1]):
            #    logging.info("Localizer or glide scope detected")
            #    approaching = True

        except TypeError:
            raise SimConnectDataError()

        return approaching

    def is_flight_stable(self):
        """Is the flight stable enough to increase the sim rate?

        Looks at a lot of factors.
        """
        stable = False
        try:
            if flight_stability.is_waypoints_valid():
                if (not self.is_ap_active()):
                    logging.info("Autopilot not enabled.")
                    stable = False
                elif(self.is_approaching()):
                    logging.info("Arrival imminent.")
                    stable = False
                elif(self.is_waypoint_close()):
                    # The AP will switch waypoints several miles away to cut corners,
                    # so we slow down far enough away that we don't enter a turn prior
                    # to slowing down (4nm). To keep from speeding up immediately when
                    # a corner is cut we also give until 2.5nm after the switch
                    logging.info("Close to waypoint.")
                    stable = False
                elif(self.is_too_low(1000)):
                    logging.info("Too close to ground.")
                    stable = False
                elif(self.are_angles_aggressive()):
                    logging.info("Pitch or bank too high")
                    stable = False
                elif(self.is_vs_aggressive(VSI_MIN, VSI_MAX)):
                    # pitch/bank may be a better/suffcient proxy
                    logging.info("Vertical speed too high.")
                    stable = False
                else:
                    logging.info("Flight stable")
                    stable = True
            else:
                logging.warning("No valid flight plan. Stability undefined.")
        except SimConnectDataError:
            logging.warning("DATA ERROR: DECEL")
            stable = False

        return stable



class SimRateManager():
    """Manages the game sim rate, and audible annuciations.
    """
    def __init__(self, sm, min_rate=1, max_rate=4):
        self.sm = sm
        self.aq = AircraftRequests(self.sm)
        self.ae = AircraftEvents(self.sm)
        self.min_rate = min_rate
        self.max_rate = max_rate
        self.sc_sim_rate = self.aq.find("SIMULATION_RATE")
        self.increase_sim_rate = self.ae.find("SIM_RATE_INCR")
        self.decrease_sim_rate = self.ae.find("SIM_RATE_DECR")
        self.tts_engine = pyttsx3.init()

    def get_sim_rate(self):
        """Get the current sim rate.
        """
        return self.sc_sim_rate.value

    def say_sim_rate(self):
        """Speak the current sim rate using text-to-speech
        """
        try:
            simrate = self.get_sim_rate()
            self.tts_engine.say(f"Sim rate {str(int(simrate))}")
            self.tts_engine.runAndWait()
        except TypeError:
            pass

    def stop_acceleration(self):
        """Decrease the sim rate to the minimum
        """
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        while simrate > self.min_rate:
            sleep(2)
            self.decelerate()
            simrate /= 2


    def decelerate(self):
        """Decrease the sim rate, up to some maximum
        """
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        if simrate > self.min_rate:
            self.decrease_sim_rate()
        elif simrate < self.min_rate:
            self.increase_sim_rate()

    def accelerate(self):
        """Increase the sim rate, up to some maximum
        """
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        if simrate < self.max_rate:
            self.increase_sim_rate()
        elif simrate > self.max_rate:
            self.decrease_sim_rate()

connected = False
while not connected:
    try:
        sm = SimConnect()
        connected = True
        logging.info("Connected to simulator.")
    except KeyboardInterrupt:
        quit()
    except:
        logging.warning("Connection failed, retrying...")
        sleep(5)

srm = SimRateManager(sm, MIN_SIM_RATE, MAX_SIM_RATE)
flight_stability = FlightStability(sm)

try:
    while True:
        prev_simrate = srm.get_sim_rate()
        try:
            if flight_stability.is_flight_stable():
                logging.info("accelerate")
                srm.accelerate()
            else:
                logging.info("decelerate")
                srm.decelerate()
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except SimConnectDataError:
            logging.warning("DATA ERROR: DECEL")
            srm.decelerate()
        finally:
            sleep(2)
            new_simrate = srm.get_sim_rate()
            if prev_simrate != new_simrate:
                srm.say_sim_rate()
                print("SIM RATE: ", prev_simrate, new_simrate)
        
except Exception as e:
    print(e)
finally:
    simrate = srm.get_sim_rate()
    srm.stop_acceleration()
    sleep(1)
    print(srm.get_sim_rate())
    srm.say_sim_rate()
    sm.exit()

quit()