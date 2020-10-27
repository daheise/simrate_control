from SimConnect import *
from geopy import distance
import logging
from SimConnect.Enum import *
from time import sleep
import os
import pyttsx3

def say_sim_rate(simrate):
    """Speak the current sim rate using text-to-speech
    """
    simrate = sc_sim_rate.value
    engine.say(f"Sim rate {str(simrate)}")
    engine.runAndWait()

def stop_acceleration(simrate):
    """Decrease the sim rate to the minimum
    """
    if simrate is None:
        return
    while simrate > MIN_RATE:
        sleep(2)
        decelerate(simrate)
        simrate /= 2


def decelerate(simrate, minimum=1):
    """Decrease the sim rate, up to some maximum
    """
    if simrate is None:
        return
    if simrate > minimum:
        decrease_sim_rate()
    elif simrate < minimum:
        increase_sim_rate()

def accelerate(simrate, maximum):
    """Increase the sim rate, up to some maximum
    """
    if simrate is None:
        return
    if simrate < maximum:
        increase_sim_rate()
    elif simrate > maximum:
        decrease_sim_rate()

def check_lat_lons(waypoint_coords):
    """I don't remember what this was going to be for
    """
    pass

logging.basicConfig(level=logging.DEBUG)
LOGGER = logging.getLogger(__name__)
LOGGER.info("START")
# time holder for inline commands
# creat simconnection and pass used user classes
MAX_RATE=4
MIN_RATE=1
sm = SimConnect()
sleep(1)

aq = AircraftRequests(sm)
ae = AircraftEvents(sm)

is_prev_wp_valid = aq.find("GPS_WP_PREV_VALID")
prev_wp_lat = aq.find("GPS_WP_PREV_LAT")
prev_wp_lon = aq.find("GPS_WP_PREV_LON")
cur_lat = aq.find("GPS_POSITION_LAT")
cur_long = aq.find("GPS_POSITION_LON")
next_wp_lat = aq.find("GPS_WP_NEXT_LAT")
next_wp_lon = aq.find("GPS_WP_NEXT_LON")
height = aq.find("PLANE_ALT_ABOVE_GROUND")
sc_sim_rate = aq.find("SIMULATION_RATE")
increase_sim_rate = ae.find("SIM_RATE_INCR")
decrease_sim_rate = ae.find("SIM_RATE_DECR")
cur_waypoint_index = aq.find("GPS_FLIGHT_PLAN_WP_INDEX")
num_waypoints = aq.find("GPS_FLIGHT_PLAN_WP_COUNT")
approach_active = aq.find("GPS_IS_APPROACH_ACTIVE")
autopilot_active = aq.find("AUTOPILOT_MASTER")
sc_vsi = aq.find("VERTICAL_SPEED")
# Waypoint distance buffer - before and after
# In the future, a fancy calculation involving ground
# speed may be useful
# Units are nm
WAYPOINT_BUFFER=1
# If VSI crosses these values, slow down
# Units are feet/s
VSI_MAX = 500 
VSI_MIN = -300
distance_from_wp = None
ground_speed = 0

print(str(is_prev_wp_valid.value))

engine = pyttsx3.init()


prev_simrate = None
new_simrate = None #sc_sim_rate.value
#say_sim_rate()
try:
    while True:
        cur_height = height.value
        cur_vsi = sc_vsi.value
        prev_simrate = sc_sim_rate.value
        if is_prev_wp_valid.value:
            try:
                prev_clearance = distance.distance((prev_wp_lat.value, prev_wp_lon.value),
                    (cur_lat.value, cur_long.value)).nm
                next_clearance = distance.distance((next_wp_lat.value, next_wp_lon.value),
                    (cur_lat.value, cur_long.value)).nm
                #print(cur_waypoint_index.value, num_waypoints.value)
                print(prev_clearance, next_clearance)
                if (not autopilot_active.value):
                    print("Autpilot not enabled. Stopping acceleration.")
                    stop_acceleration(prev_simrate)
                elif(cur_waypoint_index.value >= (num_waypoints.value - 1) and next_clearance < 25 and cur_height <= 3000):
                    # Don't really understand the counting here...
                    print("Arrival imminent. Stopping acceleration.")
                    #engine.say(f"On last leg. Prepare for landing.")
                    #engine.runAndWait()
                    stop_acceleration(prev_simrate)
                elif(approach_active.value):
                    print("Approach activated. Stopping acceleration.")
                    stop_acceleration(prev_simrate)
                elif(prev_clearance < 1 or next_clearance < 1):
                    print("Close to waypoint. Stopping acceleration.")
                    stop_acceleration(prev_simrate)
                elif(cur_height < 1000):
                    print("Too close to ground. Stopping acceleration.")
                    stop_acceleration(prev_simrate)
                elif(cur_vsi < VSI_MIN or cur_vsi > VSI_MAX):
                    print("Vertical speed too high. Stopping acceleration.")
                    decelerate(prev_simrate)
                else:
                    print("WARP SPEED!")
                    accelerate(prev_simrate, MAX_RATE)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                print("DATA ERROR: DECEL")
                decelerate(prev_simrate)
            finally:
                new_simrate = sc_sim_rate.value
                if prev_simrate != new_simrate:
                    say_sim_rate(new_simrate)
                    print("SIM RATE: ", new_simrate)
        else:
            print("No valid waypoints. Travel not possible.")
        sleep(2)

finally:
    simrate = sc_sim_rate.value
    stop_acceleration(simrate)
    print(sc_sim_rate.value)
    say_sim_rate(simrate)
    sm.exit()

quit()