from SimConnect import *
from geopy import distance
import logging
from SimConnect.Enum import *
from time import sleep
import os
import pyttsx3

FPL_DIR="C:/Users/David Heise/AppData/Local/Packages/Microsoft.FlightSimulator_8wekyb3d8bbwe/LocalState/"
plns = [ f for f in os.listdir(FPL_DIR) if f.endswith('.pln')]
plan=sorted(plns, key=lambda name:
                  os.path.getmtime(os.path.join(FPL_DIR, name)))[-1]
print(os.path.join(FPL_DIR,plan))

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
cur_sim_rate = aq.find("SIMULATION_RATE")
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
def say_sim_rate():
    simrate = cur_sim_rate.value
    engine.say(f"Sim rate {str(simrate)}")
    engine.runAndWait()

def stop_acceleration():
    simrate = cur_sim_rate.value
    if simrate is None:
        return
    while simrate > MIN_RATE:
        simrate = cur_sim_rate.value
        sleep(2)
        decelerate()

def decelerate():
    simrate = cur_sim_rate.value
    if simrate is None:
        return
    if simrate > MIN_RATE:
        decrease_sim_rate()

def accelerate():
    simrate = cur_sim_rate.value
    if simrate is None:
        return
    if simrate < MAX_RATE:
        increase_sim_rate()

def check_lat_lons(waypoint_coords):
    pass

#while height.value > 100:
prev_simrate = None
new_simrate = cur_sim_rate.value
say_sim_rate()
try:
    while True:
        cur_height = height.value
        cur_vsi = sc_vsi.value
        prev_simrate = cur_sim_rate.value
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
                    stop_acceleration()
                elif(cur_waypoint_index.value >= (num_waypoints.value - 1) and next_clearance < 25 and cur_height <= 3000):
                    # Don't really understand the counting here...
                    print("Arrival imminent. Stopping acceleration.")
                    #engine.say(f"On last leg. Prepare for landing.")
                    #engine.runAndWait()
                    stop_acceleration()
                elif(approach_active.value):
                    print("Approach activated. Stopping acceleration.")
                    stop_acceleration()
                elif(prev_clearance < 1 or next_clearance < 1):
                    print("Close to waypoint. Stopping acceleration.")
                    stop_acceleration()
                elif(cur_height < 1000):
                    print("Too close to ground. Stopping acceleration.")
                    stop_acceleration()
                elif(cur_vsi < VSI_MIN or cur_vsi > VSI_MAX):
                    print("Vertical speed too high. Stopping acceleration.")
                    stop_acceleration()
                else:
                    print("WARP SPEED!")
                    accelerate()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                print("DATA ERROR: DECEL")
                decelerate()
            finally:
                new_simrate = cur_sim_rate.value
                if prev_simrate != new_simrate:
                    say_sim_rate()
                    print("SIM RATE: ", cur_sim_rate.value)
        else:
            print("No valid waypoints. Travel not possible.")
        sleep(2)

finally:
    stop_acceleration()
    print(cur_sim_rate.value)
    say_sim_rate()
    sm.exit()

quit()