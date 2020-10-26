from SimConnect import *
import logging
from SimConnect.Enum import *
from time import sleep
import os

FPL_DIR=os.getenv("LOCALAPPDATA") + "/Packages/Microsoft.FlightSimulator_8wekyb3d8bbwe/LocalState/"
plns = [ f for f in os.listdir(FPL_DIR) if f.endswith('.pln')]
plan=sorted(plns, key=lambda name:
                  os.path.getmtime(os.path.join(FPL_DIR, name)))[-1]
print(os.path.join(FPL_DIR,plan))

logging.basicConfig(level=logging.DEBUG)
LOGGER = logging.getLogger(__name__)
LOGGER.info("START")
# time holder for inline commands
# creat simconnection and pass used user classes
sm = SimConnect()
aq = AircraftRequests(sm)
ae = AircraftEvents(sm)

reload_craft= ae.find("RELOAD_USER_AIRCRAFT")
sim_reset= ae.find("SIM_RESET")
reload = ae.find("RELOAD_PANELS")
save_sit=ae.find("SITUATION_SAVE")
reset_sit=ae.find("SITUATION_RESET")

reload()
sleep(1)
title=aq.find("TITLE")
callsign=aq.find("ATC_AIRLINE")
flight_number=aq.find("ATC_FLIGHT_NUMBER")
print(title.value)
callsign.set("Jynx".encode(encoding='UTF-8',errors='strict'))
flight_number.set("1331".encode(encoding='UTF-8',errors='strict'))
print(callsign.value)
print(flight_number.value)
print(sm.load_flight_plan(plan))
#print(sm.load_flight_plan("C:/Users/David Heise/AppData/Local/Packages/Microsoft.FlightSimulator_8wekyb3d8bbwe/LocalState/VFR Blairsville (KDZJ) to Howard Pvt (GA02).pln"))
sleep(1)
reload()
sleep(1)
sm.exit()
quit()