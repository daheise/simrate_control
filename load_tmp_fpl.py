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
sm = SimConnect()
aq = AircraftRequests(sm)

sleep(1)
title=aq.find("TITLE")
print(title.value)
print(sm.load_flight_plan(plan))
#print(sm.load_flight_plan("C:/Users/David Heise/AppData/Local/Packages/Microsoft.FlightSimulator_8wekyb3d8bbwe/LocalState/VFR Blairsville (KDZJ) to Howard Pvt (GA02).pln"))
sleep(2)
sm.exit()
quit()