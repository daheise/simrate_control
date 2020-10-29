These are some scripts I use to simplify some of my simming. These
aren't intended for production, more like personal notes.

* `load_tmp_fpl.py`: This script will look int a particular folder and load the most recently
  written `*.pln` to the aircraft FMS. Has a lot of cavets because of MSFS support for
  SimConnect functions. See also [here](https://github.com/albar965/littlenavmap/issues/35#issuecomment-716013932),
  which is the feature I'm actually wanting instead of this scripts.
* `simrate_control.py`: Inspired by the
  [SimRateBandit](https://github.com/dga711/msfs-simratebandit), I wanted the
  functionality more tailored to my own preferences. This may get receive more
  polish and get more intentional sharing in the future.

## SimRate Control Theory of Operations

This script is intended to give stable time compression while following a flight
plan on autopilot. At the moment the controller checks the following parameters before
accelerating. Violations will generally lead to sim rate being set to 1x, with
the exception of simply being close to a waypoint, which will only reduce to 2x.

Specific cited values for conditions detection are subject to change.

**Sim Rate**

Maximum simrate is 4x. Beyond 4x and flight become unstable by the
other metrics and things just waffle back and forth.

**Flight Plan**

A flight plan must be loaded into the autopilot.

**Autopilot Settings**

Autopilot must be turned on and in lateral navigation
(NAV) mode.

**Approach**

The plane must be far enough away from the a destination. An
approach based on a few conditions. User should still be mindful of the
potential for an undetected arrival or late arrival detection.

1. The last leg of the flight plan is active, and the plane is 3000ft AGL or
   lower.
2. ETE to destination is less than 10 minutes.
3. An approach procedure is active.
4. Approach mode is active.

**Pitch and Bank**

Pitch and bank must have minimal deviations from straight and
level.

**Altitude**

Acceleration will only function 1000ft AGL or higher.


**Waypoint Proximity** When the plan is 40 seconds away from a waypoint
(approaching or departing) based on ground speed, the acceleration will reduce
to 2x. This allows for accelertion through a waypoint that does not involve a
turn, but will minimize instability cause by the delay between the start of a
turn and detection.

## Quickstart SimRate Control
1. Download the release zip file.
2. Unzip in the location of your choice
3. Run simrate_control.exe

## Building from Source
```
pip install -r requirements.txt
pyinstaller .\simrate_control.spec
```

**KNOWN BUILD ISSUE**
PyInstaller searches for a file named `SimConnect/SimConnect.dllc`, but the dll is copied as `SimConnect/SimConnect.dll`. I don't know why PyInstaller is looking for an incorrect file extension. This can be fixed by renaming the file in `dist/SimConnect/SimConnect.dll` after
building.

I can't upload this as a single EXE until I figure out how to fix this.

## TODO/Known Issues

* I would like to give this a GUI, but that's not high on my priority list.
* I would like better handling of arrivals and user input on how it feels. 10 minutes could be too long, or too short.
* I have only tested this in a few GA planes. Feedback welcome.
* Move specific detection threshold values to a configuration file.
* Come up with a better name