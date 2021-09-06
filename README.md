![Simrate_control_stable](https://user-images.githubusercontent.com/5230957/132078269-3588d11b-978a-4110-85bc-a634f18469c8.PNG)

## SimRate Control Theory of Operations

This project is increases the sim rate while following a flight plan on
autopilot. At the moment the controller checks the parameters below before
accelerating. Violations will generally lead to sim rate being set to 1x,
although some conditions will only reduce to 2x, e.g. being close to a waypoint,
gentle ascents/descents.

The window provides a number of data outputs that may be useful for acheiving
maximum acceleration. It also has the convience feature up updating your
barometer setting at every simrate change. The UI is unstable at this time, both
in content and location of information.

Simrate Control assumes you are following a well configured GPS flight plan. It
uses waypoint altitudes to determine some sim rate conditions.

Specific cited values for condition detection are subject to change. Most have
configuration options in the `config.ini`.

**Sim Rate**

Maximum simrate is 4x. Beyond 4x and flight become unstable by the other metrics
and things just waffle back and forth.

**Flight Plan**

A flight plan must be loaded into the autopilot.

**Autopilot Settings**

Autopilot must be turned on and in lateral navigation ([L]NAV) mode.

**Altitude**

Acceleration will only function 1000ft AGL or higher.

**Waypoint Proximity**

When the plane is 40 seconds away from a waypoint (approaching or departing)
based on ground speed, the acceleration will reduce to 2x. This allows for time
acceleration through a waypoint that does not involve a turn, but will minimize
instability cause by the delay between the start of a turn and detection.

**Pitch and Bank**

Pitch and bank must have minimal deviations from straight and level.

**Flight Level Change and Approach**

If the next waypoint altitude is lower than the plane, then the plane must be
far enough away (measured in time) and at a vertical speed that would allow the
plane to execute a configured glidescope descent and meet the target altitude.
This functions similarly for ascent. There is also a configurable additional
time before these conditions would occur to allow you to set up your autopilot
for ascent/descent. Ascent detection can be turned off.

The plane must be far enough away from the a destination. An approach is
detected based on a few conditions. User should still be mindful of the
potential for an undetected arrival or late arrival detection. If you descend
aggressively or drastically change ground speed, you may see some simrate rubber
banding as you cross thresholds for flight level change estimation and ETE
estimates.

There is also an option in the configuration file to pause the game at initial
approach detection. This is disabled by default. Only set this to "True" if you
have a key bound to the in game "SET PAUSE OFF" binding. The binding IS NOT set
by default in game. The simrate controller does not unpause the game. The
"Toggle Pause" binding does not suffice.

1. The last leg of the flight plan is active, and the plane is lower than a configured AGL.
[Screenshot](https://user-images.githubusercontent.com/5230957/98481103-f5244180-21c5-11eb-899c-8ea748daad4c.PNG)
2. Distance to waypoint (or destination) is less than an estimated distance needed for a flight level change.
[Screenshot](https://user-images.githubusercontent.com/5230957/98481109-f5bcd800-21c5-11eb-9403-062f325c4a7b.PNG)
3. ETE to destination is less than a configured minimum.
[Screenshot](https://user-images.githubusercontent.com/5230957/98481108-f5bcd800-21c5-11eb-8096-437def4d4939.PNG)
4. Approach mode is active.
[Screenshot](https://user-images.githubusercontent.com/5230957/98481102-f48bab00-21c5-11eb-946e-7a2e3a5653b7.PNG)

## Quickstart SimRate Control

1. Download the release zip file.
2. Unzip in the location of your choice
3. Run simrate_control.exe

## Configuration

A configuration file to modify various thresholds is available in
`simrate_control/config.ini`.

## Building from Source

```
pip install -r requirements.txt
pyinstaller .\simrate_control.spec
```

## Known Issues

* This utility is only tested extensively with default planes. Mods and third
  party planes often do not have the simvars this utilty relies on, and may
  behave in unexpected ways or not at all. For example, the FlyByWire A320NX
  does not work at all, because it uses a multitude of custom simvars. The
  Working Title mods can also cause inconsistent behavior due to issues with
  flight plan synchronization.
* Some planes do not report LNAV being on/off. See config.ini
  `nav_mode_guarded`.
* ATC will tell you to fly at altitudes not in your flight plan. If you follow
  ATC, then flight level change detection may reduce your simrate. See
  config.ini `waypoint_minimum_agl` for a partial work-around.
* At every simrate change the heading bug is selected for +/- operations. This
  was done to prevent accidentally setting extreme simrate when using +/- during
  a simrate transition. Selecting the heading bug is the least problematic
  selection I could come up with that would get the active selection off of
  simrate.

Tested on: MSFS 2020 1.18.15.0

### Acknowledgements

Inspired by the [SimRateBandit](https://github.com/dga711/msfs-simratebandit).

### Other scripts

* `load_tmp_fpl.py`: This script will look int a particular folder and load the
  most recently written `*.pln` to the aircraft FMS. Has a lot of caveats
  because of MSFS support for SimConnect functions. See also
  [here](https://github.com/albar965/littlenavmap/issues/35#issuecomment-716013932),
  which is the feature I'm actually wanting instead of this scripts.
