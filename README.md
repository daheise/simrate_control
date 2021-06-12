![Simrate_control_stable](https://user-images.githubusercontent.com/5230957/98481107-f5244180-21c5-11eb-93b1-656d278f23da.PNG)

## SimRate Control Theory of Operations

This script is intended to give stable time compression while following a flight
plan on autopilot. At the moment the controller checks the following parameters
before accelerating. Violations will generally lead to sim rate being set to 1x,
with the exception of simply being close to a waypoint, which will only reduce
to 2x.

Simrate Control assumes you are following a well configured flight plan. It
uses waypoint altitudes to determine some sim rate conditions.

Specific cited values for conditions detection are subject to change. Most have
configuration options in the `config.ini`.

**Sim Rate**

Maximum simrate is 4x. Beyond 4x and flight become unstable by the other metrics
and things just waffle back and forth.

[Screenshot](https://user-images.githubusercontent.com/5230957/98481107-f5244180-21c5-11eb-93b1-656d278f23da.PNG)

**Flight Plan**

A flight plan must be loaded into the autopilot.

[Screenshot](https://user-images.githubusercontent.com/5230957/98481104-f5244180-21c5-11eb-8a26-9f6282638572.PNG)

**Autopilot Settings**

Autopilot must be turned on and in lateral navigation ([L]NAV) mode.

[Screenshot](https://user-images.githubusercontent.com/5230957/98481102-f48bab00-21c5-11eb-946e-7a2e3a5653b7.PNG)

**Altitude**

Acceleration will only function 1000ft AGL or higher.

[Screenshot](https://user-images.githubusercontent.com/5230957/98481101-f48bab00-21c5-11eb-8dca-61c6f891aeb9.PNG)

**Waypoint Proximity**

When the plane is 40 seconds away from a waypoint (approaching or departing)
based on ground speed, the acceleration will reduce to 2x. This allows for time
acceleration through a waypoint that does not involve a turn, but will minimize
instability cause by the delay between the start of a turn and detection.

[Screenshot](https://user-images.githubusercontent.com/5230957/98481111-f6556e80-21c5-11eb-8479-bd854d0e857b.PNG)

**Pitch and Bank**

Pitch and bank must have minimal deviations from straight and level.

[Screenshot](https://user-images.githubusercontent.com/5230957/98481106-f5244180-21c5-11eb-96d6-f6f3647a2cbf.PNG)

**Flight Level Change and Approach**

If the next waypoint altitude is lower than the plane, then the plane must
be far enough away/at an altitude that would allow the plane to execute a
configured glidescope descent. Conversely, if the next waypoint is a higher
altitude than the plane, then the plane must be far enough away/at an altitude
that would allow the plane to execute a configued angle of ascent. There is
also a configurable additional distance before these conditions would occur
to allow you to set up your autopilot for ascent/descent. Ascent detection can
be turned off.

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

## TODO/Known Issues

* The A320 does not report GPS_NAV1_LOCK like the other aircraft. As a
  work-around, if the AP is on then time acceleration is enabled.
  [Issue #2](https://github.com/daheise/msfs_utils/issues/2)
* I would like to give this a GUI, but that's not high on my priority list.
  [Issue #3](https://github.com/daheise/msfs_utils/issues/3)
* Any addon aircraft that does not provide the simvars used to detect
  acceleration parmeters (e.g. Working Title CJ4) will not accelerate or will
  behave in unexpected ways.
* ATC will tell you to fly at altitudes not in your flight plan. If you
  follow ATC, then flight level change detection may reduce your simrate.
* The in game flight planner will sometimes give you routes with slightly
  lower or higher altitude waypoints interspersed during cruise. The will
  trigger flight level change detection.
* Approach waypoints are all set to 0ft. Therefore, the flight level change
  detector will try to make room for you to get to the ground at every
  waypoint. This leads to some rubber banding when using approaches. There is a
  heuristic that uses plane AGL instead of 0ft MSL to reduce this effect
  somewhat.

Tested on: MSFS 2020 1.16.2.0

### Acknowledgements

Inspired by the [SimRateBandit](https://github.com/dga711/msfs-simratebandit).

### Other scripts

* `load_tmp_fpl.py`: This script will look int a particular folder and load the
  most recently written `*.pln` to the aircraft FMS. Has a lot of caveats
  because of MSFS support for SimConnect functions. See also
  [here](https://github.com/albar965/littlenavmap/issues/35#issuecomment-716013932),
  which is the feature I'm actually wanting instead of this scripts.
