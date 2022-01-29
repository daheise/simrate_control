![Simrate_control_stable](https://user-images.githubusercontent.com/5230957/132078269-3588d11b-978a-4110-85bc-a634f18469c8.PNG)

# Get-there-itis Simrate Control

This project is increases the sim rate while following a flight plan on
autopilot. At the moment the controller checks the parameters below before
accelerating. Violations will generally lead to sim rate being set to 1x,
although some conditions will only reduce to 2x, e.g. being close to a waypoint,
gentle ascents/descents.

The window provides a number of data outputs that may be useful for achieving
maximum acceleration. It also has the convenience feature up updating your
barometer setting at every simrate change. The UI is unstable at this time, both
in content and location of information.

Simrate Control assumes you are following a well configured GPS flight plan. It
uses waypoint altitudes to determine some sim rate conditions.

Specific cited values for condition detection are subject to change. Most have
configuration options in the `config.ini`.

## Dependencies

### MobiFlight

This utility depends on MobiFlight Event Module installed with the [MobiFlight
Connector](https://www.mobiflight.com/en/download.html). Ensure the package
`mobiflight-event-module` is installed in your community folder before running
this utility. This dependency allows reading of L-vars needed for add-on
aircraft, e.g. Working Title CJ4.

![mobiflight-event-module installation](https://user-images.githubusercontent.com/5230957/151675415-4cb7d144-3301-43e5-b51a-7936cd29800f.png)

## Simrate Contraints

Many of the parameters below are configurable. Text that `looks like this`
refers to a parameter that can be found in the config.ini.

### Maximum, Cautious, and Minimum Sim Rate

The maximum, cautious, and minimum simrates are configurable(`max_rate`,
`cautious_rate`, `min_rate`). During straight and level flight, the maximum
simrate (default 4x) will be selected. During maneuvers the simrate will reduce
to the cautious simrate (default 2x). During critical flight phases (namely
ascent and approach) the minimum simrate will be selected (default 1x).

It is suggested not to increase the maximum rate beyond 4x, as the autopilot has
trouble maintain track and level flight. Decreasing the minimum simrate is also
not recommended. Default configuration parameters are optimized for a 4x maximum
simrate.

### Flight Plan

A flight plan must be loaded into the autopilot from the world map. See the
[pinned issues](https://github.com/daheise/simrate_control/issues) for some
workarounds for some mods/third party planes.

**Autopilot Settings**

Autopilot must be turned on and in lateral navigation ([L]NAV) mode. There is a
configuration option (`nav_mode_guarded`) to disable the LNAV constraint.

**Altitude**

Acceleration will only function above a configured (`min_agl_cruise`) altitude
above ground level (AGL).

**Waypoint Proximity**

When the plane is `waypoint_buffer` seconds away from a waypoint (regardless of
direction) based on ground speed, the acceleration will reduce to the cautious
simrate. This allows for time acceleration through a waypoint that does not
involve a turn, but will minimize instability caused by accelerated maneuvers.

**Pitch and Bank**

Pitch and bank must have minimal deviations (`max_bank`, `max_pitch`) from
straight and level. If the constraint is violated, simrate will reduce to the
cautious simrate.

**Flight Level Change and Approach**

By default waypoint altitudes are used to detect needed flight level changes
(FLC), based on the configured ascent and descent slopes (`angle_of_climb`,
`degrees_of_descent`). If `waypoint_vnav = False` then the only constraint will
be the top of descent to the destination based on current ground speed and
altitude. Simrate will be reduced to the minimum at `descent_safety_factor`
seconds before the FLC is needed to give time to set up for the FLC. If the
vertical speed is equal to or better than the "Required FPM" then acceleration
will be reenabled. The ascent constraint can be turned off separately
(`decel_for_climb`).

An approach is detected based on a few conditions.

1. Whether you are on the last waypoint and below `min_agl_descent` AGL
2. Whether approach hold mode is on (`approach_hold_guarded`)
3. If flaps are down, or spoilers are activated (`check_cruise_configuration`).
4. You are less than `min_approach_time` from the destination.

User should still be mindful of the potential for an undetected arrival or late
arrival detection. If you descend aggressively or drastically change ground
speed, you may see some simrate rubber banding as you cross thresholds for FLC
estimation and ETE estimates.

There is also an option (`pause_at_tod`) to pause the game at FLC detection.
This is disabled by default. Setting this option implies `waypoint_vnav =
False`. You can unpause the game by in Simrate Control by pressing the "r" key,
or in the simulator by pressing the key bound to "PAUSE OFF". Pause at TOD will
only trigger once per start up of Simrate Control.

## Quickstart SimRate Control

1. Download the release zip file.
2. Unzip in the location of your choice
3. Run simrate_control.exe

## Key bindings

* Press 'w' to toggle waypoint-by-waypoint altitude detection.
* Press 'p' to toggle simrate adjustment on/off.
* Press 'r' to resume after a pause at TOD.
* Press 'q' or 'Ctrl + C' to set simrate to the minimum and exit.
* Press '0' to pause the game. `r` to unpause, as above.
* Press number keys '1-5' to adjust maximum sim rate to 1x, 2x, 4x, 8x,
  16x respectively.

NOTE: Key presses do not take effect until the next screen update, so there may
be some delay between press and effect.

## Configuration

A configuration file to modify various thresholds is available in
`simrate_control/config.ini`.

## Building from Source

```
pip install -r requirements.txt
pyinstaller .\simrate_control.spec
```

## Known Issues

* MSFS inserts "phantom waypoints" into VFR flight plans must be flown near to
  advance the flight plan as seen by the controller. They have names like
  TIMECLI and TIMEVER. A workaround is to file the flight plan IFR.
* This utility is only tested extensively with default planes. Mods and third
  party planes often do not have the simvars this utility relies on, and may
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

Tested on: MSFS 2020 at least version 1.19.8.0

### Acknowledgements

Inspired by the [SimRateBandit](https://github.com/dga711/msfs-simratebandit).

This project uses code from [Koseng /
MSFSPythonSimConnectMobiFlightExtension
](https://github.com/Koseng/MSFSPythonSimConnectMobiFlightExtension) released
under the MIT License.

### Other scripts

* `load_tmp_fpl.py`: This script will look int a particular folder and load the
  most recently written `*.pln` to the aircraft FMS. Has a lot of caveats
  because of MSFS support for SimConnect functions. See also
  [here](https://github.com/albar965/littlenavmap/issues/35#issuecomment-716013932),
  which is the feature I'm actually wanting instead of this scripts.
