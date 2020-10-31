## SimRate Control Theory of Operations

This script is intended to give stable time compression while following a flight
plan on autopilot. At the moment the controller checks the following parameters
before accelerating. Violations will generally lead to sim rate being set to 1x,
with the exception of simply being close to a waypoint, which will only reduce
to 2x.

Specific cited values for conditions detection are subject to change.

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

**Approach**

The plane must be far enough away from the a destination. An approach is
detected based on a few conditions. User should still be mindful of the
potential for an undetected arrival or late arrival detection. If you descend
aggressively or drastically change ground speed, you may see some simrate rubber
banding as you cross thresholds for top of descent (TOD) estimation and ETE
estimates.

1. The last leg of the flight plan is active, and the plane is 3000ft AGL or
   lower.
2. ETE to destination is less than an estimate for TOD.
3. An approach procedure is active.
4. Approach mode is active.

## Quickstart SimRate Control

1. Download the release zip file.
2. Unzip in the location of your choice
3. Run simrate_control.exe

## Building from Source

```
pip install -r requirements.txt
pyinstaller .\simrate_control.spec
```

**KNOWN BUILD ISSUE** PyInstaller searches for a file named
`SimConnect/SimConnect.dllc`, but the dll is copied as
`SimConnect/SimConnect.dll`. I don't know why PyInstaller is looking for an
incorrect file extension. This can be fixed by renaming the file in
`dist/SimConnect/SimConnect.dll` after building.

I can't upload this as a single EXE until I figure out how to fix this.

## TODO/Known Issues

* "Failed to load dynlib/dll '...\\simrate_control\\SimConnect\\SimConnect.dllc"
  see [Issue #6](https://github.com/daheise/msfs_utils/issues/6)
* The A320 does not report GPS_NAV1_LOCK like the other aircraft. Therefore, if
  the AP is on then time acceleration is enabled.
  [Issue #2](https://github.com/daheise/msfs_utils/issues/2)
* Move specific detection threshold values to a configuration file.
  [Issue #4](https://github.com/daheise/msfs_utils/issues/4)
* I would like to give this a GUI, but that's not high on my priority list.
  [Issue #3](https://github.com/daheise/msfs_utils/issues/3)

Tested on: MSFS 2020 1.10.7.0

### Acknowledgements

Inspired by the [SimRateBandit](https://github.com/dga711/msfs-simratebandit).

### Other scripts

* `load_tmp_fpl.py`: This script will look int a particular folder and load the
  most recently written `*.pln` to the aircraft FMS. Has a lot of caveats
  because of MSFS support for SimConnect functions. See also
  [here](https://github.com/albar965/littlenavmap/issues/35#issuecomment-716013932),
  which is the feature I'm actually wanting instead of this scripts.