# Tides
## Lua Library for From the Depths

Project is heavily WIP, but I will try to keep the documentation up to date.

I am currently rewriting many of my old Lua scripts to use this library, which will be uploaded in one batch.

If you have any questions on how to use it, how it works, or how to program in general, contact me at Knife-Kun#3265 on Discord.

## Installation

Unfortunately, FtD does not provide a nice way to import libraries. The easiest way to do it is to replace the default Lua box contents with the contents of `Wrapper.lua`. Alternatively, copy the content of `Header.lua` to the beginning of your lua script (outside of the Update function) and the content of `Tides.lua` or `Tides.min.lua` anywhere else in your script, such as the very end (again, outside of any functions).

If you wish to reduce the size of your code, you can remove any classes that you don't need.

## Full, High, Medium, Low

The classes in Tides are divided into four categories based on their level of abstraction and specialization.

**Low Tide** contains Lua code for basic constructs that are generally useful in many applications. It contains some Unity-specific code but no FtD specific code.

**Medium Tide** contains code which builds on the structures defined in Low, and is slightly more specific to the fields of programming that FtD uses. It contains some Unity-specific code but no FtD specific code.

**High Tide** contains code that is narrowly applicable to applications in FtD and similar fields. It contains FtD-specific code.

**Full** contains complete examples. Currently, there are 3 examples:

1. Mouse Aim Aircraft:
* Mouse control of aircraft
* Manual and auto aim (slot 1 for manual, 2 for auto)
* Targeting, lock-on, and lead reticule (default lead reticule mode is "trailing," which shows where you will hit if you fire now)

2. Vehicle Missile Interceptor:
* Drone that rams incoming huge missiles
* Spreads out arms mounted on spinblocks prior to collision
* Rolls to rotate arms into place, using information about which arms have already been destroyed (obsolete, now that thump propagates through subobjects)

3. Counterbattery:
* Uses projectile avoidance routine to measure trajectories of incoming shells
* Traces those trajectories back to the time of closest approach to the enemy ship based on recorded enemy positions and returns fire
* Uses multiple AIs to gather aimpoints to track rotation

4. Ship Identification (WIP):
* Uses target prioritization cards to measure stats of enemy ships
* Compares stats to table of known ships to identify them
* May be used to improve counterbattery accuracy based on known locations of weapons

5. Swerve Drive (WIP):
* Rotates wheels to turn a tank, allowing it to drive in any direction
* Automatically detects and accounts for wheel placement to properly handle steering angle (i.e. Ackermann steering)

6. Ramp Targeting (WIP):
* Aims unguided missiles while taking into account initial launch velocity and acceleration time
