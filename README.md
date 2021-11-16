# Tides
## Lua Library for From the Depths

Project is heavily WIP, but I will try to keep the documentation up to date.

If you have any questions on how to use it, how it works, or how to program in general, contact me at Knife-Kun#3265 on Discord.

## Installation

Unfortunately, FtD does not provide a nice way to import libraries. The easiest way to do it is to copy the content of `Header.lua` to the beginning of your lua script (outside of the Update function) and the content of `Tides.lua` or `Tides.min.lua` anywhere else in your script, such as the very end (again, outside of any functions). `Tides.min.lua` is minified. Do not attempt to try to read this. The same code is contained in separate files throughout the repository. I also used an automatic minifier for this so I'm not 100% sure it works ehe.

If you wish to reduce the size of your code, you can remove any classes that you don't need.

## Full, High, Medium, Low

The classes in Tides are divided into four categories based on their level of abstraction and specialization.

**Low Tide** contains Lua code for basic constructs that are generally useful in many applications, and does not contain any FtD-specific code.

**Medium Tide** contains code which builds on the structures defined in Low, and is slightly more specific to the fields of programming that FtD uses. It contains some Unity-specific code but no FtD specific code.

**High Tide** contains code that is narrowly applicable to applications in FtD and similar fields. It contains FtD-specific code.

**Full** contains complete examples. Currently, there is one example, which is the mouse aim demonstrator.

Features of the mouse aim demonstrator include:
Mouse control of aircraft
Manual and auto aim (slot 1 for manual, 2 for auto)
Targeting, lock-on, and lead reticule (default lead reticule mode is "trailing," which shows where you will hit if you fire now)