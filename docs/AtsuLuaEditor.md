# AtsuLuaEditor visualization integration

This repository vendors the official `src/LuaDocs.cs` snapshot at
`docs/vendor/AtsuLuaEditor-LuaDocs.cs` (source revision
`df1ce44d98b9c5e3854c9e09dad1e13ba9d7bd88`). The source is from the
[AtsuLuaEditor repository](https://git.sr.ht/~alisa/FtD-AtsuLuaEditor) and is
licensed CC BY-SA 4.0; this snapshot is included for attribution and offline
API reference.

The documented extension overloads `I:LogToHud` when the first argument uses
the `draw` syntax. The counterbattery debug stage uses the documented `sphere`,
`line`, `dash`, `gimbal`, `color`, and `duration` forms. `_ALE` is the optional
version global exposed by AtsuLuaEditor; when it is absent, visualization
products are still produced for tests and inspection, but no draw command is
sent and the script logs one warning.

Enable it in `Full/Counterbattery/CounterBattery.lua` with:

```lua
local counterbatteryDebugVisualization = true
```

The pure product is `debug/counterbattery-visualization`. It contains draw
primitives for true target CoM, aimpoints, tracked axes, grid orientation,
subobject axes, density mean, deduplicated trajectory lines, missile warnings,
and the current ballistic sensor observation. The renderer is an edge adapter;
it does not mutate any upstream path-like product.

Refresh the vendored snapshot with:

```sh
.venv/bin/python Util/download_atsuluaeditor_docs.py
```

The official workshop page describes the same `LogToHud` in-world debugging
feature: [AtsuLuaEditor on Steam Workshop](https://steamcommunity.com/sharedfiles/filedetails/?id=3405611847).
