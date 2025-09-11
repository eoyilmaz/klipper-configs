Klipper MMU3
------------

This project contains the required config and code files to enable MMU3
hardware on klipper based 3d printers.

This is based on the GCode macro based version distributed with Klipper. But,
it is quite a bit enhanced and all the functionality has been moved to Python.
This itself supplied a better development environment for all the features
missing on the original version. On top of the original version this has the
following features:

- Sensorless homing for the selector
- MMU specific menus
- Cut functionality
- Smoother load/unload experience
- Easier path to future implementations

Currently, for ease of development, as it is hard to fit Klipper firmware to
the board, instead of the Prusa MMBoard, an SKR Mini E3 v3 is utilized, and all
the configuration files are based on that. But, theoretically this should work
with the Prusa MMBoard.

Installation
------------

No installation scripts yet. Just follow the steps:

1. Copy the `mmu3.py` to `~/klipper/klippy/extras/`
2. Copy `mmu3.cfg`, `mmu3_menus.cfg` and `beep.cfg` to your config folder, typically `printer_data/config`.
3. Add `mmu3.cfg` to your `printer.cfg`.
4. Update `mmu3.cfg` according to your setup.

Usage
-----

It is very hard to explain all the functionalities here.

Typically you don't need to know all the commands, the menus supply all the necessary functionality to prepare MMU for printing and troubleshoot. Just add the following GCode to your Machine start G-Code, somewhere after all the normal homing, bed leveling stuff finished and before any filament is used:

```gcode
HOME_MMU
SELECT_TOOL VALUE=-1
T[initial_tool]
```

The extension supplies all the necessary gcode commands.

1. `Tx`

   The tool change command, i.e. `T0`, `T1`, `T2`, `T3`, `T4`.

2. `Kx`

   Cut filament command, i.e. `K0`, `K1`, `K2`, `K3`, `K4`

3. `HOME_MMU`

   Homes the MMU idler and selector. Typically add this to your Machine start
   G-Code command as explained above.

4. `SELECT_TOOL`

   Selects the requested slot:

   ```gcode
   SELECT_TOOL VALUE=0
   ```

5. `LOAD_FILAMENT_TO_FINDA` / `UNLOAD_FILAMENT_FROM_FINDA`

   Loads/Unloads the filament to FINDA.

6. `LOAD_FILAMENT_TO_EXTRUDER` / `UNLOAD_FILAMENT_FROM_EXTRUDER`

   Loads/Unloads the filament from extruder.

7. `UNLOCK_MMU`

   Unlocks the MMU by moving the idler to the home position. Mostly needed when
   you need to pull/push the filament manually.

The following is the list of all the commands available, most of them are
internally used and will be removed in the future as they are not supplying any
user facing functionality, but are residues from the previous GCode Macro based
design.

   ```gcode
   EJECT_BEFORE_HOME
   EJECT_FROM_EXTRUDER
   EJECT_RAMMING
   ENDSTOPS_STATUS
   HOME_IDLER
   HOME_MMU
   HOME_MMU_ONLY
   K0
   K1
   K2
   K3
   K4
   LOAD_FILAMENT_FROM_FINDA_TO_EXTRUDER
   LOAD_FILAMENT_IN_EXTRUDER
   LOAD_FILAMENT_TO_EXTRUDER
   LOAD_FILAMENT_TO_FINDA
   LOAD_FILAMENT_TO_FINDA_IN_LOOP
   LT
   M702
   PAUSE_MMU
   RETRY_LOAD_FILAMENT_IN_EXTRUDER
   RETRY_UNLOAD_FILAMENT_IN_EXTRUDER
   SELECT_TOOL
   T0
   T1
   T2
   T3
   T4
   UNLOAD_FILAMENT_FROM_EXTRUDER
   UNLOAD_FILAMENT_FROM_EXTRUDER_TO_FINDA
   UNLOAD_FILAMENT_FROM_FINDA
   UNLOAD_FILAMENT_IN_EXTRUDER
   UNLOAD_FILAMENT_IN_EXTRUDER_WITH_RAMMING
   UNLOCK_MMU
   UNSELECT_TOOL
   UT
   ```

