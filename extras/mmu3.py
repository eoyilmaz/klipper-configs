from __future__ import annotations

from functools import partial, wraps
from typing import TYPE_CHECKING

from . import filament_switch_sensor

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from extras.filament_switch_sensor import RunoutHelper, SwitchSensor
    from extras.heaters import Heater, PrinterHeaters
    from extras.manual_stepper import ManualStepper
    from extras.query_endstops import QueryEndstops
    from gcode import GCodeCommand, GCodeDispatch
    from kinematics.extruder import PrinterExtruder
    from klippy import Printer
    from mcu import MCU_endstop
    from toolhead import ToolHead


IDLER_STEPPER_NAME = "manual_stepper idler_stepper"
PULLEY_STEPPER_NAME = "manual_stepper pulley_stepper"
SELECTOR_STEPPER_NAME = "manual_stepper selector_stepper"

STEPPER_NAME_MAP = {
    PULLEY_STEPPER_NAME: "PINDA",
    SELECTOR_STEPPER_NAME: "Selector",
}


def gcmd_grabber(f):
    """Decorator to grab the gcmd arg temporarily from command methods."""

    @wraps(f)
    def wrapped_f(self, gcmd, *args, **kwargs):
        self._gcmd = gcmd
        result = f(self, gcmd, *args, **kwargs)
        self._gcmd = None
        return result

    return wrapped_f


class MMU3:
    def __init__(self, config: ConfigWrapper):
        self.printer: Printer = config.get_printer()
        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        self.query_endstops: QueryEndstops = self.printer.load_object(config, "query_endstops")

        self._mcu = None
        self._toolhead = None
        self._extruder = None
        self._extruder_heater = None
        self._heaters = None
        self._idler_stepper = None
        self._idler_stepper_endstop = None
        self._pulley_stepper = None
        self._pulley_stepper_endstop = None
        self._selector_stepper = None
        self._selector_stepper_endstop = None

        # state variables
        self.debug = False
        self._gcmd = None
        self.is_paused = False
        self.is_homed = False
        self.extruder_temp = None
        self.current_tool = None
        self.current_filament = None

        # statistics variables
        self.number_of_material_changes = 0
        self.number_of_successful_material_changes = 0
        self.number_of_fails = 0

        # load config values
        # are we in debug mode
        self.debug = config.getboolean("debug", False)
        self.number_of_tools = config.getint("number_of_tools", 5)
        # timeouts
        self.timeout_pause = config.getint("timeout_pause", 36000)
        self.disable_heater = config.getint("disable_heater", 600)
        # bowden load
        self.bowden_load_length1 = config.getint("bowden_load_length1", 450)
        self.bowden_load_length2 = config.getint("bowden_load_length2", 20)
        self.bowden_load_speed1 = config.getint("bowden_load_speed1", 120)
        self.bowden_load_speed2 = config.getint("bowden_load_speed2", 60)
        self.bowden_load_accel1 = config.getint("bowden_load_accel1", 80)
        self.bowden_load_accel2 = config.getint("bowden_load_accel2", 80)
        # bowden unload
        self.bowden_unload_length = config.getfloat("bowden_unload_length", 830)
        self.bowden_unload_speed = config.getint("bowden_unload_speed", 120)
        self.bowden_unload_accel = config.getint("bowden_unload_accel", 120)
        # pinda load/unload
        self.pinda_endstop = None
        self.pinda_load_length = config.getfloat("pinda_load_length", 120)
        self.pinda_unload_length = config.getfloat("pinda_unload_length", 35)
        self.pinda_load_speed = config.getint("pinda_load_speed", 20)
        self.pinda_unload_speed = config.getint("pinda_unload_speed", 20)
        self.pinda_load_accel = config.getint("pinda_load_accel", 50)
        self.pinda_unload_accel = config.getint("pinda_unload_accel", 50)
        # selector
        self.selector_positions = [
            float(f.strip())
            for f in config.getlist(
                "selector_positions", [73.5, 59.375, 45.25, 31.125, 17]
            )
        ]
        # idler
        self.idler_positions = [
            float(f.strip())
            for f in config.getlist("idler_positions", [5, 20, 35, 50, 65])
        ]
        self.idler_home_position = config.getfloat("idler_home_position", 85)
        self.idler_load_to_extruder_speed = config.getint(
            "idler_load_to_extruder_speed", 30
        )
        self.idler_unload_speed = config.getint("idler_unload_speed", 30)
        # pause values
        self.pause_before_disabling_steppers = (
            config.getint("pause_before_disabling_steppers", 50) / 1000.0
        )
        self.pause_after_disabling_steppers = (
            config.getint("pause_after_disabling_steppers", 200) / 1000.0
        )
        self.pause_position = [
            float(f.strip())
            for f in config.getlist("pause_position", [0, 200, 10])
        ]
        # temperature
        self.min_temp_extruder = config.getint("min_temp_extruder", 180)
        self.extruder_eject_temp = config.getint("extruder_eject_temp", 200)
        #
        self.enable_5in1: bool = config.getboolean("enable_5in1", False)
        self.pinda_load_retry = config.getint("pinda_load_retry", 100)
        self.load_retry = config.getint("load_retry", 5)
        self.unload_retry = config.getint("unload_retry", 5)
        self.filament_sensor_name = config.get(
            "filament_sensor_name", "filament_switch_sensor my_filament_sensor"
        )
        self._filament_sensor = None

        # register commands
        self.register_commands()

    def respond_info(self, msg):
        """Respond info through the current GCodeCommand instance."""
        if self._gcmd is None:
            self.gcode.respond_info(f"MMU3: {msg}")
        else:
            self._gcmd.respond_info(f"MMU3: {msg}")

    def respond_debug(self, msg):
        """Respond debug through the current GCodeCommand instance."""
        if not self.debug:
            return
        if self._gcmd is None:
            self.gcode.respond_info(f"MMU3: {msg}")
        else:
            self._gcmd.respond_info(f"MMU3: {msg}")

    def register_commands(self):
        """Register new GCode commands."""
        self.gcode.register_command(
            "LOAD_FILAMENT_TO_PINDA_IN_LOOP", self.cmd_load_filament_to_pinda_in_loop
        )
        self.gcode.register_command("ENDSTOPS_STATUS", self.cmd_endstops_status)
        self.gcode.register_command("HOME_IDLER", self.cmd_home_idler)
        self.gcode.register_command("HOME_MMU", self.cmd_home_mmu)
        self.gcode.register_command("HOME_MMU_ONLY", self.cmd_home_mmu_only)
        self.gcode.register_command("PAUSE_MMU", self.cmd_pause)

        for i in range(self.number_of_tools):
            self.gcode.register_command(f"T{i}", partial(self.cmd_Tx, tool_id=i))

        self.gcode.register_command("UNLOCK_MMU", self.cmd_unlock)
        self.gcode.register_command("LT", self.cmd_load_tool)
        self.gcode.register_command("UT", self.cmd_unload_tool)
        self.gcode.register_command("SELECT_TOOL", self.cmd_select_tool)
        self.gcode.register_command("UNSELECT_TOOL", self.cmd_unselect_tool)
        self.gcode.register_command(
            "RETRY_LOAD_FILAMENT_IN_EXTRUDER", self.cmd_retry_load_filament_in_extruder
        )
        self.gcode.register_command(
            "LOAD_FILAMENT_IN_EXTRUDER", self.cmd_load_filament_in_extruder
        )
        self.gcode.register_command(
            "RETRY_UNLOAD_FILAMENT_IN_EXTRUDER",
            self.cmd_retry_unload_filament_in_extruder,
        )
        self.gcode.register_command(
            "UNLOAD_FILAMENT_IN_EXTRUDER", self.cmd_unload_filament_in_extruder
        )
        self.gcode.register_command("RAMMING_SLICER", self.cmd_ramming_slicer)
        self.gcode.register_command("EJECT_RAMMING", self.cmd_eject_ramming)
        self.gcode.register_command(
            "UNLOAD_FILAMENT_IN_EXTRUDER_WITH_RAMMING",
            self.cmd_unload_filament_in_extruder_with_ramming,
        )
        self.gcode.register_command(
            "LOAD_FILAMENT_TO_PINDA", self.cmd_load_filament_to_pinda
        )
        self.gcode.register_command(
            "LOAD_FILAMENT_FROM_PINDA_TO_EXTRUDER",
            self.cmd_load_filament_from_pinda_to_extruder,
        )
        self.gcode.register_command(
            "LOAD_FILAMENT_TO_EXTRUDER", self.cmd_load_filament_to_extruder
        )
        self.gcode.register_command(
            "UNLOAD_FILAMENT_FROM_PINDA", self.cmd_unload_filament_from_pinda
        )
        self.gcode.register_command(
            "UNLOAD_FILAMENT_FROM_EXTRUDER_TO_PINDA",
            self.cmd_unload_filament_from_extruder_to_pinda,
        )
        self.gcode.register_command(
            "UNLOAD_FILAMENT_FROM_EXTRUDER", self.cmd_unload_filament_from_extruder
        )
        self.gcode.register_command("M702", self.cmd_M702)
        self.gcode.register_command("EJECT_FROM_EXTRUDER", self.cmd_eject_from_extruder)
        self.gcode.register_command("EJECT_BEFORE_HOME", self.cmd_eject_before_home)

    @property
    def toolhead(self) -> ToolHead:
        """Return the toolhead.

        Returns:
            ToolHead: The toolhead.
        """
        if self._toolhead is None:
            self._toolhead = self.printer.lookup_object("toolhead")
        return self._toolhead

    @property
    def extruder(self) -> PrinterExtruder:
        """Return the extruder.

        Returns:
            PrinterExtruder: The extruder.
        """
        if self._extruder is None:
            self._extruder = self.toolhead.get_extruder()
        return self._extruder

    @property
    def heaters(self) -> PrinterHeaters:
        """Return the heater.

        Returns:
            PrinterHeaters: The printer heaters.
        """
        if self._heaters is None:
            self._heaters: PrinterHeaters = self.printer.lookup_object("heaters")
        return self._heaters

    @property
    def extruder_heater(self) -> Heater:
        """Return the extruder heater.

        Returns:
            Heater: The extruder heater.
        """
        if self._extruder_heater is None:
            self._extruder_heater: Heater = self.heaters.lookup_heater("extruder")
        return self._extruder_heater

    @property
    def idler_stepper(self) -> ManualStepper:
        """Return idler stepper."""
        if self._idler_stepper is None:
            self._idler_stepper = self.printer.lookup_object(IDLER_STEPPER_NAME)
        return self._idler_stepper

    @property
    def pulley_stepper(self) -> ManualStepper:
        """Return pulley stepper."""
        if self._pulley_stepper is None:
            self._pulley_stepper = self.printer.lookup_object(PULLEY_STEPPER_NAME)
        return self._pulley_stepper

    @property
    def pulley_stepper_endstop(self) -> MCU_endstop:
        """Return pulley stepper endstop.

        Returns:
            MCU_endstop: The pulley stepper endstop.
        """
        if self._pulley_stepper_endstop is None:
            self._pulley_stepper_endstop = self.get_endstop(PULLEY_STEPPER_NAME)
        return self._pulley_stepper_endstop

    @property
    def selector_stepper(self) -> ManualStepper:
        """Return the selector stepper.

        Returns:
            ManualStepper: The selector stepper.
        """
        if self._selector_stepper is None:
            self._selector_stepper = self.printer.lookup_object(SELECTOR_STEPPER_NAME)
        return self._selector_stepper

    @property
    def selector_stepper_endstop(self) -> MCU_endstop:
        """Return selector stepper endstop.

        Returns:
            MCU_endstop: The selector stepper endstop.
        """
        if self._selector_stepper_endstop is None:
            self._selector_stepper_endstop = self.get_endstop(SELECTOR_STEPPER_NAME)
        return self._selector_stepper_endstop

    @property
    def mcu(self) -> MCU_endstop:
        """Return the mcu."""
        if not self._mcu:
            self._mcu = self.pulley_stepper_endstop.get_mcu()
        return self._mcu

    def get_endstop(self, endstop_name: str) -> None | MCU_endstop:
        """Return the endstop with the given name.

        Args:
            endstop_name (str): The name of the endstop.

        Returns:
            None | MCU_endstop: The requested endstop if found, else None.
        """
        for endstop in self.query_endstops.endstops:
            if endstop[1] == endstop_name:
                return endstop[0]
        return None

    @property
    def filament_sensor(self) -> SwitchSensor:
        """Return the RunoutHelper.

        Returns:
            RunoutHelper: The runout helper.
        """
        if self._filament_sensor is None:
            self._filament_sensor = self.printer.lookup_object(
                self.filament_sensor_name
            )
        return self._filament_sensor

    @property
    def is_filament_present_in_extruder(self) -> bool:
        """Return if the filament present in the extruder runout sensor.

        Returns:
            bool: True if filament sensor is triggered, False otherwise.
        """
        return self.filament_sensor.get_status(None)["filament_detected"]

    @property
    def is_filament_in_pinda(self) -> bool:
        """Return if the filament is in pinda or not.

        Returns:
            bool: True if the filament is present in pinda, False otherwise.
        """
        print_time = self.toolhead.get_last_move_time()
        return bool(self.pulley_stepper_endstop.query_endstop(print_time))

    def validate_filament_in_extruder(self) -> bool:
        """Call PAUSE_MMU if the filament is not detected by the filament sensor.

        Returns:
            bool: True if filament in extruder, False otherwise.
        """
        self.respond_info("Checking if filament in extruder")
        if self.is_filament_present_in_extruder:
            self.respond_info("Filament in extruder")
            return True
        else:
            self.respond_info("Filament not in extruder")
            self.pause()
            return False

    def validate_filament_not_stuck_in_extruder(self) -> bool:
        """Validate filament is not stuck in extruder.

        Returns:
            bool: True if the filament is not present in pinda, False otherwise.
        """
        self.respond_info("Checking if filament stuck in extruder")
        if self.is_filament_present_in_extruder:
            self.respond_info("Filament stuck in extruder")
            self.pause()
            return False
        else:
            self.respond_info("Filament not in extruder")
            return True

    def validate_filament_is_in_pinda(self) -> bool:
        """Validate filament is in PINDA.

        Returns:
            bool: True if filament is in pinda, False otherwise.
        """
        self.respond_info("Checking if filament in PINDA")
        if self.is_filament_in_pinda:
            self.respond_info("Filament in PINDA")
            return True
        else:
            self.respond_info("Filament not in PINDA")
            self.pause()
            return False

    def validate_filament_not_stuck_in_pinda(self) -> bool:
        """Validate filament is not stuck in PINDA.

        Returns:
            bool: True if filament is not stuck in PINDA, False otherwise.
        """
        self.respond_info("Checking if filament stuck in PINDA")
        if self.is_filament_in_pinda:
            self.respond_info("Filament stuck in PINDA")
            self.pause()
            return False
        else:
            self.respond_info("Filament not in PINDA")
            return True

    def validate_hotend_is_hot_enough(self) -> bool:
        """Validate if the hotend is hot enough.

        Pauses if hotend is not hot enough.

        Returns:
            bool: True if hotend is hot enough, False otherwise.
        """
        self.respond_info("Checking if hotend is too cold")
        print_time = self.toolhead.get_last_move_time()
        if self.extruder_heater.get_temp(print_time)[0] < self.min_temp_extruder:
            self.respond_info("Hotend is too cold")
            self.pause()
            return False
        return True

    def home_idler(self):
        """Home the idler

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        # Home the idler
        self.respond_info("Homing idler")
        self.idler_stepper.do_set_position(0)
        self.idler_stepper.do_move(
            7,
            self.idler_stepper.velocity,
            self.idler_stepper.accel,
        )
        self.idler_stepper.do_move(
            -95,
            self.idler_stepper.velocity,
            self.idler_stepper.accel,
        )
        self.idler_stepper.do_set_position(2)
        self.idler_stepper.do_move(
            self.idler_home_position,
            self.idler_stepper.velocity,
            self.idler_stepper.accel,
        )
        self.idler_stepper.dwell(self.pause_before_disabling_steppers)
        self.idler_stepper.do_enable(False)
        self.idler_stepper.dwell(self.pause_after_disabling_steppers)

    def home_mmu(self) -> bool:
        """Home the MMU.

        Eject filament if loaded with EJECT_BEFORE_HOME
        next home the mmu with HOME_MMU_ONLY

        Returns:
            bool: True, if homed, False otherwise.
        """
        self.respond_debug("Start of home_mmu")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        self.filament_sensor.runout_helper.sensor_enabled = False
        self.is_homed = True
        self.respond_info("Homing MMU ...")
        if not self.eject_before_home():
            return False

        self.respond_debug("Before home_mmu_only inside home_mmu")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        home_mmu_only_result = self.home_mmu_only()
        self.respond_debug("After home_mmu_only inside home_mmu")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        return home_mmu_only_result

    def home_mmu_only(self) -> bool:
        """Home the MMU.

        Follow the steps:

        1) home the idler
        2) home the selector (if needed)
        3) try to load filament 0 to PINDA and then unload it. Used to verify
           the MMU3 gear

        if all is ok, the MMU3 is ready to be used

        Returns:
            bool: True, if mmu homed, False otherwise.
        """
        if self.is_paused:
            self.respond_info("Homing MMU failed, MMU is paused, unlock it ...")
            return False

        self.home_idler()
        if not self.enable_5in1:
            self.respond_info("Homing selector")
            self.selector_stepper.do_set_position(0)
            self.selector_stepper.do_homing_move(
                -76,
                self.selector_stepper.velocity,
                self.selector_stepper.accel,
                True,
                True,
            )
            self.selector_stepper.do_set_position(0)
            self.selector_stepper.dwell(self.pause_before_disabling_steppers)
            self.selector_stepper.do_enable(False)
            self.selector_stepper.dwell(self.pause_after_disabling_steppers)

        # self.idler_stepper.do_move(
        #     0,
        #     self.idler_stepper.velocity,
        #     self.idler_stepper.accel,
        # )
        self.current_tool = None
        self.current_filament = None
        self.idler_stepper.dwell(self.pause_before_disabling_steppers)
        self.idler_stepper.do_enable(False)
        self.idler_stepper.dwell(self.pause_after_disabling_steppers)
        self.respond_info("Move selector to filament 0")
        self.select_tool(0)
        self.unselect_tool()
        self.is_homed = True
        self.respond_info("Homing MMU ended ...")

        return True

    def load_filament_to_pinda_in_loop(self) -> None:
        """Load the filament to pinda in a infinite loop.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        while True:
            self.pulley_stepper.do_set_position(0)
            self.pulley_stepper.do_homing_move(
                self.pinda_load_length,
                self.pinda_load_speed,
                self.pinda_load_accel,
                True,
                False,
            )
            self.toolhead.wait_moves()

            # check endstop status and exit from the loop
            if self.is_filament_in_pinda:
                self.respond_info("Pinda endstop triggered. Exiting filament load.")
                break
            else:
                self.respond_info("Pinda endstop not triggered. Retrying...")

    def pause(self):
        """Pause the MMU.

        Park the extruder at the parking position
        Save the current state and start the delayed stop of the heated modify
        the timeout of the printer accordingly to timeout_pause.

        PAUSE MACROS
        PAUSE_MMU is called when an human intervention is needed
        use UNLOCK_MMU to park the idler and start the manual intervention
        and use RESUME when the invention is ended to resume the current print
        """
        print_time = self.toolhead.get_last_move_time()
        self.extruder_temp = self.extruder_heater.get_temp(print_time)[0]
        self.is_paused = True
        self.gcode.run_script_from_command(f"""
            SAVE_GCODE_STATE NAME=PAUSE_MMU_state
            SET_IDLE_TIMEOUT TIMEOUT={self.timeout_pause}
            M118 Start PAUSE
            M300
            M300
            M300
            PAUSE
            G90
            G1 X{self.pause_position[0]} Y{self.pause_position[1]} F3000
            M300
            M300
            M300
            M118 End PAUSE
            RESTORE_GCODE_STATE NAME=PAUSE_MMU_state
        """)

    def unlock(self) -> None:
        """Park the idler, stop the delayed stop of the heater.

        Args:
            gcmd GCodeCommand: The G-code command.
        """
        self.respond_info("Resume print")
        self.is_paused = False
        self.home_idler()

    def select_tool(self, tool_id: int) -> bool:
        """Select a tool. move the idler and then move the selector (if needed).

        Args:
            tool_id (int): The tool id.

        Returns:
            bool: True, if tool is selected, False otherwise.
        """
        if self.is_paused:
            return False

        if not self.is_homed:
            self.respond_info("Could not select tool, MMU is not homed")
            return False

        if tool_id is None or tool_id < 0:
            return False

        self.respond_info(f"Select Tool {tool_id} ...")
        self.idler_stepper.do_move(
            self.idler_positions[tool_id],
            self.idler_stepper.velocity,
            self.idler_stepper.accel
        )

        if not self.enable_5in1:
            self.selector_stepper.do_move(
                self.selector_positions[tool_id],
                self.selector_stepper.velocity,
                self.selector_stepper.accel,
            )
            self.selector_stepper.dwell(self.pause_before_disabling_steppers)
            self.selector_stepper.do_enable(False)
            self.selector_stepper.dwell(self.pause_after_disabling_steppers)
        self.current_tool = tool_id
        # self.current_filament = None  # tool_id
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        self.respond_info(f"Tool {tool_id} Enabled")
        return True

    def unselect_tool(self) -> bool:
        """Unselect a tool, only park the idler.

        Returns:
            bool: True, if tool is unselected, False otherwise.
        """
        if self.is_paused:
            return False
        if not self.is_homed:
            self.respond_info("Could not unselect tool, MMU is not homed")
            return False

        if self.current_tool is not None:
            self.respond_info(f"Unselecting Tool T{self.current_tool}")
        else:
            self.respond_info("Unselecting tool while Current Tool is None!")

        self.idler_stepper.do_move(
            self.idler_home_position,
            self.idler_stepper.velocity,
            self.idler_stepper.accel,
        )
        self.current_tool = None
        self.idler_stepper.dwell(self.pause_before_disabling_steppers)
        self.idler_stepper.do_enable(False)
        self.idler_stepper.dwell(self.pause_after_disabling_steppers)

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        self.respond_info("Unselect Tool is complete!")
        return True

    def retry_load_filament_in_extruder(self) -> bool:
        """Try to reinsert the filament into the extruder.

        Called when the IR sensor does not detect the filament the MMU3 push
        the filament of 10mm and the extruder gear try to insert it into the
        nozzle.

        Returns:
            bool: True, if filament loaded in extruder, False otherwise.
        """
        if self.is_filament_present_in_extruder:
            return True

        self.respond_info("Retry loading ...")
        print_time = self.toolhead.get_last_move_time()
        if self.is_paused:
            self.respond_info("Printer is paused ...")
            return False

        if self.extruder_heater.get_temp(print_time)[0] < self.min_temp_extruder:
            self.respond_info("Hotend is not hot enough ...")
            return False

        self.respond_info("Loading Filament...")
        self.gcode.run_script_from_command("G91")
        self.select_tool(self.current_filament)

        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            10,
            self.pulley_stepper.velocity,
            self.pulley_stepper.accel,
        )

        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.dwell(self.pause_before_disabling_steppers)
        self.pulley_stepper.do_enable(False)
        self.pulley_stepper.dwell(self.pause_after_disabling_steppers)

        self.gcode.run_script_from_command("G1 E5 F600")
        self.unselect_tool()
        self.gcode.run_script_from_command("""
            G1 E2 F1800
            G1 E3 F1393
            G1 E2 F614
            G92 E0
            G90
        """)
        return True

    def load_filament_in_extruder(self) -> bool:
        """Load the filament into the extruder.

        The MMU3 push the filament of 20mm and the extruder gear try to insert
        it into the nozzle if the filament is not detected by the IR, call
        RETRY_LOAD_FILAMENT_IN_EXTRUDER 5 times.

        Call PAUSE_MMU if the filament is not detected by the IR sensor.

        Returns:
            bool: True, if filament loaded in extruder.
        """
        if self.is_paused:
            return False

        if not self.validate_hotend_is_hot_enough():
            return False

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        self.respond_info("Loading Filament...")
        self.gcode.run_script_from_command("G91")
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            20,
            self.idler_load_to_extruder_speed,
            self.pulley_stepper.accel,
        )
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.dwell(self.pause_before_disabling_steppers)
        self.pulley_stepper.do_enable(False)
        self.pulley_stepper.dwell(self.pause_after_disabling_steppers)
        self.gcode.run_script_from_command("G1 E10 F600")
        self.unselect_tool()
        self.gcode.run_script_from_command("""
            G1 E3 F1800
            G1 E4 F1393
            G1 E3 F614
            G92 E0
            G90
        """)
        if not self.is_filament_present_in_extruder:
            for _ in range(self.load_retry):
                self.retry_load_filament_in_extruder()

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        if self.validate_filament_in_extruder():
            self.respond_info("Load Complete")
            self.respond_debug(f"self.current_tool    : {self.current_tool}")
            self.respond_debug(f"self.current_filament: {self.current_filament}")
            return True
        else:
            self.respond_debug(f"self.current_tool    : {self.current_tool}")
            self.respond_debug(f"self.current_filament: {self.current_filament}")
            return False

    def retry_unload_filament_in_extruder(self) -> None:
        """Retry unload, try correct misalignment of bondtech gear."""
        if not self.is_filament_present_in_extruder:
            return True

        self.respond_info("Retry unloading ....")
        if self.is_paused:
            self.respond_info("MMU is paused")
            return False

        print_time = self.toolhead.get_last_move_time()
        if self.extruder_heater.get_temp(print_time)[0] < self.min_temp_extruder:
            self.respond_info("Hotend is too cold")
            return False

        self.respond_info("Unloading Filament...")
        self.gcode.run_script_from_command("""
            G91
            G92 E0
            G1 E10 F500
            G1 E-20 F500
            G1 E-30 F3000
            G92 E0
            G90
        """)
        return True

    def unload_filament_in_extruder(self) -> bool:
        """Unload the filament from the nozzle (without RAMMING !!!).

        Retract the filament from the nozzle to the out of the extruder gear.
        Call PAUSE_MMU if the IR sensor detects the filament after the ejection

        Returns:
            bool: True, if the filament unloaded from extruder.
        """
        if self.is_paused:
            return False

        if not self.validate_hotend_is_hot_enough():
            return False

        if not self.is_filament_present_in_extruder:
            self.respond_info("No filament in extruder")
            return True

        if self.current_tool is not None:
            # self.respond_info("Tool selected, UNSELECT it")
            # self.pause()
            # return False
            self.respond_info(f"Tool T{self.current_tool} selected!")
            self.respond_info("Auto unselecting it!")
            self.unselect_tool()

        self.respond_info("Unloading Filament...")
        self.gcode.run_script_from_command("""
            G91
            G92 E0
            G1 E-20 F500
            G1 E-30 F3000
            G90
            G92 E0
            G4 P1000
        """)
        if self.is_filament_present_in_extruder:
            for _ in range(self.unload_retry):
                self.retry_unload_filament_in_extruder()

        if self.validate_filament_not_stuck_in_extruder():
            self.respond_info("Filament removed")
            return True
        else:
            return False

    def ramming_slicer(self):
        """Ramming process for standard PLA, code extracted from OrcaSlicer gcode."""
        self.gcode.run_script_from_command("""
        G91
        G92 E0
        ;TYPE:Prime tower
        ;WIDTH:0.5
        ;--------------------
        ; CP TOOLCHANGE START
        M220 S100
        ; CP TOOLCHANGE UNLOAD
        ;WIDTH:0.65
        SET_PRESSURE_ADVANCE ADVANCE=0
        G1 E0.2213 F1052
        G1 E0.2481 F1180
        G1 E0.3051 F1451
        G1 E0.3722 F1769
        G1 E0.4191 F1993
        G1 E0.4728 F2248
        G1 E0.5800 F2758
        G1 E0.3344 F3379
        G1 E0.3764 F3379
        G1 E0.7846 F3730
        G1 E0.8080 F3842
        G1 E0.8751 F4161
        G1 E0.1090 F4750
        G1 E0.8902 F4750
        G1 E1.0863 F5165
        G1 E0.9765 F5245
        G1 E0.1266 F5245
        ; Ramming start
        ; Retract(unload)
        G1 E-15.0000 F12000
        G1 E-8.0500 F5400
        G1 E-2.3000 F2700
        G1 E-1.1500 F1620
        ; Cooling
        G1 E5.0000 F2016
        G1 E-5.0000 F1920
        G1 E5.0000 F1824
        G1 E-5.0000 F1728
        G1 E5.0000 F1632
        G1 E-5.0000 F1536
        G1 E5.0000 F1440
        G1 E-5.0000 F1344
        ; Cooling park
        G1 E1.5000 F2000
        ; Ramming end
        G1 E-1 F5100
        G1 E-50.0000 F2000
        G90
        G92 E0
        """)

    def eject_ramming(self) -> bool:
        """Eject the filament with ramming from the extruder nozzle to the MMU3.

        Returns:
            bool: True if ejected, False otherwise.
        """
        if self.is_paused:
            return False

        if self.current_filament is None:
            return False

        self.respond_info(f"UT {self.current_filament} ...")
        if not self.unload_filament_in_extruder_with_ramming():
            return False
        self.select_tool(self.current_filament)
        return self.unload_filament_from_extruder()

    def unload_filament_in_extruder_with_ramming(self) -> bool:
        """Unload from extruder with ramming.

        Returns:
            bool: True, if filament unloaded from extruder, False otherwise.
        """
        if self.is_paused:
            return False

        if not self.validate_hotend_is_hot_enough():
            return False

        if self.current_tool is None:
            self.respond_info("Ramming and Unloading Filament...")
            self.ramming_slicer()
            if self.unload_filament_in_extruder():
                self.respond_info("Filament rammed and removed")
                return True
        else:
            self.respond_info("Tool selected, UNSELECT it")
            self.pause()
            return False

    def load_filament_to_pinda(self) -> bool:
        """Load filament until the PINDA detect it.

        Then push it 10mm more to be sure is well detected.
        PAUSE_MMU is called if the PINDA does not detect the filament

        Returns:
            bool: True, if the filament is loaded to pinda, False otherwise.
        """
        if self.is_paused:
            return False

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        if self.current_tool is None:
            self.respond_info("Cannot load to PINDA, tool not selected !!")
            return False

        self.respond_info("Loading filament to PINDA ...")
        self.load_filament_to_pinda_in_loop()

        self.pulley_stepper.do_set_position(0)
        # self.pulley_stepper.do_move(
        #     10,
        #     self.pulley_stepper.velocity,
        #     self.pulley_stepper.accel,
        # )
        self.pulley_stepper.dwell(self.pause_before_disabling_steppers)
        self.pulley_stepper.do_enable(False)
        self.pulley_stepper.dwell(self.pause_after_disabling_steppers)

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        if self.validate_filament_is_in_pinda():
            self.current_filament = self.current_tool
            self.respond_info("Loading done to PINDA")
            self.respond_debug(f"self.current_tool    : {self.current_tool}")
            self.respond_debug(f"self.current_filament: {self.current_filament}")
            return True
        else:
            return False

    def load_filament_from_pinda_to_extruder(self) -> bool:
        """Load from the PINDA to the extruder gear.

        Returns:
            bool: True, if filament is loaded from pinda to extruder, False
                otherwise.
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            self.respond_info("Cannot load to extruder, tool not selected !!")
            return False

        self.respond_info("Loading filament from PINDA to extruder ...")
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            self.bowden_load_length1,
            self.bowden_load_speed1,
            self.bowden_load_accel1,
        )
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            self.bowden_load_length2,
            self.bowden_load_speed2,
            self.bowden_load_accel2,
        )
        self.pulley_stepper.dwell(self.pause_before_disabling_steppers)
        self.pulley_stepper.do_enable(False)
        self.pulley_stepper.dwell(self.pause_after_disabling_steppers)
        self.respond_info("Loading done from PINDA to extruder")

        return True

    def load_filament_to_extruder(self) -> bool:
        """Load from MMU3 to extruder gear by calling LOAD_FILAMENT_TO_PINDA

        Then LOAD_FILAMENT_FROM_PINDA_TO_EXTRUDER.
        PAUSE_MMU is called if the PINDA does not detect the filament.

        Args:
            bool: True, if filament is loaded to extruder
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            self.respond_info("Cannot load to extruder, tool not selected !!")
            return False

        self.respond_info("Loading filament from MMU to extruder ...")
        if self.enable_5in1 is False:
            if not self.load_filament_to_pinda():
                return False

        if self.load_filament_from_pinda_to_extruder():
            self.respond_info("Loading done from MMU to extruder")
            return True
        else:
            return False

    def unload_filament_from_pinda(self) -> None:
        """Unload filament until the PINDA detect it.

        Then push it -10mm more to be sure is well not detected.
        PAUSE_MMU is called if the PINDA does detect the filament.

        Returns:
            bool: True, if filament unloaded from pinda, False otherwise.
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            self.respond_info("Cannot unload from PINDA, tool not selected !!")
            return False

        self.respond_info("Unloading filament from PINDA ...")
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            -self.pinda_unload_length,
            self.pinda_unload_speed,
            self.pinda_unload_accel,
        )
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.dwell(self.pause_before_disabling_steppers)
        self.pulley_stepper.do_enable(False)
        self.pulley_stepper.dwell(self.pause_after_disabling_steppers)
        if not self.validate_filament_not_stuck_in_pinda():
            return False
        self.current_filament = None
        self.respond_info("Unloading done from PINDA")
        return True

    def unload_filament_from_extruder_to_pinda(self) -> bool:
        """Unload from extruder gear to the PINDA

        Returns:
            bool: True, if filament unloaded from extruder to pinda.
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            self.respond_info(
                "Cannot unload from extruder to PINDA, tool not selected !!"
            )
            return False

        self.respond_info("Unloading filament from extruder to PINDA ...")
        self.pulley_stepper.do_set_position(0)
        if not self.enable_5in1:
            self.pulley_stepper.do_homing_move(
                -self.bowden_unload_length,
                self.bowden_unload_speed,
                self.bowden_unload_accel,
                False,
                False,
            )
            # self.pulley_stepper.do_homing_move(
            #     -self.bowden_unload_length,
            #     self.bowden_unload_speed,
            #     self.bowden_unload_accel,
            #     False,
            #     False,
            # )
            if not self.validate_filament_not_stuck_in_pinda():
                return False
        else:
            self.pulley_stepper.do_move(
                -self.bowden_unload_length,
                self.bowden_unload_speed,
                self.bowden_unload_accel,
            )
        self.pulley_stepper.dwell(self.pause_before_disabling_steppers)
        self.pulley_stepper.do_enable(False)
        self.pulley_stepper.dwell(self.pause_after_disabling_steppers)
        self.respond_info("Unloading done from PINDA to extruder")
        return True

    def unload_filament_from_extruder(self) -> bool:
        """Unload from the extruder gear to the MMU3

        Do it by calling UNLOAD_FILAMENT_FROM_EXTRUDER_TO_PINDA and
        then UNLOAD_FILAMENT_FROM_PINDA

        Returns:
            bool: True, if filament unloaded from the extruder, False otherwise.
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            self.respond_info(
                "Cannot unload from extruder to MMU, tool not selected !!"
            )
            return False

        self.respond_info("Unloading filament from extruder to MMU ...")
        if not self.unload_filament_from_extruder_to_pinda():
            return False

        if self.enable_5in1:
            self.respond_info("Unloading done from extruder to MMU")
            return True

        if not self.unload_filament_from_pinda():
            return False

        self.respond_info("Unloading done from extruder to MMU")
        return True

    def load_tool(self, tool_id: int) -> bool:
        """Load filament from MMU3 to nozzle.

        Args:
            tool_id (int): The tool id.

        Returns:
            bool: True, if filament is loaded, False otherwise.
        """
        if self.is_paused:
            return False

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        self.respond_info(f"LT {tool_id}")
        if not self.select_tool(tool_id):
            self.respond_debug(f"self.current_tool    : {self.current_tool}")
            self.respond_debug(f"self.current_filament: {self.current_filament}")
            return False
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        if not self.load_filament_to_extruder():
            self.respond_debug(f"self.current_tool    : {self.current_tool}")
            self.respond_debug(f"self.current_filament: {self.current_filament}")
            return False
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        return self.load_filament_in_extruder()

    def unload_tool(self) -> bool:
        """Unload filament from nozzle to MMU3.

        Returns:
            bool: True, if tool is unloaded, False otherwise.
        """
        if self.is_paused:
            return False

        if self.current_filament is None:
            self.respond_info("Current filament is None!")
            # so we don't know what tool we are in.
            # we should try homing first
            # and then try unloading_filament one_by_one

            # for i in range(self.number_of_tools):
            #     self.home_idler()
            #     self.current_tool = i
            #     self.respond_info(f"Trying UT {self.current_tool}")

            #     self.current_tool = None

            #     if not self.unload_filament_in_extruder():
            #         continue
            #     else:
            #         break

            # # now we can home the selector if the filament is not in the
            # # extruder
            # if self.validate_filament_not_stuck_in_extruder():
            #     self.home_mmu()
            #     self.current_tool = 0
            #     self.select_tool(self.current_tool)
            #     return True
            # else:
            #     self.current_tool = None
            #     return False
            if self.is_filament_in_pinda:
                self.respond_info("But there is a filament in PINDA!")
                if self.current_tool is None:
                    self.respond_info("Current Tool is also None!")
                    self.respond_info("Cancelling unload!!!")
                    return False
                else:
                    self.respond_info(f"Current Tool is {self.current_tool}")
                    self.current_filament = self.current_tool
                    self.respond_info(f"Also setting Current filament to {self.current_filament}")
            else:
                self.respond_info("And no filament in PINDA")
                self.respond_info("No need to unload!")
                return True

        self.respond_info(f"UT {self.current_filament}")
        if not self.unload_filament_in_extruder():
            return False
        if not self.select_tool(self.current_filament):
            return False
        return self.unload_filament_from_extruder()

    def eject_from_extruder(self) -> bool:
        """Preheat the heater if needed and unload the filament with ramming.

        Eject from nozzle to extruder gear out.

        Returns:
            bool: True, if the filament is ejected from extruder, False
                otherwise.
        """
        if self.is_paused:
            return False

        if not self.is_filament_present_in_extruder:
            self.respond_info("Filament not in extruder")
            return True

        self.respond_info("Filament in extruder, trying to eject it ...")
        self.respond_info("Preheat Nozzle")
        print_time = self.toolhead.get_last_move_time()
        self.gcode.run_script_from_command(
            f"M109 S{max(self.extruder_heater.get_temp(print_time)[0],self.extruder_eject_temp)}"
        )
        if not self.unload_filament_in_extruder_with_ramming():
            return False
        self.gcode.run_script_from_command("M104 S0")

    def eject_before_home(self) -> None:
        """Eject from extruder gear to MMU3.

        Returns:
            bool: True, if filament ejected, False otherwise.
        """
        self.respond_info("Eject Filament if loaded ...")
        if self.is_filament_present_in_extruder:
            if not self.eject_from_extruder():
                return False
            if not self.validate_filament_not_stuck_in_extruder():
                return False

        if not self.enable_5in1:
            if self.is_filament_in_pinda:
                if not self.unload_filament_from_extruder():
                    return False
                if not self.validate_filament_not_stuck_in_pinda():
                    return False
                self.respond_info("Filament ejected !")
            else:
                self.respond_info("Filament already ejected !")
        else:
            self.respond_info("Filament already ejected !")

        return True

    @gcmd_grabber
    def cmd_endstops_status(self, gcmd) -> None:
        """Print the status of all endstops.

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        # Query the endstops
        print_time = self.toolhead.get_last_move_time()

        # Report results
        self.respond_info("Endstop status")
        self.respond_info("==============")
        self.respond_info(f"Extruder : {self.is_filament_present_in_extruder}")
        self.respond_info(
            f"{STEPPER_NAME_MAP[PULLEY_STEPPER_NAME]} : "
            f"{self.pulley_stepper_endstop.query_endstop(print_time)}"
        )
        # gcmd.respond_info(f"is_filament_in_pinda: {self.is_filament_in_pinda}")
        self.respond_info(
            f"{STEPPER_NAME_MAP[SELECTOR_STEPPER_NAME]} : "
            f"{self.selector_stepper_endstop.query_endstop(print_time)}"
        )

        # _ = self.idler_stepper

    @gcmd_grabber
    def cmd_home_idler(self, gcmd) -> None:
        """Home the idler

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        self.home_idler()

    @gcmd_grabber
    def cmd_home_mmu(self, gcmd) -> None:
        """Home the MMU.

        Eject filament if loaded with EJECT_BEFORE_HOME
        next home the mmu with HOME_MMU_ONLY

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        self.home_mmu()

    @gcmd_grabber
    def cmd_home_mmu_only(self, gcmd) -> None:
        """Home the MMU.

        Follow the steps:

        1) home the idler
        2) home the selector (if needed)
        3) try to load filament 0 to PINDA and then unload it. Used to verify
           the MMU3 gear

        if all is ok, the MMU3 is ready to be used

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        self.home_mmu_only()

    @gcmd_grabber
    def cmd_load_filament_to_pinda_in_loop(self, gcmd: GCodeCommand) -> None:
        """Load the filament to pinda in a infinite loop.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_to_pinda_in_loop()

    @gcmd_grabber
    def cmd_pause(self, gcmd: GCodeCommand) -> None:
        """Pause the MMU.

        Park the extruder at the parking position
        Save the current state and start the delayed stop of the heated modify
        the timeout of the printer accordingly to timeout_pause.

        Args:
            gcmd: (GCodeCommand): The G-code command.
        """
        self.pause()

    @gcmd_grabber
    def cmd_Tx(self, gcmd: GCodeCommand, tool_id: int = 0) -> None:
        """The generic Tx command.

        Args:
            gcmd (GCodeCommand): The G-code command.
            tool_id (int, optional): The tool id to load. Defaults to 0.
        """
        self.respond_info(f"Requested tool {tool_id}")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        if self.current_filament == tool_id:
            return
        self.respond_info(f"Change Tool T{tool_id}")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        runout_pause = None
        if self.filament_sensor.runout_helper.runout_pause:
            self.respond_info(f"Disabling filament runout sensor!")
            runout_pause = self.filament_sensor.runout_helper.runout_pause
            self.filament_sensor.runout_helper.runout_pause = False

        if not self.unload_tool():
            self.respond_info(f"Apparently unload tool failed!")
            self.respond_debug(f"self.current_tool    : {self.current_tool}")
            self.respond_debug(f"self.current_filament: {self.current_filament}")
            return
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        self.load_tool(tool_id)
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        if runout_pause is not None:
            self.respond_info(f"Re-Enabling filament runout sensor!")
            self.filament_sensor.runout_helper.runout_pause = runout_pause

    @gcmd_grabber
    def cmd_unlock(self, gcmd: GCodeCommand) -> None:
        """Park the idler, stop the delayed stop of the heater.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unlock()

    @gcmd_grabber
    def cmd_load_tool(self, gcmd: GCodeCommand) -> None:
        """Load filament from MMU3 to nozzle.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        tool_id = gcmd.get_int("VALUE", None)
        self.load_tool(tool_id)

    @gcmd_grabber
    def cmd_unload_tool(self, gcmd: GCodeCommand) -> None:
        """Unload filament from nozzle to MMU3.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_tool()

    @gcmd_grabber
    def cmd_select_tool(self, gcmd: GCodeCommand) -> None:
        """Select a tool. move the idler and then move the selector (if needed).

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        tool_id = gcmd.get_int("VALUE", None)
        self.select_tool(tool_id)

    @gcmd_grabber
    def cmd_unselect_tool(self, gcmd: GCodeCommand) -> None:
        """Unselect a tool, only park the idler.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unselect_tool()

    @gcmd_grabber
    def cmd_retry_load_filament_in_extruder(self, gcmd: GCodeCommand) -> None:
        """Try to reinsert the filament into the extruder.

        Called when the IR sensor does not detect the filament the MMU3 push
        the filament of 10mm and the extruder gear try to insert it into the
        nozzle.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.retry_load_filament_in_extruder()

    @gcmd_grabber
    def cmd_load_filament_in_extruder(self, gcmd: GCodeCommand) -> None:
        """Load the filament into the extruder.

        The MMU3 push the filament of 20mm and the extruder gear try to insert
        it into the nozzle if the filament is not detected by the IR, call
        RETRY_LOAD_FILAMENT_IN_EXTRUDER 5 times.

        Call PAUSE_MMU if the filament is not detected by the IR sensor.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_in_extruder()

    @gcmd_grabber
    def cmd_retry_unload_filament_in_extruder(self, gcmd: GCodeCommand) -> None:
        """Retry unload, try correct misalignment of bondtech gear.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.retry_unload_filament_in_extruder()

    @gcmd_grabber
    def cmd_unload_filament_in_extruder(self, gcmd: GCodeCommand) -> None:
        """Unload the filament from the nozzle (without RAMMING !!!).

        Retract the filament from the nozzle to the out of the extruder gear.
        Call PAUSE_MMU if the IR sensor detects the filament after the ejection

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_filament_in_extruder()

    @gcmd_grabber
    def cmd_ramming_slicer(self, gcmd: GCodeCommand) -> None:
        """Ramming process for standard PLA, code extracted from OrcaSlicer gcode.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.ramming_slicer()

    @gcmd_grabber
    def cmd_eject_ramming(self, gcmd: GCodeCommand) -> None:
        """Eject the filament with ramming from the extruder nozzle to the MMU3.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.eject_ramming()

    @gcmd_grabber
    def cmd_unload_filament_in_extruder_with_ramming(self, gcmd: GCodeCommand) -> None:
        """Unload from extruder with ramming.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_filament_in_extruder_with_ramming()

    @gcmd_grabber
    def cmd_load_filament_to_pinda(self, gcmd: GCodeCommand) -> None:
        """Load filament until the PINDA detect it.

        Then push it 10mm more to be sure is well detected.
        PAUSE_MMU is called if the PINDA does not detect the filament

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_to_pinda()

    @gcmd_grabber
    def cmd_load_filament_from_pinda_to_extruder(self, gcmd: GCodeCommand) -> None:
        """Load from the PINDA to the extruder gear.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_from_pinda_to_extruder()

    @gcmd_grabber
    def cmd_load_filament_to_extruder(self, gcmd: GCodeCommand) -> None:
        """Load from MMU3 to extruder gear by calling LOAD_FILAMENT_TO_PINDA

        Then LOAD_FILAMENT_FROM_PINDA_TO_EXTRUDER.
        PAUSE_MMU is called if the PINDA does not detect the filament.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_to_extruder()

    @gcmd_grabber
    def cmd_unload_filament_from_pinda(self, gcmd: GCodeCommand) -> None:
        """Unload filament until the PINDA detect it.

        Then push it -10mm more to be sure is well not detected.
        PAUSE_MMU is called if the PINDA does detect the filament.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_filament_from_pinda()

    @gcmd_grabber
    def cmd_unload_filament_from_extruder_to_pinda(self, gcmd: GCodeCommand) -> None:
        """Unload from extruder gear to the PINDA

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_filament_from_extruder_to_pinda()

    @gcmd_grabber
    def cmd_unload_filament_from_extruder(self, gcmd: GCodeCommand) -> None:
        """Unload from the extruder gear to the MMU3

        Do it by calling UNLOAD_FILAMENT_FROM_EXTRUDER_TO_PINDA and
        then UNLOAD_FILAMENT_FROM_PINDA

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_filament_from_extruder()

    @gcmd_grabber
    def cmd_M702(self, gcmd: GCodeCommand) -> None:
        """Unload filament if inserted into the IR sensor.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_tool()
        if not self.enable_5in1:
            if not self.is_filament_in_pinda:
                self.unselect_tool()
                self.respond_info("M702 ok ...")
            else:
                self.respond_info("M702 Error !!!")
        else:
            self.unselect_tool()
            self.current_filament = None
            self.respond_info("M702 ok ...")

    @gcmd_grabber
    def cmd_eject_from_extruder(self, gcmd: GCodeCommand) -> None:
        """Preheat the heater if needed and unload the filament with ramming.

        Eject from nozzle to extruder gear out.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.eject_from_extruder()

    @gcmd_grabber
    def cmd_eject_before_home(self, gcmd: GCodeCommand) -> None:
        """Eject from extruder gear to MMU3.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.eject_before_home()


def load_config_prefix(config):
    return MMU3(config)
