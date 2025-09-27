"""MMU3 multi-material unit management."""

from __future__ import annotations

from functools import partial, wraps
from typing import TYPE_CHECKING, Callable

from extras.manual_stepper import ManualStepper

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from extras.display_status import DisplayStatus
    from extras.filament_switch_sensor import SwitchSensor
    from extras.filament_motion_sensor import EncoderSensor
    from extras.heaters import Heater, PrinterHeaters
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
    PULLEY_STEPPER_NAME: "FINDA",
    SELECTOR_STEPPER_NAME: "Selector",
}


def gcmd_grabber(f: Callable) -> Callable:
    """Decorator to grab the gcmd arg temporarily from command methods.

    Args:
        f (Callable): The function to wrap.

    Returns:
        Callable: The wrapped function.
    """

    @wraps(f)
    def wrapped_f(self: object, gcmd: GCodeCommand, *args, **kwargs) -> None:
        self._gcmd = gcmd
        result = f(self, gcmd, *args, **kwargs)
        self._gcmd = None
        return result

    return wrapped_f


class MMU3:
    """MMU3 class to manage the MMU3 multi-material unit.

    Args:
        config (ConfigWrapper): The configuration wrapper.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        self.printer: Printer = config.get_printer()
        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        self.query_endstops: QueryEndstops = self.printer.load_object(
            config, "query_endstops"
        )
        self.reactor = self.printer.get_reactor()

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
        self._display_status = None

        # state variables
        self.debug = False
        self._gcmd = None
        self.is_paused = False
        self.is_homed = False
        self.extruder_temp = None
        self.current_tool = None
        self.current_filament = None

        # statistics variables
        self.number_of_material_changes = 0
        self.number_of_successful_material_changes = 0
        self.number_of_fails = 0

        # load config values
        # are we in debug mode
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
        # FINDA load/unload
        self.finda_load_retry = config.getint("finda_load_retry", 20)
        self.finda_load_length = config.getfloat("finda_load_length", 120)
        self.finda_unload_retry = config.getint("finda_unload_retry", 10)
        self.finda_unload_length = config.getfloat("finda_unload_length", 30)
        self.finda_load_speed = config.getint("finda_load_speed", 20)
        self.finda_unload_speed = config.getint("finda_unload_speed", 20)
        self.finda_load_accel = config.getint("finda_load_accel", 50)
        self.finda_unload_accel = config.getint("finda_unload_accel", 50)
        # cut length
        self.cut_filament_length = config.getfloat("cut_filament_length", 20)
        self.cutting_edge_retract = config.getfloat("cutting_edge_retract", 5)
        self.cut_stepper_current = config.getfloat("cut_stepper_current", 1.0)
        # selector
        self.selector_speed = config.getfloat("selector_speed", 35)
        self.selector_homing_speed = config.getfloat("selector_homing_speed", 20)
        self.selector_accel = config.getfloat("selector_accel", 200)
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
            float(f.strip()) for f in config.getlist("pause_position", [0, 200, 10])
        ]
        # temperature
        self.min_temp_extruder = config.getint("min_temp_extruder", 180)
        self.extruder_eject_temp = config.getint("extruder_eject_temp", 200)
        # other options
        self.enable_no_selector_mode: bool = config.getboolean(
            "enable_no_selector_mode",
            False,
        )
        self.load_retry = config.getint("load_retry", 5)
        self.unload_retry = config.getint("unload_retry", 5)
        self.filament_switch_sensor_name = config.get(
            "filament_switch_sensor_name", "filament_switch_sensor my_filament_sensor"
        )
        self._filament_switch_sensor = None
        self.filament_motion_sensor_name = config.get(
            "filament_motion_sensor_name", "filament_motion_sensor encoder_sensor"
        )
        self._filament_motion_sensor = None

        # register commands
        self.register_commands()

    def respond_info(self, msg: str) -> None:
        """Respond info through the current GCodeCommand instance.

        Args:
            msg (str): The info message.
        """
        if self._gcmd is None:
            self.gcode.respond_info(f"MMU3: {msg}")
        else:
            self._gcmd.respond_info(f"MMU3: {msg}")

    def respond_debug(self, msg: str) -> None:
        """Respond debug through the current GCodeCommand instance.

        Args:
            msg (str): The debug message.
        """
        if not self.debug:
            return
        if self._gcmd is None:
            self.gcode.respond_info(f"MMU3: {msg}")
        else:
            self._gcmd.respond_info(f"MMU3: {msg}")

    def display_status_msg(self, msg: str) -> None:
        """Display the given status message in the LCD display."""
        # also send the message to the console
        self.respond_info(msg)
        # if self.display_status is not None:
        #     self.display_status.message = msg
        #     self.display_status.progress
        self.gcode.run_script_from_command(f"M117 {msg}")

    def register_commands(self) -> None:
        """Register new GCode commands."""
        self.gcode.register_command(
            "LOAD_FILAMENT_TO_FINDA_IN_LOOP", self.cmd_load_filament_to_finda_in_loop
        )
        self.gcode.register_command("ENDSTOPS_STATUS", self.cmd_endstops_status)
        self.gcode.register_command("HOME_IDLER", self.cmd_home_idler)
        self.gcode.register_command("HOME_MMU", self.cmd_home_mmu)
        self.gcode.register_command("HOME_MMU_ONLY", self.cmd_home_mmu_only)
        self.gcode.register_command("PAUSE_MMU", self.cmd_pause)
        self.gcode.register_command("RESUME_MMU", self.cmd_resume)

        for i in range(self.number_of_tools):
            self.gcode.register_command(f"T{i}", partial(self.cmd_tx, tool_id=i))
            self.gcode.register_command(f"K{i}", partial(self.cmd_kx, tool_id=i))

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
        self.gcode.register_command("EJECT_RAMMING", self.cmd_eject_ramming)
        self.gcode.register_command(
            "UNLOAD_FILAMENT_IN_EXTRUDER_WITH_RAMMING",
            self.cmd_unload_filament_in_extruder_with_ramming,
        )
        self.gcode.register_command(
            "LOAD_FILAMENT_TO_FINDA", self.cmd_load_filament_to_finda
        )
        self.gcode.register_command(
            "LOAD_FILAMENT_FROM_FINDA_TO_EXTRUDER",
            self.cmd_load_filament_from_finda_to_extruder,
        )
        self.gcode.register_command(
            "LOAD_FILAMENT_TO_EXTRUDER", self.cmd_load_filament_to_extruder
        )
        self.gcode.register_command(
            "UNLOAD_FILAMENT_FROM_FINDA", self.cmd_unload_filament_from_finda
        )
        self.gcode.register_command(
            "UNLOAD_FILAMENT_FROM_EXTRUDER_TO_FINDA",
            self.cmd_unload_filament_from_extruder_to_finda,
        )
        self.gcode.register_command(
            "UNLOAD_FILAMENT_FROM_EXTRUDER", self.cmd_unload_filament_from_extruder
        )
        self.gcode.register_command("M702", self.cmd_m702)
        self.gcode.register_command("EJECT_FROM_EXTRUDER", self.cmd_eject_from_extruder)
        self.gcode.register_command("EJECT_BEFORE_HOME", self.cmd_eject_before_home)

    @property
    def display_status(self) -> DisplayStatus:
        """Return the DisplayStatus instance.

        Returns:
            DisplayStatus: The LCD display to set messages.
        """
        if self._display_status is None:
            self._display_status = self.printer.lookup_object("display_status")

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
    def filament_switch_sensor(self) -> SwitchSensor:
        """Return the SwitchSensor.

        Returns:
            SwitchSensor: The switch sensor.
        """
        if self._filament_switch_sensor is None:
            self._filament_switch_sensor = self.printer.lookup_object(
                self.filament_switch_sensor_name
            )
        return self._filament_switch_sensor

    @property
    def filament_motion_sensor(self) -> EncoderSensor:
        """Return the EncoderSensor.

        Returns:
            EncoderSensor: The switch sensor.
        """
        if self._filament_motion_sensor is None:
            self._filament_motion_sensor = self.printer.lookup_object(
                self.filament_motion_sensor_name
            )
        return self._filament_motion_sensor

    @property
    def is_filament_present_in_extruder(self) -> bool:
        """Return if the filament present in the extruder filament switch sensor.

        Returns:
            bool: True if filament sensor is triggered, False otherwise.
        """
        return self.filament_switch_sensor.get_status(None)["filament_detected"]

    @property
    def is_filament_in_finda(self) -> bool:
        """Return if the filament is in FINDA or not.

        Returns:
            bool: True if the filament is present in FINDA, False otherwise.
        """
        print_time = self.toolhead.get_last_move_time()
        return bool(self.pulley_stepper_endstop.query_endstop(print_time))

    def disable_steppers(
        self, steppers: None | ManualStepper | list[ManualStepper] = None
    ) -> bool:
        """Disable all stepper motors.

        Args:
            steppers (None | list[ManualStepper]): If None all the steppers are
                disabled, if it is a list, only the given steppers are
                disabled.

        Returns:
            bool: True, if all are successfully disabled, False otherwise.
        """
        if steppers is None:
            steppers = [self.pulley_stepper, self.selector_stepper, self.idler_stepper]
        elif isinstance(steppers, ManualStepper):
            steppers = [steppers]

        if not isinstance(steppers, list):
            return False

        for stepper in steppers:
            stepper.dwell(self.pause_before_disabling_steppers)
            stepper.do_enable(False)
            stepper.dwell(self.pause_after_disabling_steppers)

        return True

    def validate_filament_in_extruder(self) -> bool:
        """Call PAUSE_MMU if the filament is not detected by the filament sensor.

        Returns:
            bool: True if filament in extruder, False otherwise.
        """
        self.display_status_msg("Checking if filament in extruder")
        if not self.is_filament_present_in_extruder:
            self.display_status_msg("Filament not in extruder")
            self.pause()
            return False
        self.display_status_msg("Filament in extruder")
        return True

    def validate_filament_not_stuck_in_extruder(self) -> bool:
        """Validate filament is not stuck in extruder.

        Returns:
            bool: True if the filament is not present in FINDA, False otherwise.
        """
        self.display_status_msg("Checking if filament stuck in extruder")
        if self.is_filament_present_in_extruder:
            self.display_status_msg("Filament stuck in extruder")
            self.pause()
            return False
        self.display_status_msg("Filament not in extruder")
        return True

    def validate_filament_is_in_finda(self) -> bool:
        """Validate filament is in FINDA.

        Returns:
            bool: True if filament is in FINDA, False otherwise.
        """
        self.display_status_msg("Checking if filament in FINDA")
        if not self.is_filament_in_finda:
            self.display_status_msg("Filament not in FINDA")
            self.pause()
            return False
        self.display_status_msg("Filament in FINDA")
        return True

    def validate_filament_not_stuck_in_finda(self) -> bool:
        """Validate filament is not stuck in FINDA.

        Returns:
            bool: True if filament is not stuck in FINDA, False otherwise.
        """
        self.display_status_msg("Checking if filament stuck in FINDA")
        if self.is_filament_in_finda:
            self.display_status_msg("Filament stuck in FINDA")
            self.pause()
            return False
        self.display_status_msg("Filament not in FINDA")
        return True

    def validate_hotend_is_hot_enough(self) -> bool:
        """Validate if the hotend is hot enough.

        Pauses if hotend is not hot enough.

        Returns:
            bool: True if hotend is hot enough, False otherwise.
        """
        self.display_status_msg("Checking if hotend is too cold")
        print_time = self.toolhead.get_last_move_time()
        if self.extruder_heater.get_temp(print_time)[0] < self.min_temp_extruder:
            self.display_status_msg("Hotend is too cold")
            self.pause()
            return False
        return True

    def home_idler(self) -> None:
        """Home the idler.

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        # Home the idler
        self.display_status_msg("Homing idler")
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
        self.disable_steppers(self.idler_stepper)

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

        filament_switch_sensor_state = None
        if self.filament_switch_sensor and self.filament_switch_sensor.runout_helper.sensor_enabled:
            self.respond_info("Disabling filament runout sensor!")
            filament_switch_sensor_state = self.filament_switch_sensor.runout_helper.sensor_enabled
            self.filament_switch_sensor.runout_helper.sensor_enabled = False

        self.is_homed = True
        self.display_status_msg("Homing MMU ...")
        if not self.eject_before_home():
            return False

        self.respond_debug("Before home_mmu_only inside home_mmu")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        home_mmu_only_result = self.home_mmu_only()
        self.respond_debug("After home_mmu_only inside home_mmu")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        if filament_switch_sensor_state is not None:
            self.respond_info("Re-Enabling filament runout sensor!")
            self.filament_switch_sensor.runout_helper.sensor_enabled = filament_switch_sensor_state

        return home_mmu_only_result

    def home_mmu_only(self) -> bool:
        """Home the MMU.

        Follow the steps:

        1) home the idler
        2) home the selector (if needed)
        3) try to load filament 0 to FINDA and then unload it. Used to verify
           the MMU3 gear

        if all is ok, the MMU3 is ready to be used

        Returns:
            bool: True, if mmu homed, False otherwise.
        """
        if self.is_paused:
            self.display_status_msg("Homing MMU failed, MMU is paused, unlock it ...")
            return False

        self.home_idler()
        if not self.enable_no_selector_mode:
            self.display_status_msg("Homing selector")
            self.selector_stepper.do_set_position(0)
            self.selector_stepper.do_homing_move(
                -76,
                self.selector_homing_speed,
                self.selector_accel,
                True,
                True,
            )
            self.selector_stepper.do_set_position(0)
            self.disable_steppers(self.selector_stepper)

        # self.idler_stepper.do_move(
        #     0,
        #     self.idler_stepper.velocity,
        #     self.idler_stepper.accel,
        # )
        self.current_tool = None
        self.current_filament = None
        self.disable_steppers(self.idler_stepper)
        self.display_status_msg("Move selector to filament 0")
        self.select_tool(0)
        self.unselect_tool()
        self.is_homed = True
        self.display_status_msg("Homing MMU ended ...")

        self.disable_steppers()

        return True

    def load_filament_to_finda_in_loop(self) -> bool:
        """Load the filament to FINDA in a infinite loop.

        Args:
            gcmd (GCodeCommand): The G-code command.

        Returns:
            bool: True, if filament loaded to FINDA, False otherwise.
        """
        for i in range(self.finda_load_retry):
            self.pulley_stepper.do_set_position(0)
            self.pulley_stepper.do_homing_move(
                self.finda_load_length,
                self.finda_load_speed,
                self.finda_load_accel,
                True,
                False,
            )
            self.toolhead.wait_moves()

            # check endstop status and exit from the loop
            if self.is_filament_in_finda:
                self.display_status_msg(
                    "FINDA endstop triggered. Exiting filament load."
                )
                return True
            self.display_status_msg(f"FINDA endstop not triggered. Retrying... {i + 1}")
        self.display_status_msg(
            f"Couldn't load filament to FINDA after {self.finda_load_retry} tries!"
        )
        self.pause()
        return False

    def pause(self) -> None:
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
            PAUSE
            G90
            G1 X{self.pause_position[0]} Y{self.pause_position[1]} F3000
            M300
            M300
            M300
        """)

    def resume(self) -> None:
        """Resume the MMU."""
        self.gcode.run_script_from_command(
            """
            M118 End PAUSE
            RESTORE_GCODE_STATE NAME=PAUSE_MMU_state
            """
        )

    def unlock(self) -> None:
        """Park the idler, stop the delayed stop of the heater.

        Args:
            gcmd GCodeCommand: The G-code command.
        """
        self.display_status_msg("Resume print")
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
            self.display_status_msg("Could not select tool, MMU is not homed")
            return False

        if tool_id is None or tool_id < 0:
            return False

        self.display_status_msg(f"Select Tool {tool_id} ...")
        self.idler_stepper.do_move(
            self.idler_positions[tool_id],
            self.idler_stepper.velocity,
            self.idler_stepper.accel,
        )

        if not self.enable_no_selector_mode:
            self.selector_stepper.do_move(
                self.selector_positions[tool_id],
                self.selector_speed,
                self.selector_accel,
            )
            self.disable_steppers(self.selector_stepper)
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
            self.display_status_msg("Could not unselect tool, MMU is not homed")
            return False

        if self.current_tool is not None:
            self.display_status_msg(f"Unselecting Tool T{self.current_tool}")
        else:
            self.respond_info("Unselecting tool while Current Tool is None!")

        self.idler_stepper.do_move(
            self.idler_home_position,
            self.idler_stepper.velocity,
            self.idler_stepper.accel,
        )
        self.current_tool = None
        self.disable_steppers(self.idler_stepper)

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        self.display_status_msg("Unselect Tool is complete!")
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

        self.display_status_msg("Retry loading ...")
        print_time = self.toolhead.get_last_move_time()
        if self.is_paused:
            self.display_status_msg("Printer is paused ...")
            return False

        if self.extruder_heater.get_temp(print_time)[0] < self.min_temp_extruder:
            self.display_status_msg("Hotend is not hot enough ...")
            return False

        self.display_status_msg("Loading Filament...")
        self.gcode.run_script_from_command("G91")
        self.select_tool(self.current_filament)

        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            10,
            self.pulley_stepper.velocity,
            self.pulley_stepper.accel,
        )

        self.pulley_stepper.do_set_position(0)
        self.disable_steppers(self.pulley_stepper)

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

        self.display_status_msg("Loading Filament...")
        self.gcode.run_script_from_command("G91")
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            20,
            self.idler_load_to_extruder_speed,
            self.pulley_stepper.accel,
        )
        self.pulley_stepper.do_set_position(0)
        self.disable_steppers(self.pulley_stepper)
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
        if not self.validate_filament_in_extruder():
            self.respond_debug(f"self.current_tool    : {self.current_tool}")
            self.respond_debug(f"self.current_filament: {self.current_filament}")
            return False

        self.display_status_msg("Load Complete")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        return True

    def retry_unload_filament_in_extruder(self) -> None:
        """Retry unload, try correct misalignment of bondtech gear."""
        if not self.is_filament_present_in_extruder:
            return True

        self.display_status_msg("Retry unloading ....")
        if self.is_paused:
            self.display_status_msg("MMU is paused")
            return False

        print_time = self.toolhead.get_last_move_time()
        if self.extruder_heater.get_temp(print_time)[0] < self.min_temp_extruder:
            self.display_status_msg("Hotend is too cold")
            return False

        self.display_status_msg("Unloading Filament...")
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
            self.display_status_msg("No filament in extruder")
            return True

        if self.current_tool is not None:
            # self.respond_info("Tool selected, UNSELECT it")
            # self.pause()
            # return False
            self.respond_info(f"Tool T{self.current_tool} selected!")
            self.respond_info("Auto unselecting it!")
            self.respond_info(f"Auto unselecting T{self.current_tool}")
            self.unselect_tool()

        self.display_status_msg("Unloading Filament...")
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

        if not self.validate_filament_not_stuck_in_extruder():
            return False

        self.display_status_msg("Filament removed")
        return True

    def ramming_slicer(self) -> None:
        """Call the ramming process."""
        self.gcode.run_script_from_command("RAMMING_SLICER")

    def eject_ramming(self) -> bool:
        """Eject the filament with ramming from the extruder nozzle to the MMU3.

        Returns:
            bool: True if ejected, False otherwise.
        """
        if self.is_paused:
            return False

        if self.current_filament is None:
            return False

        self.display_status_msg(f"UT {self.current_filament} ...")
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

        if self.current_tool is not None:
            # # current tool is not None
            # self.respond_info("Tool selected, UNSELECT it")
            # self.pause()
            # return False
            self.respond_info(f"Tool T{self.current_tool} selected!")
            self.respond_info("Auto unselecting it!")
            self.display_status_msg(f"Auto unselecting T{self.current_tool}")
            self.unselect_tool()

        self.display_status_msg("Ramming and Unloading Filament...")
        self.ramming_slicer()
        if not self.unload_filament_in_extruder():
            return False
        self.display_status_msg("Filament rammed and removed")
        return True

    def load_filament_to_finda(self) -> bool:
        """Load filament until the FINDA detect it.

        Then push it 10mm more to be sure is well detected.
        PAUSE_MMU is called if the FINDA does not detect the filament

        Returns:
            bool: True, if the filament is loaded to FINDA, False otherwise.
        """
        if self.is_paused:
            return False

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        if self.current_tool is None:
            self.display_status_msg("Cannot load to FINDA, tool not selected !!")
            return False

        self.display_status_msg("Loading filament to FINDA ...")
        if not self.load_filament_to_finda_in_loop():
            return False

        self.pulley_stepper.do_set_position(0)
        # self.pulley_stepper.do_move(
        #     10,
        #     self.pulley_stepper.velocity,
        #     self.pulley_stepper.accel,
        # )
        self.disable_steppers(self.pulley_stepper)

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        if not self.validate_filament_is_in_finda():
            return False

        self.current_filament = self.current_tool
        self.display_status_msg("Loading done to FINDA")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        return True

    def load_filament_from_finda_to_extruder(self) -> bool:
        """Load from the FINDA to the extruder gear.

        Returns:
            bool: True, if filament is loaded from FINDA to extruder, False
                otherwise.
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            self.display_status_msg("Cannot load to extruder, tool not selected !!")
            return False

        self.display_status_msg("Loading filament from FINDA to extruder ...")
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
        self.disable_steppers(self.pulley_stepper)
        self.display_status_msg("Loading done from FINDA to extruder")

        return True

    def load_filament_to_extruder(self) -> bool:
        """Load from MMU3 to extruder gear by calling LOAD_FILAMENT_TO_FINDA.

        Then LOAD_FILAMENT_FROM_FINDA_TO_EXTRUDER.
        PAUSE_MMU is called if the FINDA does not detect the filament.

        Args:
            bool: True, if filament is loaded to extruder
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            self.display_status_msg("Cannot load to extruder, tool not selected !!")
            return False

        self.display_status_msg("Loading filament from MMU to extruder ...")
        if self.enable_no_selector_mode is False and not self.load_filament_to_finda():
            return False

        if self.load_filament_from_finda_to_extruder():
            self.display_status_msg("Loading done from MMU to extruder")
            return True
        # there should be an error about loading from FINDA to extruder
        return False

    def unload_filament_from_finda(self) -> None:
        """Unload filament until the FINDA detect it.

        Then push it -10mm more to be sure is well not detected.
        PAUSE_MMU is called if the FINDA does detect the filament.

        Returns:
            bool: True, if filament unloaded from FINDA, False otherwise.
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            if self.current_filament is not None:
                # Auto select tool
                self.select_tool(self.current_filament)
            else:
                self.display_status_msg(
                    "Cannot unload from FINDA, tool not selected !!"
                )
                return False

        self.display_status_msg("Unloading filament from FINDA ...")
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            -self.finda_unload_length,
            self.finda_unload_speed,
            self.finda_unload_accel,
        )
        self.pulley_stepper.do_set_position(0)
        self.disable_steppers(self.pulley_stepper)
        if not self.validate_filament_not_stuck_in_finda():
            return False
        self.current_filament = None
        self.display_status_msg("Unloading done from FINDA")
        return True

    def unload_filament_from_extruder_to_finda(self) -> bool:
        """Unload from extruder gear to the FINDA.

        Returns:
            bool: True, if filament unloaded from extruder to FINDA.
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            if self.current_filament is not None:
                # Auto select tool
                self.select_tool(self.current_filament)
            else:
                self.display_status_msg(
                    "Cannot unload from extruder to FINDA, tool not selected !!"
                )
                return False

        self.display_status_msg("Unloading filament from extruder to FINDA ...")
        self.pulley_stepper.do_set_position(0)
        if not self.enable_no_selector_mode:
            self.pulley_stepper.do_homing_move(
                -self.bowden_unload_length,
                self.bowden_unload_speed,
                self.bowden_unload_accel,
                False,
                False,
            )

            # if filament is still in finda, get into an unload loop...
            if (
                self.is_filament_in_finda
                and not self.unload_filament_to_finda_in_loop()
            ):
                return False

            if not self.validate_filament_not_stuck_in_finda():
                return False
        else:
            self.pulley_stepper.do_move(
                -self.bowden_unload_length,
                self.bowden_unload_speed,
                self.bowden_unload_accel,
            )
        self.disable_steppers(self.pulley_stepper)
        self.display_status_msg("Done unloading from FINDA!")
        return True

    def unload_filament_to_finda_in_loop(self) -> bool:
        """Unload the filament to FINDA in a loop.

        Args:
            gcmd (GCodeCommand): The G-code command.

        Returns:
            bool: True, if filament unloaded to FINDA, False otherwise.
        """
        for i in range(self.finda_unload_retry):
            self.pulley_stepper.do_set_position(0)
            self.pulley_stepper.do_homing_move(
                -self.finda_unload_length,
                self.finda_unload_speed,
                self.finda_unload_accel,
                False,
                False,
            )
            self.toolhead.wait_moves()

            # check endstop status and exit from the loop
            if not self.is_filament_in_finda:
                self.display_status_msg(
                    "FINDA endstop triggered. Exiting filament unload."
                )
                return True
            self.display_status_msg(f"FINDA endstop not triggered. Retrying... {i + 1}")
        self.display_status_msg(
            f"Couldn't unload filament to FINDA after {self.finda_unload_retry} tries!"
        )
        self.pause()
        return False

    def unload_filament_from_extruder(self) -> bool:
        """Unload from the extruder gear to the MMU3.

        Do it by calling UNLOAD_FILAMENT_FROM_EXTRUDER_TO_FINDA and
        then UNLOAD_FILAMENT_FROM_FINDA

        Returns:
            bool: True, if filament unloaded from the extruder, False otherwise.
        """
        if self.is_paused:
            return False

        if self.current_tool is None:
            if self.current_filament is not None:
                # Auto select tool
                self.select_tool(self.current_filament)
            else:
                self.display_status_msg(
                    "Cannot unload from extruder to MMU, tool not selected !!"
                )
                return False

        self.display_status_msg("Unloading filament from extruder to MMU ...")
        if not self.unload_filament_from_extruder_to_finda():
            return False

        if self.enable_no_selector_mode:
            self.display_status_msg("Unloading done from extruder to MMU")
            return True

        if not self.unload_filament_from_finda():
            return False

        self.display_status_msg("Unloading done from extruder to MMU")
        return True

    def cut_filament(self, tool_id: int) -> bool:
        """Cut the filament in the MMU3.

        Perform the cut from right to left.

        Args:
            tool_id (int): The tool id.

        Returns:
            bool: True, if filament is cut, False otherwise.
        """
        if self.is_paused:
            return False

        if self.enable_no_selector_mode:
            self.display_status_msg("Cannot perform cut in 5in1 mode!")
            return False

        self.display_status_msg(f"Cutting filament T{tool_id} ...")

        # First unload filament
        if not self.unload_tool():
            self.display_status_msg("Apparently unload tool failed!")
            return False

        # Select tool
        if not self.select_tool(tool_id):
            return False

        # Feed to FINDA
        if not self.load_filament_to_finda():
            return False

        # Unload filament from FINDA
        if not self.unload_filament_from_finda():
            return False

        # Prepare blade
        # - move the idler to the current tool position,
        #   to keep the filament tight in place.
        # - move the selector to the 0 position
        self.idler_stepper.do_move(
            self.idler_positions[tool_id],
            self.idler_stepper.velocity,
            self.idler_stepper.accel,
        )
        # move the selector to 0 position or close to 0
        self.selector_stepper.do_move(
            5,
            self.selector_speed,
            self.selector_accel,
        )

        # Push filament
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            self.cut_filament_length + self.cutting_edge_retract,
            self.pulley_stepper.velocity,
            self.pulley_stepper.accel,
        )

        # Unlock the selector
        # Perform the cut by moving to the current slot
        # set stepper current
        # decrease driver_SGTHRS
        # SET_TMC_FIELD
        # SET_TMC_CURRENT
        # TODO: Use the Python API for this, maybe a context manager with `with`?
        stepper_name = SELECTOR_STEPPER_NAME.split(" ")[-1]
        self.gcode.run_script_from_command(f"""
            SET_TMC_FIELD STEPPER={stepper_name} FIELD=SGTHRS VALUE=0
            SET_TMC_CURRENT STEPPER={stepper_name} CURRENT={self.cut_stepper_current}
        """)

        # do cut
        self.selector_stepper.do_move(
            self.selector_positions[tool_id],
            self.selector_homing_speed,
            0,
        )

        # return the stepper current and threshold to normal
        # TODO: This is manual for now
        self.gcode.run_script_from_command(f"""
            SET_TMC_FIELD STEPPER={stepper_name} FIELD=SGTHRS VALUE=96
            SET_TMC_CURRENT STEPPER={stepper_name} CURRENT=0.580
        """)

        # Pull filament back from the cutting edge
        self.pulley_stepper.do_set_position(0)
        self.pulley_stepper.do_move(
            -self.cutting_edge_retract,
            self.pulley_stepper.velocity,
            self.pulley_stepper.accel,
        )

        # Home the mmu
        self.home_mmu()

        self.display_status_msg(f"Done cutting T{tool_id}!")
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
        self.display_status_msg(f"LT {tool_id}")
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
            self.display_status_msg("Current filament is None!")
            if self.is_filament_in_finda:
                self.display_status_msg("Filament in FINDA!")
                self.respond_info("But there is a filament in FINDA!")
                if self.current_tool is None:
                    self.display_status_msg("Current Tool is also None!")
                    self.display_status_msg("Cancelling unload!!!")
                    return False
                self.display_status_msg(f"Current Tool is {self.current_tool}")
                self.current_filament = self.current_tool
                self.display_status_msg(
                    f"Also setting Current filament to {self.current_filament}"
                )
                return True
            # filament is not in FINDA
            self.display_status_msg("And no filament in FINDA")
            self.display_status_msg("No need to unload!")
            return True

        self.display_status_msg(f"UT {self.current_filament}")
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
            self.display_status_msg("Filament not in extruder")
            return True

        self.display_status_msg("Filament in extruder, trying to eject it ...")
        self.display_status_msg("Preheat Nozzle")
        print_time = self.toolhead.get_last_move_time()
        min_temp = max(
            self.extruder_heater.get_temp(print_time)[0], self.extruder_eject_temp
        )
        self.gcode.run_script_from_command(f"M109 S{min_temp}")
        if not self.unload_filament_in_extruder_with_ramming():
            return False
        self.gcode.run_script_from_command("M104 S0")
        return True

    def eject_before_home(self) -> None:
        """Eject from extruder gear to MMU3.

        Returns:
            bool: True, if filament ejected, False otherwise.
        """
        self.display_status_msg("Eject Filament if loaded ...")
        if self.is_filament_present_in_extruder:
            if not self.eject_from_extruder():
                return False
            if not self.validate_filament_not_stuck_in_extruder():
                return False

        if not self.enable_no_selector_mode:
            if self.is_filament_in_finda:
                if not self.unload_filament_from_extruder():
                    return False
                if not self.validate_filament_not_stuck_in_finda():
                    return False
                self.display_status_msg("Filament ejected !")
            else:
                self.display_status_msg("Filament already ejected !")
        else:
            self.display_status_msg("Filament already ejected !")

        return True

    @gcmd_grabber
    def cmd_endstops_status(self, gcmd: GCodeCommand) -> None:
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
        # gcmd.respond_info(f"is_filament_in_finda: {self.is_filament_in_finda}")
        self.respond_info(
            f"{STEPPER_NAME_MAP[SELECTOR_STEPPER_NAME]} : "
            f"{self.selector_stepper_endstop.query_endstop(print_time)}"
        )

        # _ = self.idler_stepper

    @gcmd_grabber
    def cmd_home_idler(self, gcmd: GCodeCommand) -> None:
        """Home the idler.

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        self.home_idler()

    @gcmd_grabber
    def cmd_home_mmu(self, gcmd: GCodeCommand) -> None:
        """Home the MMU.

        Eject filament if loaded with EJECT_BEFORE_HOME
        next home the mmu with HOME_MMU_ONLY

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        self.home_mmu()

    @gcmd_grabber
    def cmd_home_mmu_only(self, gcmd: GCodeCommand) -> None:
        """Home the MMU.

        Follow the steps:

        1) home the idler
        2) home the selector (if needed)
        3) try to load filament 0 to FINDA and then unload it. Used to verify
           the MMU3 gear

        if all is ok, the MMU3 is ready to be used

        Args:
            gcmd (GcodeCommand): The G-code command.
        """
        self.home_mmu_only()

    @gcmd_grabber
    def cmd_load_filament_to_finda_in_loop(self, gcmd: GCodeCommand) -> None:
        """Load the filament to FINDA in a infinite loop.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_to_finda_in_loop()

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
    def cmd_resume(self, gcmd: GCodeCommand) -> None:
        """Resume the MMU.

        Args:
            gcmd: (GCodeCommand): The G-code command.
        """
        self.resume()

    @gcmd_grabber
    def cmd_tx(self, gcmd: GCodeCommand, tool_id: int = 0) -> None:
        """The generic Tx command.

        Args:
            gcmd (GCodeCommand): The G-code command.
            tool_id (int, optional): The tool id to load. Defaults to 0.
        """
        self.display_status_msg(f"Requested tool {tool_id}")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        if self.current_filament == tool_id:
            return
        self.display_status_msg(f"Change Tool T{tool_id}")
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        filament_switch_sensor_state = None
        if self.filament_switch_sensor and self.filament_switch_sensor.runout_helper.sensor_enabled:
            self.respond_info("Disabling filament runout sensor!")
            filament_switch_sensor_state = self.filament_switch_sensor.runout_helper.sensor_enabled
            self.filament_switch_sensor.runout_helper.sensor_enabled = False

        filament_motion_sensor_state = None
        if self.filament_motion_sensor and self.filament_motion_sensor.runout_helper.sensor_enabled:
            self.respond_info("Disabling filament motion sensor!")
            filament_motion_sensor_state = self.filament_motion_sensor.runout_helper.sensor_enabled
            self.filament_motion_sensor.runout_helper.sensor_enabled = False


        if not self.unload_tool():
            self.respond_info("Apparently unload tool failed!")
            self.respond_debug(f"self.current_tool    : {self.current_tool}")
            self.respond_debug(f"self.current_filament: {self.current_filament}")

            if filament_switch_sensor_state:
                self.respond_info("Re-Enabling filament runout sensor!")
                self.filament_switch_sensor.runout_helper.sensor_enabled = True

            if filament_motion_sensor_state:
                self.respond_info("Re-Enabling filament motion sensor!")
                # also update the event time so that the runout doesn't trigger
                # print_time = self.toolhead.get_last_move_time()
                # self.filament_motion_sensor._update_filament_runout_pos()
                event_time = self.reactor.monotonic() or self.toolhead.get_last_move_time()
                self.filament_motion_sensor.encoder_event(event_time, None)
                self.filament_motion_sensor.runout_helper.sensor_enabled = True
            return

        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")
        if not self.load_tool(tool_id):
            # TODO: if load fails, try unloading and loading again for several
            #       times, most of the time unloading and loading again solves
            #       the load issue especially after installing the filament
            #       encoder the filament path become problematic.
            #       Also after all attempts failed we can cut the tip of the
            #       filament and try again...

            if filament_switch_sensor_state:
                self.respond_info("Re-Enabling filament runout sensor!")
                self.filament_switch_sensor.runout_helper.sensor_enabled = True

            if filament_motion_sensor_state:
                self.respond_info("Re-Enabling filament motion sensor!")
                # also update the event time so that the runout doesn't trigger
                # print_time = self.toolhead.get_last_move_time()
                # self.filament_motion_sensor._update_filament_runout_pos()
                event_time = self.reactor.monotonic() or self.toolhead.get_last_move_time()
                self.filament_motion_sensor.encoder_event(event_time, None)
                self.filament_motion_sensor.runout_helper.sensor_enabled = True
            return
        self.respond_debug(f"self.current_tool    : {self.current_tool}")
        self.respond_debug(f"self.current_filament: {self.current_filament}")

        if filament_switch_sensor_state:
            self.respond_info("Re-Enabling filament runout sensor!")
            self.filament_switch_sensor.runout_helper.sensor_enabled = True

        if filament_motion_sensor_state:
            self.respond_info("Re-Enabling filament motion sensor!")
            # also update the event time so that the runout doesn't trigger
            # print_time = self.toolhead.get_last_move_time()
            # self.filament_motion_sensor._update_filament_runout_pos(print_time)
            event_time = self.reactor.monotonic() or self.toolhead.get_last_move_time()
            self.filament_motion_sensor.encoder_event(event_time, None)
            self.filament_motion_sensor.runout_helper.sensor_enabled = True

        self.display_status_msg(f"Done T{tool_id}")

    @gcmd_grabber
    def cmd_kx(self, gcmd: GCodeCommand, tool_id: int = 0) -> None:
        """The generic Kx command.

        Args:
            gcmd (GCodeCommand): The G-code command.
            tool_id (int, optional): The tool id to cut. Defaults to 0.
        """
        self.cut_filament(tool_id)

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
    def cmd_load_filament_to_finda(self, gcmd: GCodeCommand) -> None:
        """Load filament until the FINDA detect it.

        Then push it 10mm more to be sure is well detected.
        PAUSE_MMU is called if the FINDA does not detect the filament

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_to_finda()

    @gcmd_grabber
    def cmd_load_filament_from_finda_to_extruder(self, gcmd: GCodeCommand) -> None:
        """Load from the FINDA to the extruder gear.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_from_finda_to_extruder()

    @gcmd_grabber
    def cmd_load_filament_to_extruder(self, gcmd: GCodeCommand) -> None:
        """Load from MMU3 to extruder gear by calling LOAD_FILAMENT_TO_FINDA.

        Then LOAD_FILAMENT_FROM_FINDA_TO_EXTRUDER.
        PAUSE_MMU is called if the FINDA does not detect the filament.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.load_filament_to_extruder()

    @gcmd_grabber
    def cmd_unload_filament_from_finda(self, gcmd: GCodeCommand) -> None:
        """Unload filament until the FINDA detect it.

        Then push it -10mm more to be sure is well not detected.
        PAUSE_MMU is called if the FINDA does detect the filament.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_filament_from_finda()

    @gcmd_grabber
    def cmd_unload_filament_from_extruder_to_finda(self, gcmd: GCodeCommand) -> None:
        """Unload from extruder gear to the FINDA.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_filament_from_extruder_to_finda()

    @gcmd_grabber
    def cmd_unload_filament_from_extruder(self, gcmd: GCodeCommand) -> None:
        """Unload from the extruder gear to the MMU3.

        Do it by calling UNLOAD_FILAMENT_FROM_EXTRUDER_TO_FINDA and
        then UNLOAD_FILAMENT_FROM_FINDA

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_filament_from_extruder()

    @gcmd_grabber
    def cmd_m702(self, gcmd: GCodeCommand) -> None:
        """Unload filament if inserted into the IR sensor.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        self.unload_tool()
        if not self.enable_no_selector_mode:
            if not self.is_filament_in_finda:
                self.unselect_tool()
                self.display_status_msg("M702 ok ...")
            else:
                self.display_status_msg("M702 Error !!!")
        else:
            self.unselect_tool()
            self.current_filament = None
            self.display_status_msg("M702 ok ...")

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


def load_config_prefix(config: ConfigWrapper) -> MMU3:
    """Load the mmu3 config prefix.

    Args:
        config (ConfigWrapper): The config wrapper.

    Returns:
        MMU3: The MMU3 instance.
    """
    return MMU3(config)
