from __future__ import annotations

from typing import TYPE_CHECKING

from . import filament_switch_sensor

if TYPE_CHECKING:
    from gcode import GCodeCommand
    from mcu import MCU_endstop


PULLEY_STEPPER_NAME = "manual_stepper pulley_stepper"
SELECTOR_STEPPER_NAME = "manual_stepper selector_stepper"

STEPPER_NAME_MAP = {
    PULLEY_STEPPER_NAME: "PINDA",
    SELECTOR_STEPPER_NAME: "Selector",
}


class MMU3:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.query_endstops = self.printer.load_object(config, "query_endstops")
        self.gcode = self.printer.lookup_object("gcode")

        self._mcu = None
        self._toolhead = None
        self._pulley_stepper = None
        self._pulley_stepper_endstop = None
        self._selector_stepper = None
        self._selector_stepper_endstop = None

        # load config values
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
        self.pinda_load_accel = config.getint("pinda_load_accel", 800)
        self.pinda_unload_accel = config.getint("pinda_unload_accel", 800)
        # selector
        self.selector = config.getlist("selector", [73.5, 59.375, 45.25, 31.125, 17])
        # idler
        self.idler = config.getlist("idler", [5, 20, 35, 50, 65])
        self.idler_home_position = config.getfloat("idler_home_position", 85)
        self.idler_load_to_extruder_speed = config.getint(
            "idler_load_to_extruder_speed", 30
        )
        self.idler_unload_speed = config.getint("idler_unload_speed", 30)
        # pause values
        self.pause_before_disabling_steppers = config.getint(
            "pause_before_disabling_steppers", 50
        )
        self.pause_after_disabling_steppers = config.getint(
            "pause_after_disabling_steppers", 200
        )
        self.pause_x = config.getfloat("pause_x", 0)
        self.pause_y = config.getfloat("pause_y", 200)
        self.pause_z = config.getfloat("pause_z", 10)
        # temperature
        self.min_temp_extruder = config.getint("min_temp_extruder", 180)
        self.extruder_eject_temp = config.getint("extruder_eject_temp", 200)
        #
        self.enable_5in1 = config.getint("enable_5in1", 0)
        self.pinda_load_retry = config.getint("pinda_load_retry", 100)
        self.load_retry = config.getint("load_retry", 5)
        self.unload_retry = config.getint("unload_retry", 5)
        self.filament_sensor_name = config.get(
            "filament_sensor_name", "filament_switch_sensor my_filament_sensor"
        )
        self.filament_sensor_helper = filament_switch_sensor.RunoutHelper(config)

        # register commands
        self.register_commands()

    def register_commands(self):
        """Register new GCode commands."""
        self.gcode.register_command(
            "LOAD_FILAMENT_TO_PINDA_IN_LOOP", self.load_filament_to_pinda_in_loop
        )
        self.gcode.register_command("ENDSTOPS_STATUS", self.endstops_status)

    @property
    def toolhead(self):
        """Return the toolhead."""
        if self._toolhead is None:
            self._toolhead = self.printer.lookup_object("toolhead")
        return self._toolhead

    @property
    def pulley_stepper(self) -> None | object:
        """Return pulley stepper."""
        if self._pulley_stepper is None:
            steppers = self.pulley_stepper_endstop.get_steppers()
            for stepper in steppers:
                self.gcode.respond_info(
                    f"MMU3: stepper.__class__.__name__: {stepper.__class__.__name__}"
                )
                self.gcode.respond_info(f"MMU3: dir(stepper): {dir(stepper)}")
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
            # self.gcode.respond_info(f"MMU3: self.mcu.__class__.__name__: {self._mcu.__class__.__name__}")
            # self.gcode.respond_info(f"MMU3: dir(self.mcu): {dir(self._mcu)}")
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
    def is_filament_present_in_extruder(self) -> bool:
        """Return if the filament present in the extruder runout sensor.

        Returns:
            bool: True if filament sensor is triggered, False otherwise.
        """
        return self.filament_sensor_helper.get_status(None)["filament_detected"]

    @property
    def is_filament_in_pinda(self) -> bool:
        """Return if the filament is in pinda or not.

        Returns:
            bool: True if the filament is present in pinda, False otherwise.
        """
        print_time = self.toolhead.get_last_move_time()
        return bool(self.pulley_stepper_endstop.query_endstop(print_time))

    def endstops_status(self, gcmd) -> None:
        """Print the status of all endstops."""
        # Query the endstops
        print_time = self.toolhead.get_last_move_time()

        # Report results
        gcmd.respond_info("Endstop status")
        gcmd.respond_info("==============")
        gcmd.respond_info(f"Extruder : {self.is_filament_present_in_extruder}")
        gcmd.respond_info(
            f"{STEPPER_NAME_MAP[PULLEY_STEPPER_NAME]} : "
            f"{self.pulley_stepper_endstop.query_endstop(print_time)}"
        )
        # gcmd.respond_info(f"is_filament_in_pinda: {self.is_filament_in_pinda}")
        gcmd.respond_info(
            f"{STEPPER_NAME_MAP[SELECTOR_STEPPER_NAME]} : "
            f"{self.selector_stepper_endstop.query_endstop(print_time)}"
        )

    def load_filament_to_pinda_in_loop(self, gcmd: GCodeCommand) -> None:
        """Load the filament to pinda in a infinite loop.

        Args:
            gcmd (GCodeCommand): The G-code command.
        """
        # _ = self.mcu
        # _ = self.pulley_stepper
        while True:
            self.gcode.run_script_from_command(
                "MANUAL_STEPPER STEPPER=pulley_stepper SET_POSITION=0"
            )
            self.gcode.run_script_from_command(
                "MANUAL_STEPPER "
                "STEPPER=pulley_stepper "
                f"MOVE={self.pinda_load_length} "
                f"SPEED={self.pinda_load_speed} "
                f"ACCEL={self.pinda_load_accel} "
                "STOP_ON_ENDSTOP=2"
            )
            self.gcode.run_script_from_command("M400")
            # check endstop status and exit from the loop
            print_time = self.toolhead.get_last_move_time()
            pulley_endstop_status = self.pulley_stepper_endstop.query_endstop(
                print_time
            )

            if pulley_endstop_status:
                gcmd.respond_info(
                    "MMU3: Pinda endstop triggered. Exiting filament load."
                )
                break
            else:
                gcmd.respond_info("MMU3: Pinda endstop not triggered. Retrying...")
            i += 1


def load_config_prefix(config):
    return MMU3(config)
