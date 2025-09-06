from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from mcu import MCU_endstop



PULLEY_STEPPER_NAME = "manual_stepper pulley_stepper"

class MMU3:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.query_endstops = self.printer.load_object(config, "query_endstops")

        # load config values
        # timeouts
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
        # selector
        self.selector = config.getlist("selector", [73.5,59.375,45.25,31.125,17])
        # idler
        self.idler = config.getlist("idler", [5,20,35,50,65])
        self.idler_home_position = config.getfloat("idler_home_position", 85)
        self.idler_load_to_extruder_speed = config.getint("idler_load_to_extruder_speed", 30)
        self.idler_unload_speed = config.getint("idler_unload_speed", 30)
        # pause values
        self.pause_before_disabling_steppers = config.getint("pause_before_disabling_steppers", 50)
        self.pause_after_disabling_steppers = config.getint("pause_after_disabling_steppers", 200)
        self.pause_x = config.getfloat("pause_x", 0)
        self.pause_y = config.getfloat("pause_y", 200)
        self.pause_z = config.getfloat("pause_z", 10)
        # temperature
        self.min_temp_extruder = config.getint("min_temp_extruder", 180)
        self.extruder_eject_temp = config.getint("extruder_eject_temp", 200)
        #
        self.enable_5in1 = config.getint("enable_5in1", 0)
        self.pinda_load_retry = config.getint("pinda_load_retry", 100)
        self.load_retry = config.getint("load_retry", 5)
        self.unload_retry = config.getint("unload_retry", 5)

        # register commands
        self.register_commands()

    def register_commands(self):
        """Register new GCode commands."""
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("LOAD_FILAMENT_TO_PINDA_PY", self.load_filament_to_pinda_py)

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

    def load_filament_to_pinda_py(self, gcmd):
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info("Pinda endstop not triggered. Retrying...")
        pulley_stepper_endstop = self.get_endstop(PULLEY_STEPPER_NAME)
        toolhead = self.printer.lookup_object("toolhead")
        while True:
            gcode.run_script_from_command(
                "MANUAL_STEPPER STEPPER=pulley_stepper SET_POSITION=0"
            )
            gcode.run_script_from_command(
                "MANUAL_STEPPER "
                "STEPPER=pulley_stepper "
                f"MOVE={self.pinda_load_length} "
                f"SPEED={self.pinda_load_speed} "
                f"ACCEL={self.pinda_load_accel} "
                "STOP_ON_ENDSTOP=2"
            )
            gcode.run_script_from_command("M400")

            # check endstop status and exit from the loop
            print_time = toolhead.get_last_move_time()
            pulley_endstop_status = pulley_stepper_endstop.query_endstop(print_time)

            if pulley_endstop_status:
                gcode.respond_info("Pinda endstop triggered. Exiting filament load.")
                break
            else:
                gcode.respond_info("Pinda endstop not triggered. Retrying...")
            i += 1



def load_config_prefix(config):
    return MMU3(config)
