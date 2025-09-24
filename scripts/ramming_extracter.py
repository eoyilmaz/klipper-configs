"""Extract ramming sequence from the given gcode."""
import re

RAMMING_TEST_GCODE = """
M104 S200
HOME_MMU
T0
G92 E0
G1 E70 F600
EJECT_BEFORE_HOME
UNLOCK_MMU
M104 S200
"""

GCODE_SEQUENCE = """
SET_PRESSURE_ADVANCE ADVANCE=0
G1  X143.396  E2.1091
G1 F1052
G1  X146.403  E0.1518
G1  Y144.240  F7200
G1 F1052
G1  X145.026  E0.0695
G1 F1180
G1  X140.111  E0.2481
G1 F1451
G1  X134.067  E0.3051
G1 F1769
G1  X126.694  E0.3722
G1 F1993
G1  X118.392  E0.4191
G1 F2248
G1  X109.026  E0.4728
G1 F2758
G1  X97.536  E0.5800
G1 F3379
G1  X87.903  E0.4862
G1  Y145.020  F7200
G1 F3379
G1  X92.352  E0.2246
G1 F3730
G1  X107.894  E0.7846
G1 F3842
G1  X123.901  E0.8080
G1 F4161
G1  X141.237  E0.8751
G1 F4750
G1  X146.403  E0.2608
G1  Y145.800  F7200
G1 F4750
G1  X131.776  E0.7384
G1 F5165
G1  X110.255  E1.0863
G1 F5245
G1  X88.403  E1.1031
;WIDTH:0.5
; Ramming start
; Retract(unload)
G1 E-15.0000 F1500
G1 E-7.7000 F600
G1 E-2.2000 F300
G1 E-1.1000 F180
G4 S0
M104 S215
; Cooling
G1  Y146.580
G1  X146.403  E5.0000 F696
G1  X88.403  E-5.0000
G1  X108.403  E11.5000 F1043
G1  X88.403  E5.0000 F2400
G1 E-16.5000 F600
G1  X146.403  E5.0000 F696
G1  X88.403  E-5.0000
G1  X108.403  E11.5000 F1043
G1  X88.403  E5.0000 F2400
G1 E-16.5000 F600
G1  X146.403  E5.0000 F696
G1  X88.403  E-5.0000
G1  X108.403  E11.5000 F1043
G1  X88.403  E5.0000 F2400
G1 E-16.5000 F600
G1  X146.403  E5.0000 F696
G1  X88.403  E-5.0000
; Cooling park
G4 S3.000
G1 E-2.0000 F2000
M73 P1 R623
; Ramming end
G1  Y146.440  F2400
G4 S0
G1 E-2 F5100
; filament end gcode
"""


def main():
    ramming_buffer = []
    g1_regex = re.compile(r"(G1  X)([0-9.\-\s]+)(E[0-9.\-]+)([\sF0-9]*)")
    g1_f_regex = re.compile(r"(G1 )([\sF0-9]*)")
    for line in GCODE_SEQUENCE.split("\n"):
        if (
            line.startswith(("SET_PRESSURE_ADVANCE", ";", "G1 F", "G1 E", "G4", "G91"))
        ):
            ramming_buffer.append(line)
        elif line.startswith("G1  Y") or line.startswith("M73 "):
            continue
        elif m:=g1_regex.match(line):
            ramming_buffer.append(f"G1 {m.groups()[2]}{m.groups()[3]}")

    # join G1 F's with next line
    for i, line in enumerate(ramming_buffer):
        if line.startswith("G1 F"):
            # join with the next line
            m = g1_f_regex.match(line)
            speed = m.groups()[1] if m else ""
            ramming_buffer[i] = ""
            ramming_buffer[i + 1] = f"{ramming_buffer[i + 1]} {speed}"

    # remove empty lines
    ramming_buffer = (
        """    G91
    G92 E0
    ;TYPE:Prime tower
    ;WIDTH:0.5
    ;--------------------
    ; CP TOOLCHANGE START
    M220 S100
    ; CP TOOLCHANGE UNLOAD
    ;WIDTH:0.65""".split("\n") +
    [f"    {line}" for line in ramming_buffer if line.strip()]
    )

    print("\n".join(ramming_buffer))



if __name__ == "__main__":
    main()