# Two-Link Parallel Drawing Robot

**Project status:** Prototype / Under Validation

A two-link parallel drawing robot driven by two servo motors. The current software provides inverse-kinematics-based planar motion, a desktop visualization interface, text-contour extraction, and basic DXF line import. An Arduino sketch receives joint-angle commands over serial communication.

## Repository structure

- `Software/main.py` — Python desktop application and motion-planning logic.
- `Firmware/Arduino/servo_controller.ino` — Arduino firmware for the two servo motors.
- `requirements.txt` — Python dependencies.

## Current software parameters

| Parameter | Value |
|---|---:|
| Base pivot spacing, d | 48.0 mm |
| Link length, L1 | 108.8 mm |
| Link length, L2 | 131.2 mm |
| Text workspace width | 250.0 mm |
| Text workspace height | 100.0 mm |
| Text workspace center | (0, 118) mm |
| Serial baud rate | 9600 |
| Arduino servo pins | D9 and D10 |

These are the values currently defined in the supplied source code. Their correspondence to the final physical robot dimensions and calibration remains to be verified.

## Current features

- Planar inverse kinematics for a two-link parallel mechanism.
- Workspace boundary visualization.
- Text-to-contour conversion using Pillow, OpenCV, Arabic reshaping, and bidirectional text handling.
- Basic DXF import for entities of type `LINE`.
- Serial communication with an Arduino-based two-servo controller.
- Simulation fallback when the configured serial connection cannot be opened.

## Requirements

Install the Python dependencies:

```bash
pip install -r requirements.txt
```

Run the application:

```bash
python Software/main.py
```

The current source configures the serial port as `COM5` at 9600 baud. Change this setting in `Software/main.py` to match your system.

## Known limitations and validation work

- The current Arduino firmware controls only the two arm servos. No physical pen-lift actuator or pen-up/pen-down command is implemented.
- The Python path planner distinguishes drawing and travel moves in its internal path data, but that state is not transmitted to the Arduino. Separate contours may therefore be connected by unwanted drawn travel moves unless a pen-lift mechanism is added.
- DXF import currently reads only `LINE` entities; polylines, arcs, circles, and splines are not handled.
- Servo direction, zero offsets, mechanical limits, and angle calibration require physical verification.
- The visual workspace does not yet account for servo-angle limits or mechanical collision constraints.
- The serial port is hard-coded, and the current firmware uses a blocking line read.
- Motion accuracy, repeatability, and drawing performance have not yet been quantitatively characterized.

## Attribution

- Designer: Mohammadreza Goharimanesh
- Supervisor: Dr. Masoud Gohari-Manesh

The university affiliation is intentionally omitted until its official English name is confirmed.

## License

No license has been specified yet. All rights remain with the project author unless a license is added.
