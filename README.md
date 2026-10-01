# TMC5160 for STM32CubeIDE

This repository contains a header-only C TMC5160 library and an STM32G474
CubeIDE example. The library works from C and C++. It has two layers:
named register operations and motor functions in radians, rad/s and rad/s².
`robot_config.h` defines the board pins and motor constants. No motor
instance or handle is passed to the API.

## Use in another CubeIDE project

1. Copy both `Core/Inc/tmc5160.h` and `Core/Inc/robot_config.h` into
   the new project's include directory. Edit the pin and motor defines
   in `robot_config.h`. SPI handle and CS are required. Remove the
   optional pin defines for signals strapped on the board.
2. Enable an SPI peripheral in full-duplex master mode, 8-bit, MSB first,
   CPOL=1 and CPHA=1 (SPI mode 3). Configure CS as a software-controlled
   GPIO output, initially high.
3. Include `tmc5160.h` in the application and initialize the motor
   after Cube HAL has initialized GPIO and SPI:

```c
tmc5160_init();
tmc5160_set_pos(0.5);             /* Output shaft: 0.5 rad */
tmc5160_move(-0.25);              /* Output shaft: -0.25 rad/s */
double position = tmc5160_get_pos();
double velocity = tmc5160_get_velocity();
tmc5160_stop();
/* After the shaft has stopped: */
tmc5160_set_zero();
```

The configuration header contains motor type, gearing, microstep resolution,
chip clock, current, ramp profile, SPI and board GPIO. The library reads
these constants directly.
Runtime helpers include `tmc5160_set_velocity()`,
`tmc5160_set_acceleration()`, `tmc5160_set_current()`,
`tmc5160_set_motor_direction()`, `tmc5160_arm()` and
`tmc5160_disarm()`.
`tmc5160_set_pos()` uses the velocity from `robot_config.h`;
call `tmc5160_set_velocity()` afterwards to change speed during that move.

The library performs blocking HAL SPI transfers. Each register read sends
two 40-bit frames with CS released between them. Functions that set
configuration registers take the complete register value. The current
function writes all three fields of `IHOLD_IRUN` together; the global
scaler has its own register. A scaler value of 0 means full scale;
32–255 select reduced current. The actual motor current also depends on
the board's sense resistors.

The motor API uses output-shaft radians, rad/s and rad/s². The register
API uses raw TMC units and microsteps. Conversion uses the configured
chip clock, gearing and steps per motor revolution. `set_zero()` should
be called when the shaft is stationary. HAL transfer results are not
verified.

## Example project

Import the directory as an existing STM32CubeIDE project and build the
Debug configuration. The example uses SPI1, PA4 for CS and PC5 for
driver enable. `robot_config.h` contains the board's motor and startup
settings; `tmc5160_init()` applies them. The main loop moves the
target between 0 and 0.064 radians. If CubeIDE opens checked-in Debug artifacts
from another machine, run **Project → Clean** before building.
