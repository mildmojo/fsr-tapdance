# fsr-tapdance

Arduino code to make one or more force-sensing resistors (FSRs) look like a
simple switch that activates when pushed. Used for 3D printer bed probing.

The code reads a high-resolution I<sup>2</sup>C-connected analog-to-digital
converter, automatically calculating trigger and recovery thresholds, and
outputting a digital signal with a falling edge on trigger.

## Status

The code's at about a v1.0 state.

I'm actively using it with Klipper on my Deltaprintr for bed height probing and
(sometimes) levelling. I haven't cleaned out cruft from the development process.
There's commented-out code from experiments that didn't work, and some stub code
from stuff I never got around to trying.

## Quick start: Installing firmware on a microcontroller

0. Install the [PlatformIO CLI](https://platformio.org/install/cli) (or the IDE).
1. Find your microcontroller [in the PlatformIO docs](https://docs.platformio.org/en/latest/boards/index.html).
   Configure `platformio.ini` for your board.
2. Edit `configuration.h` and adjust `OUTPUT_PIN` and optionally `PROBE_ENABLE_PIN`
   to match the pins you've connected between the MCU and the printer controller
   board.
3. `pio pkg install` to install dependencies.
4. `pio run --list-targets` to list configured boards.
5. `pio run -t upload -e teensylc` to upload firmware to your connected MCU
   (where `upload` is the `Name` field from the previous step, and `teensylc` is
   the `Environment`).

You can also use e.g. `pio run -e teensylc` to compile the code without
uploading, and `pio check` to check the source code for errors and style issues
if you've made changes.

## Tuning

Disconnect the MCU's VCC line from your printer board and connect the MCU to
your PC over USB. Use the Arduino IDE's [serial plotter](https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-serial-plotter/)
to view readings and thresholds as the system runs. Try tapping the FSRs to see
if you can trigger the output.

In `configuation.h`:

- You may need to tune `ADC_GAIN` to a "zoom in" on the voltage range you're
  getting from your FSRs. If you see full-scale readings above 32K, you may want
  to switch to the next-larger range. If you never see readings above 16K, you
  may want to switch to the next-smaller range. See range constants in the
  [ADS1115_WE library](https://github.com/wollewald/ADS1115_WE/blob/master/src/ADS1115_WE.h#L94).
- Adjust `FSR_TRIGGER_MULTIPLIER`, making it higher to move the trigger threshold
  farther above the average reading, or making it lower to move the trigger
  down closer to the average.
- Adjust `FSR_RECOVERY_MULTIPLIER` the same way, to adjust the recovery threshold
  relative to the trigger threshold.
- Adjust `TRIGGER_UPDATE_SAMPLE_COUNT` if the average reading is too flat and
  slow to update, or too spiky. Make the count lower if it's flat and slow, or
  higher if the average reading is bouncing very quickly by a large amount when
  idle.

## History

The original Deltaprintr 3D printer used force-sensing resistors under the build
plate to detect the nozzle tapping the surface. It used this system with Marlin
firmware to compute a bed mesh before each print for a better first layer. It
was never the most reliable system, leading to areas that were detected too high
or too low.

When I switched the printer to [Klipper](https://www.klipper3d.org/) firmware,
I needed to replicate this system on a microcontroller to translate analog
FSR readings to a digital probe pin signal. I dug a long-discontinued Digispark
Pro out of my parts bin and began adapting the author's ancient original code to
run on it.

This started as a modification to the ATTINY_TRINKET_AUTOFSR code from
[WingTangWong/AutoTuningFSRTrigger](https://github.com/WingTangWong/AutoTuningFSRTrigger).
It diverged far enough that I decided to do a rewrite from scratch and port it
to a Teensy LC and a SparkFun Pro Micro RP2040, using an outboard ADS1115 16-bit
ADC to improve sensitivity.

## Code improvements

The original code calculated trigger thresholds once on startup, but I always
saw FSR readings drift over time until the signal was stuck on.

The current code, when not triggered, takes a running average of analog readings
and continually recalculates trigger and recovery thresholds to account for
sensor drift.

Additionally, the MCU's built-in 10-bit ADC resolution wasn't fine enough to
detect nozzle taps when operating near the limits of the detectable force range
on the FSRs, so the code now reads from an I<sup>2</sup>C-connected 16-bit ADC
instead.

If you leave the MCU connected to a host PC while it's running, you'll get a
dump of readings and thresholds over serial that you can graph with the Arduino
IDE's [serial plotter](https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-serial-plotter/).

## Circuit

(TODO: Fritzing diagram maybe someday?)

Parts:
- 3 FSRs ([low-force](https://www.adafruit.com/product/166) for acrylic bed, [high-force](https://www.adafruit.com/product/5475) for glass bed),
  connected in parallel
- 1 [ADS1115 16-bit ADC](https://www.adafruit.com/product/1085) breakout
- 1 Arduino-compatible microcontroller (Teensy LC, Trinket, Feather, Raspberry
  Pi Pico, etc.)

Connections:
- FSRs are connected between VCC and one channel on the ADS1115 ADC
- MCU is connected to the ADS1115 ADC over I<sup>2</sup>C
- MCU is connected to the printer's controller board for VCC, GND, digital signal
  (output normally HIGH, output LOW on trigger)
- MCU is optionally connected to another printer controller pin to receive a
  probe enable signal from the printer just before it taps the bed

## Klipper config sample

Sample below, change pin names to suit your setup:

```ini
[output_pin probe_enable]
pin: PC6 # Optional output connected to MCU to signal threshold reset
value: 0

[probe]
pin: !PD2 # Input connected to MCU output trigger pin, inverted to detect falling edge
samples: 4
speed: 5
lift_speed: 30
samples_tolerance: 0.04
samples_tolerance_retries: 4
samples_result: median
# Send signal to MCU that it should recalibrate thresholds for an incoming probe
activate_gcode:
  SET_PIN PIN=probe_enable VALUE=1
deactivate_gcode:
  SET_PIN PIN=probe_enable VALUE=0
```

## Does it work well?

In Klipper, I get better results from a manual bed level, but that is _tedious_
for a zillion probe points. It does work well for a single bed height probe at
the start of a print.

I'm able to get about 0.04mm accuracy in under 10 retries. According to the
Klipper docs, that's not good enough for bed levelling (0.02mm), but honestly
the docs are written by perfectionists using printers far newer, stiffer, and
more precise than mine. 0.04mm is _fantastic_ for this printer, only a few
microsteps, and far better than I ever got with the old code.

A bigger problem that I never solved is FSR bias. Each sensor has a different
response to the same pressure, and I suspect that affects how quickly the system
registers bed taps over different parts of the bed, leading to inaccuracies in
the mesh.

I **don't** trust it for automatic delta calibration. I do that manually with a
strip of paper. It's only like 8 points, and manual readings work really well.

## License

This code is licensed under the GNU General Public License (GPL) version 2. See
the `LICENSE` file for details.
