# Precision Ohmmeter for M5Stack Cardputer

A DIY precision ohmmeter using M5Stack Cardputer v1.1 and ADS1115 16-bit ADC module.

Measures resistance from **22 Ohm to 2 MOhm** with automatic ranging, adaptive averaging, smooth display output, and continuity test mode with audible buzzer.

## Features

- **16-bit precision** — ADS1115 ADC with 7.8 µV resolution at highest gain
- **Auto-PGA ranging** — automatically selects optimal gain (±0.256V to ±6.144V) for best accuracy
- **Wide range** — measures from 22 Ohm to 2 MOhm in a single mode
- **High-precision VDD calibration** — 128-sample calibration at startup, periodic recalibration during idle
- **Adaptive averaging** — 16 samples for low-R, up to 128 samples for high-R measurements
- **EMA smoothing** — exponential moving average filter for stable, non-jumping display
- **Continuity test** — press `S` to toggle audible buzzer mode (continuous 2 kHz tone when R < 50 Ohm)
- **Auto-units** — displays Ohm, kOhm, or MOhm automatically
- **Open circuit detection** — shows "OPEN" when probes are disconnected (with 3-reading confirmation)
- **Real-time info** — voltage, PGA range, VDD, and reference resistor shown on screen
- **Safe for boards** — max 0.5 mA test current through 10 kOhm reference resistor

## Tested Accuracy

| Resistor | Multimeter | Cardputer | Error |
|----------|-----------|-----------|-------|
| 46.6 Ohm | 46.6 Ohm | 46.4 Ohm | 0.4% |
| 995 Ohm | 995 Ohm | 994 Ohm | 0.1% |
| 101.4 kOhm | 101.4 kOhm | 101.1 kOhm | 0.3% |
| 299.9 kOhm | 299.9 kOhm | 298 kOhm | 0.6% |
| 467.5 kOhm | 467.5 kOhm | 465 kOhm | 0.5% |
| 1.009 MOhm | 1.009 MOhm | 1.002 MOhm | 0.7% |

## Hardware Required

- M5Stack Cardputer v1.1
- ADS1115 16-bit ADC module (blue breakout board)
- 10 kOhm resistor (1% tolerance recommended, measure exact value with multimeter)
- Grove cable or 4 wires for I2C connection
- 2 probe wires

## Circuit

```
Cardputer Grove Port A          ADS1115 Module
  5V (red wire)  ──────────────── VDD
  GND  ──────────────────────── GND
  GPIO 2 (SDA) ─────────────── SDA
  GPIO 1 (SCL) ─────────────── SCL

ADS1115 Measurement Circuit:
  VDD (5V) ── [R_known 10k] ── A0 ──── Probe 1
                                         |
  GND ──────────────────────────────── Probe 2

Connect unknown resistor between Probe 1 and Probe 2.
```

**Important:** On Cardputer v1.1, Grove Port A pins are reversed — SDA is GPIO 2, SCL is GPIO 1.

## How It Works

The circuit forms a voltage divider: VDD → R_known → junction (A0) → R_unknown → GND.

The ADS1115 measures voltage at the junction point. From this voltage and the known reference resistor value, we calculate the unknown resistance:

```
R_unknown = R_known × V_measured / (VDD - V_measured)
```

### Auto-PGA Ranging

The ADS1115 has a Programmable Gain Amplifier with 6 ranges. The firmware automatically selects the best range for maximum resolution:

| PGA Range | LSB Size | Best For |
|-----------|----------|----------|
| ±6.144V | 187.5 µV | High-R (>100 kOhm) |
| ±4.096V | 125 µV | Medium-high R |
| ±2.048V | 62.5 µV | Medium R |
| ±1.024V | 31.25 µV | Medium-low R |
| ±0.512V | 15.6 µV | Low R |
| ±0.256V | 7.8 µV | Very low R (<100 Ohm) |

### Adaptive Averaging

To reduce noise at high resistance values (where signal-to-noise ratio is worse), the firmware uses more samples:

| Resistance Range | Samples |
|-----------------|---------|
| < 10 kOhm | 16 |
| 10k - 50 kOhm | 32 |
| 50k - 200 kOhm | 64 |
| > 200 kOhm | 128 |

### Continuity Test Mode

Press `S` on the keyboard to toggle continuity mode on/off. When enabled:
- A continuous 2 kHz tone plays through the built-in speaker while resistance is below 50 Ohm
- The tone stops instantly when probes are separated
- A green "BEEP" indicator appears on screen
- Status shown in title bar: `[S] BEEP ON` / `[S] beep off`

The test current is limited to 0.5 mA by the 10 kOhm reference resistor, making it safe for probing PCB traces and components.

## Configuration

Edit these constants in `main.cpp` to match your hardware:

```cpp
const float R_KNOWN = 10040.0;  // Your reference resistor value in Ohms
                                // Measure it precisely with a multimeter!
```

VDD is auto-calibrated — leave probes open at startup for best accuracy.

## Building

### PlatformIO (recommended)

```bash
# Build
pio run

# Upload
pio run -t upload

# Serial monitor
pio device monitor --baud 115200
```

### Dependencies

- M5Cardputer library
- M5Unified
- M5GFX
- Adafruit ADS1X15

All dependencies are installed automatically by PlatformIO.

## Usage

1. Connect the ADS1115 module to Cardputer via Grove cable
2. Solder/connect the reference resistor between VDD and A0 on the ADS1115 module
3. Connect probe wires to A0 (Probe 1) and GND (Probe 2)
4. Flash the firmware
5. **Leave probes open at startup** — the device calibrates VDD automatically (128 samples)
6. Connect the unknown resistor between the probes
7. Read the value on screen
8. Press `S` to enable/disable continuity buzzer mode

## Keyboard Controls

| Key | Function |
|-----|----------|
| S | Toggle continuity test mode (buzzer on/off) |

## Display Layout

```
+--------------------------------------+
| OHMMETER v1.5       [S] beep off    |  <- Title bar + continuity status
|                                      |
|           10.04                      |  <- Big digits (resistance value)
|           kOhm                       |  <- Auto-units (or green BEEP bar)
|                                      |
| V: 0.4453V           PGA: 1.024V    |  <- Measured voltage & PGA range
| VDD:4.94V            Rref=10.04k    |  <- Supply voltage & reference R
+--------------------------------------+
```

## Version History

- **v1.5.1** — Continuity test mode with `S` key toggle and continuous buzzer
- **v1.4.1** — Improved 1 MOhm measurement: high-precision VDD calibration (128 samples), OPEN confirmation (3 reads)
- **v1.3.0** — Adaptive averaging (16-128 samples), EMA smoothing filter
- **v1.2.0** — Fixed OPEN threshold for high-R measurements (>220 kOhm)
- **v1.1.0** — Auto VDD calibration (Grove port is 5V, not 3.3V)
- **v1.0.0** — Initial release

## Credits

- **Andy** — Hardware, testing, circuit design
- **AI Assistant** — Firmware development
- **Adafruit** — ADS1X15 library
- **Texas Instruments** — ADS1115 ADC chip

## License

MIT License
