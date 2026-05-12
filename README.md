# TankSensor Library

A robust Arduino library for reading 4-20mA submersible liquid level sensors via an **ADS1115 16-bit I2C ADC**, with EMA smoothing, flexible tank geometry support, and persistent EEPROM calibration.

## Features

- **16-bit ADS1115 ADC** — Much higher resolution than onboard Arduino/ESP ADCs
- **Two tank geometry modes:**
  - **Vertical** — Any constant cross-section (cylinder, rectangle, custom shape). Volume = baseSurface × height
  - **Horizontal cylinder** — Calculates circular segment area at each level. Handles the complex non-linear volume curve
- **Exponential Moving Average (EMA) smoothing** — Configurable smoothing factor for noisy sensor signals
- **EEPROM persistence** — Save and restore all calibration parameters across power cycles
- **Configurable ADS1115 settings** — Channel selection (0–3), I2C address, and PGA gain (±6.144 V to ±0.256 V)
- **Works on ESP8266, ESP32, and AVR (Arduino Uno/Nano)**

## Hardware Requirements

### Electronics
- **ADS1115 module** — 16-bit I2C ADC breakout (widely available, ~$3–5 USD)
- **4-20mA submersible level sensor** — Any industrial sensor in this current range (e.g., 0–5 m, 0–10 m range)
- **150Ω shunt resistor** — Converts 4–20 mA to 0.6–3.0 V (standard for 150Ω)
  - Can use 100Ω (0.4–2.0 V), 50Ω (0.2–1.0 V), or other values — adjust `setAdsGain()` and calibration accordingly
- **10µF capacitor** — Noise filtering across the shunt resistor
- **12–24 V DC power supply** — For the sensor (specs depend on your specific sensor)
- **ESP8266, ESP32, or Arduino Uno/Nano** — For the microcontroller

### Libraries
- **Adafruit ADS1X15** — Install via Arduino Library Manager or manually

## Installation

### 1. Install the Adafruit ADS1X15 Library

In the Arduino IDE:
- **Sketch → Include Library → Manage Libraries**
- Search: `Adafruit ADS1X15`
- Click **Install**

Or via command line (using Arduino CLI):
```bash
arduino-cli lib install "Adafruit ADS1X15"
```

### 2. Install TankSensor Library

Copy the `TankSensor` folder to your Arduino `libraries` directory:
- **macOS / Linux:** `~/Arduino/libraries/`
- **Windows:** `Documents\Arduino\libraries\`

Or use Arduino IDE: **Sketch → Include Library → Add .ZIP Library** and select the folder.

## Wiring

### Pin Layout by Board

The ADS1115 communicates via **I2C (TWI)** — all boards use the same I2C protocol, but pins differ.

#### ESP8266 (Wemos D1 Mini)
| ADS1115 Pin | Wemos Pin | Notes |
|-------------|-----------|-------|
| VDD         | 3.3V      | Power  |
| GND         | GND       | Ground |
| SCL         | D1        | I2C clock |
| SDA         | D2        | I2C data  |
| ADDR        | GND       | I2C address = 0x48 (default) |
| A0          | (analog)  | Sensor signal (see diagram) |

#### ESP32
| ADS1115 Pin | ESP32 Pin | Notes |
|-------------|-----------|-------|
| VDD         | 3.3V      | Power  |
| GND         | GND       | Ground |
| SCL         | GPIO22    | I2C clock (can change via `Wire.begin(SDA, SCL)`) |
| SDA         | GPIO21    | I2C data  |
| ADDR        | GND       | I2C address = 0x48 (default) |

#### Arduino Uno / Nano
| ADS1115 Pin | Arduino Pin | Notes |
|-------------|-------------|-------|
| VDD         | 5V          | Power  |
| GND         | GND         | Ground |
| SCL         | A5          | I2C clock |
| SDA         | A4          | I2C data  |
| ADDR        | GND         | I2C address = 0x48 (default) |

### Sensor Signal Chain

```
12-24V PSU (+)  ------>  Sensor RED wire
                         (internal 4-20mA loop)
Sensor BLACK/GREEN  --+-->  150Ω shunt resistor  --+-->  GND
                      |     (converts 4-20mA to       |
                      |      0.6-3.0V)                |
                      |     10µF capacitor  (noise)   |
                      |                               |
                      +------------>  ADS1115 A0 pin  |
                                                      |
12-24V PSU (-)  ------>  Common GND  <---------------+
```

**Key points:**
- The 4-20mA sensor loop is **independent** from microcontroller power
- The shunt resistor and capacitor sit **between the sensor output and the ADS1115 input**
- All grounds (PSU, MCU, sensor) must be **common**
- The **ADDR pin** determines the I2C address:
  - GND → `0x48` (default)
  - VDD → `0x49`
  - SDA → `0x4A`
  - SCL → `0x4B`

## Quick Start

### Minimal Vertical Tank Example

```cpp
#include <TankSensor.h>

// Create a sensor on ADS1115 channel 0, default I2C address 0x48
TankSensor tank;

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize the ADS1115
  if (!tank.begin()) {
    Serial.println("ERROR: ADS1115 not found!");
    while (1) delay(100);
  }

  // Try to load saved calibration from EEPROM
  if (!tank.loadConfig()) {
    // First boot — configure the sensor
    tank.setAdsGain(GAIN_ONE);      // ±4.096V for 150Ω shunt
    tank.setVoltageMin(0.60);       // 4 mA reading
    tank.setVoltageMax(3.00);       // 20 mA reading
    tank.setSensorRange(5.0);       // sensor rated to 5 meters

    // Vertical tank: cylinder with 28.5 cm diameter, 36.5 cm height
    tank.setTankType(TANK_VERTICAL);
    tank.setBaseSurface(0.0638);    // π * (0.285/2)² m²
    tank.setTankHeight(0.365);

    tank.setEmaAlpha(0.2);          // EMA smoothing
    tank.saveConfig();              // Save to EEPROM
    Serial.println("Calibration saved.");
  }
}

void loop() {
  TankReading r = tank.read();
  
  if (r.valid) {
    Serial.print("Level: ");    Serial.print(r.levelCm);       Serial.println(" cm");
    Serial.print("Fill: ");     Serial.print(r.fillPercent);   Serial.println(" %");
    Serial.print("Volume: ");   Serial.print(r.volumeLiters);  Serial.println(" L");
  }
  
  delay(2000);
}
```

### Horizontal Cylinder Tank Example

```cpp
#include <TankSensor.h>

TankSensor tank;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!tank.begin()) {
    Serial.println("ERROR: ADS1115 not found!");
    while (1) delay(100);
  }

  if (!tank.loadConfig()) {
    // Configure for a horizontal cylindrical tank
    tank.setAdsGain(GAIN_ONE);
    tank.setVoltageMin(0.60);
    tank.setVoltageMax(3.00);
    tank.setSensorRange(5.0);

    // Horizontal cylinder: internal diameter 60 cm, length 120 cm
    tank.setTankType(TANK_HORIZONTAL_CYLINDER);
    tank.setTankHeight(0.60);    // internal diameter
    tank.setTankLength(1.20);    // axial length

    tank.setEmaAlpha(0.2);
    tank.saveConfig();
  }
}

void loop() {
  TankReading r = tank.read();
  if (r.valid) {
    Serial.print("Level: ");  Serial.print(r.levelCm);      Serial.println(" cm");
    Serial.print("Volume: "); Serial.print(r.volumeLiters); Serial.println(" L");
  }
  delay(2000);
}
```

## Configuration Guide

### Calibration: Voltage Min/Max

The sensor outputs a **4–20 mA current** proportional to the liquid level. The shunt resistor converts this to voltage:

- At **empty (0 m)**: 4 mA × 150Ω = **0.60 V**
- At **full sensor range**: 20 mA × 150Ω = **3.00 V**

If using a different shunt resistor, adjust accordingly:
- **100Ω shunt**: 0.40–2.00 V
- **50Ω shunt**: 0.20–1.00 V

**To calibrate:**
1. Place the sensor at a **known empty level** (e.g., on a bench, not in liquid)
2. Measure the voltage with a multimeter or read `Serial.print(r.voltage)` in raw mode
3. Call `tank.setVoltageMin(measured_voltage)`
4. Repeat at **full level** with `setVoltageMax()`
5. Save with `tank.saveConfig()`

### ADS Gain Selection (PGA)

The **Programmable Gain Amplifier** determines the full-scale voltage range and resolution:

| Gain | Full-scale | Resolution | Use Case |
|------|-----------|------------|----------|
| `GAIN_TWOTHIRDS` | ±6.144 V | 0.1875 mV/bit | Large shunt voltages (> 5 V) |
| `GAIN_ONE` | ±4.096 V | 0.125 mV/bit | **150Ω shunt (0.6–3.0 V)** ← default |
| `GAIN_TWO` | ±2.048 V | 0.0625 mV/bit | 100Ω shunt (0.4–2.0 V) |
| `GAIN_FOUR` | ±1.024 V | 0.03125 mV/bit | 50Ω shunt (0.2–1.0 V) |
| `GAIN_EIGHT` | ±0.512 V | 0.015625 mV/bit | Very small signals |
| `GAIN_SIXTEEN` | ±0.256 V | 0.0078125 mV/bit | Extreme precision |

**Choose a gain where your expected voltage range (0.6–3.0 V for a 150Ω shunt) occupies the middle 50–80% of the full scale.** This maximizes precision while avoiding clipping.

**⚠️ WARNING:** Never apply a voltage outside the selected full-scale range to the ADS1115 — this can damage the IC.

### EMA Smoothing

The Exponential Moving Average reduces jitter:

```cpp
tank.setEmaAlpha(0.05);   // Heavy smoothing, slow to respond
tank.setEmaAlpha(0.2);    // Moderate smoothing (default)
tank.setEmaAlpha(1.0);    // No smoothing, raw readings
```

Lower alpha = more smoothing (slower response). Adjust based on sensor noise and application latency requirements.

### Tank Geometry

#### Vertical Tanks

For any constant cross-section (upright cylinder, rectangle, custom shape):

```cpp
tank.setTankType(TANK_VERTICAL);
tank.setBaseSurface(area_in_m2);   // Cross-sectional area
tank.setTankHeight(height_in_m);   // Usable fill height
```

**Example: 28.5 cm diameter cylinder**
```cpp
float radius = 0.285 / 2.0;
float area = 3.14159 * radius * radius;  // ≈ 0.0638 m²
tank.setBaseSurface(area);
```

**Example: Rectangular tank 1.5 m × 0.5 m**
```cpp
tank.setBaseSurface(1.5 * 0.5);  // 0.75 m²
```

#### Horizontal Cylinders

For a cylinder lying on its side:

```cpp
tank.setTankType(TANK_HORIZONTAL_CYLINDER);
tank.setTankHeight(internal_diameter_m);  // Also the max fill level
tank.setTankLength(axial_length_m);       // Length of the cylinder
```

The library calculates the circular segment area at each level using:

```
A(h) = r² × acos((r - h)/r) - (r - h) × √(2rh - h²)
V(h) = A(h) × length
```

where `r = diameter / 2` and `h` is the liquid level from the bottom.

## API Reference

### Core Methods

```cpp
// Initialize the ADS1115 on the I2C bus
bool begin();                              // Returns false if chip not found

// Take a reading
TankReading read();                        // Returns a TankReading struct

// Reset the EMA smoothing filter (call after changing channel or gain)
void resetSmoothing();
```

### Sensor Calibration

```cpp
void setVoltageMin(float v);              // Voltage at empty (V)
void setVoltageMax(float v);              // Voltage at full (V)
void setSensorRange(float meters);        // Sensor rated max range (m)
void setEmaAlpha(float alpha);            // EMA smoothing factor (0.01–1.0)
```

### Tank Geometry

```cpp
void setTankType(TankType type);          // TANK_VERTICAL or TANK_HORIZONTAL_CYLINDER
void setBaseSurface(float m2);            // Cross-sectional area (vertical only) (m²)
void setTankHeight(float meters);         // Fill height (vertical) or diameter (horizontal)
void setTankLength(float meters);         // Axial length (horizontal cylinder only)
```

### ADS1115 Configuration

```cpp
void setAdsChannel(uint8_t ch);           // Change input channel (0–3)
void setAdsGain(adsGain_t gain);          // Set PGA gain (GAIN_ONE, etc.)
```

### EEPROM

```cpp
void setEepromAddress(uint8_t addr);      // Set EEPROM start byte (before load/save)
bool loadConfig();                        // Restore calibration from EEPROM
void saveConfig();                        // Persist calibration to EEPROM
```

### Quick Configuration

```cpp
// All-in-one helper for vertical tanks
void configure(float voltMin, float voltMax, float sensorRange,
               float baseSurface, float tankHeight, float emaAlpha);
```

### Getters

```cpp
float     getVoltageMin()    const;
float     getVoltageMax()    const;
float     getSensorRange()   const;
TankType  getTankType()      const;
float     getBaseSurface()   const;
float     getTankHeight()    const;
float     getTankLength()    const;
float     getEmaAlpha()      const;
uint8_t   getAdsChannel()    const;
adsGain_t getAdsGain()       const;
uint8_t   getEepromAddress() const;
```

### TankReading Struct

```cpp
struct TankReading {
  int16_t rawADC;       // Raw 16-bit ADC value from ADS1115
  float   voltage;      // EMA-smoothed voltage (V)
  float   levelMeters;  // Liquid level (m)
  float   levelCm;      // Liquid level (cm)
  float   fillPercent;  // Fill % by height (0–100)
  float   volumeLiters; // Volume (litres)
  float   volumeGallons;// Volume (US gallons)
  bool    valid;        // false if begin() not called or reading failed
};
```

## Troubleshooting

### ADS1115 Not Detected

**Symptom:** `begin()` returns `false`; "ADS1115 not found" message

**Checklist:**
1. **I2C wiring:** Verify SCL and SDA are connected to the correct pins for your board (see [Wiring](#wiring) section)
2. **Power:** Ensure ADS1115 VDD is connected (3.3V or 5V, depending on board)
3. **ADDR pin:** Confirm it's tied to GND (or whichever voltage sets the I2C address you expect — default 0x48)
4. **Pull-ups:** I2C requires 4.7kΩ pull-up resistors on SCL and SDA to VDD. Many ADS1115 modules include these; if not, add them
5. **I2C address conflict:** Try setting a different ADDR pin state (tie to VDD for 0x49, SDA for 0x4A, SCL for 0x4B)

### Voltage Out of Range (0.6–3.0 V)

**Symptom:** Readings are clipped, stuck at min/max, or wildly inaccurate

**Checklist:**
1. **Shunt value:** Verify you're using the correct shunt resistor (150Ω standard). Measure the actual voltage with a multimeter
2. **Gain setting:** Ensure the selected gain accommodates your voltage range. Raise the gain (lower ±V) if voltage is too high; lower the gain if too low
3. **Sensor wiring:** Confirm the 4-20mA loop is complete — sensor positive (red) to PSU+, sensor negative (black/green) through shunt to GND
4. **Capacitor:** The 10µF capacitor should be across the shunt resistor (not in series)

### Noisy Readings

**Symptom:** `r.voltage` jumps around; `r.levelCm` fluctuates wildly

**Fixes:**
1. **Increase EMA smoothing:** Lower `setEmaAlpha()` (e.g., 0.05 instead of 0.2)
2. **Add RC filter:** Place a 1kΩ resistor + 100nF capacitor on the ADS1115 A0 input (low-pass filter)
3. **Cable shielding:** Use shielded wire between the shunt and ADS1115 A0 pin; ground the shield at the ADS1115 end only
4. **Check capacitor:** Verify the 10µF capacitor is in good condition (multimeter ESR test)

### EEPROM Not Persisting

**Symptom:** Calibration settings reset after power cycle

**Fixes:**
1. **Call `saveConfig()`:** Ensure you explicitly call `tank.saveConfig()` after setting parameters (not automatic)
2. **EEPROM address conflict:** If using shared EEPROM space, change the address: `tank.setEepromAddress(32)` before `loadConfig()` / `saveConfig()`
3. **EEPROM limits:** AVR boards have ~1 KB EEPROM; ESP8266/ESP32 have more but fragmentation can occur — use non-overlapping addresses

## Sensor Selection Tips

- **Submersible pressure sensors:** Most common for tanks. Typically 4-20mA output, 0–5 m, 0–10 m, or custom ranges
- **Common manufacturers:** Seafloor Systems, Blue Robotics, Keller, Druck, Wika
- **Accuracy:** Look for ±0.5% or better (full scale)
- **Cable:** Shielded 2-wire or 4-wire (2-wire is typical for current-loop sensors)
- **Cost:** Budget $50–200 USD for a quality industrial sensor

## Examples

See `examples/BasicReading/BasicReading.ino` for a complete working example that demonstrates both vertical and horizontal cylinder tank configurations.

## License

This library is provided as-is for hobbyist and educational use.

## Support

For issues or questions:
1. Check the [Troubleshooting](#troubleshooting) section
2. Verify the [Wiring](#wiring) diagram for your specific board
3. Test with a multimeter to isolate hardware vs. software issues
4. Review the Adafruit ADS1X15 library documentation for I2C-specific issues
