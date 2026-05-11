/*
 * ============================================
 * TankSensor - 4-20mA Level Sensor Library
 * ============================================
 *
 * Reads a 4-20mA submersible pressure/level sensor via an ADS1115
 * external 16-bit ADC (I2C) and converts the signal into level,
 * fill %, and volume for a cylindrical tank.
 *
 * Features:
 *   - ADS1115 16-bit I2C ADC (much higher resolution than onboard ADC)
 *   - Configurable ADS1115 channel (0–3) and I2C address
 *   - Configurable gain (PGA) setting for the ADS1115
 *   - Exponential Moving Average (EMA) smoothing
 *   - Cylindrical tank volume calculation (litres + US gallons)
 *   - EEPROM save / load for all calibration values
 *   - Configurable EEPROM start address (avoid conflicts with other data)
 *
 * Dependency: Adafruit ADS1X15 library
 *   Install via Arduino Library Manager: "Adafruit ADS1X15"
 *
 * ============================================
 * TYPICAL WIRING  (ESP8266 / ESP32 / AVR)
 * ============================================
 *
 *   12-24V PSU (+) ────► Sensor RED wire
 *
 *   Sensor BLACK/GREEN ──┬──► ADS1115 A0 (channel 0)
 *                        ├──► 150Ω resistor ──► GND
 *                        └──► 10µF capacitor ──► GND
 *
 *   12-24V PSU (–) ──────► Common GND (shared with MCU)
 *
 *   ADS1115 ──► MCU
 *     VDD  ──► 3.3V or 5V
 *     GND  ──► GND
 *     SCL  ──► SCL (D1 on Wemos D1 Mini)
 *     SDA  ──► SDA (D2 on Wemos D1 Mini)
 *     ADDR ──► GND  (I2C address 0x48, default)
 *            or VDD  (0x49)
 *            or SDA  (0x4A)
 *            or SCL  (0x4B)
 *
 *   Shunt resistor converts 4–20 mA → 0.6–3.0 V.
 *   Use GAIN_ONE (±4.096 V) for a 150Ω shunt — gives headroom above 3 V.
 *   Adjust VoltageMin / VoltageMax to match your measured values.
 *
 * ============================================
 * QUICK START
 * ============================================
 *
 *   #include <TankSensor.h>
 *
 *   TankSensor tank;              // default: channel 0, addr 0x48, EEPROM 0
 *   // TankSensor tank(1);        // ADS1115 channel 1
 *   // TankSensor tank(0, 0x49);  // channel 0, alternate I2C address
 *
 *   void setup() {
 *     if (!tank.begin()) { Serial.println("ADS1115 not found!"); while(1); }
 *
 *     if (!tank.loadConfig()) {       // restore saved calibration
 *       // First boot — set defaults then save
 *       tank.setAdsGain(GAIN_ONE);    // ±4.096 V (suits 150Ω shunt)
 *       tank.setVoltageMin(0.60);     // 4 mA × 150 Ω
 *       tank.setVoltageMax(3.00);     // 20 mA × 150 Ω
 *       tank.setSensorRange(5.0);     // sensor rated range (m)
 *       tank.setTankDiameter(0.285);  // inner diameter (m)
 *       tank.setTankHeight(0.365);    // usable height (m)
 *       tank.saveConfig();
 *     }
 *   }
 *
 *   void loop() {
 *     TankReading r = tank.read();
 *     if (r.valid) {
 *       Serial.print(r.levelCm);      Serial.println(" cm");
 *       Serial.print(r.volumeLiters); Serial.println(" L");
 *       Serial.print(r.levelPercent); Serial.println(" %");
 *     }
 *     delay(2000);
 *   }
 *
 * ============================================
 * ADS1115 GAIN  (setAdsGain)
 * ============================================
 *
 *   Gain constant      Full-scale  Resolution   Typical use
 *   -----------------  ----------  -----------  --------------------------
 *   GAIN_TWOTHIRDS     +/-6.144 V  0.1875 mV    > 5 V shunt voltages
 *   GAIN_ONE           +/-4.096 V  0.125  mV    150ohm shunt (0.6-3.0 V) <- recommended
 *   GAIN_TWO           +/-2.048 V  0.0625 mV    100ohm shunt (0.4-2.0 V)
 *   GAIN_FOUR          +/-1.024 V  0.03125 mV   50ohm shunt  (0.2-1.0 V)
 *   GAIN_EIGHT         +/-0.512 V  0.015625 mV  Small shunt values
 *   GAIN_SIXTEEN       +/-0.256 V  0.0078125 mV Very small signals
 *
 *   WARNING: Never apply a voltage outside the selected full-scale range
 *   to the ADS1115 input — this can damage the IC.
 *
 * ============================================
 * EEPROM LAYOUT  (27 bytes from eepromAddr)
 * ============================================
 *
 *   [addr+0]  magic byte (0xAC)
 *   [addr+1]  voltMin    (float,   4 bytes)
 *   [addr+5]  voltMax    (float,   4 bytes)
 *   [addr+9]  sensorRange(float,   4 bytes)
 *   [addr+13] tankDiam   (float,   4 bytes)
 *   [addr+17] tankHeight (float,   4 bytes)
 *   [addr+21] emaAlpha   (float,   4 bytes)
 *   [addr+25] adsGain    (uint16_t,2 bytes)
 *   Total: 27 bytes
 *
 */

#ifndef TANK_SENSOR_H
#define TANK_SENSOR_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

// ============================================
// DEFAULTS
// ============================================

#define TANK_DEFAULT_VOLT_MIN      0.60f
#define TANK_DEFAULT_VOLT_MAX      3.00f
#define TANK_DEFAULT_SENSOR_RANGE  5.0f
#define TANK_DEFAULT_DIAMETER      0.56f    // ~oil drum example
#define TANK_DEFAULT_HEIGHT        0.84f    // ~oil drum example
#define TANK_DEFAULT_EMA_ALPHA     0.2f
#define TANK_DEFAULT_ADS_CHANNEL   0        // ADS1115 channel A0
#define TANK_DEFAULT_ADS_ADDRESS   0x48     // ADDR pin → GND
#define TANK_DEFAULT_ADS_GAIN      GAIN_ONE // ±4.096 V

// ============================================
// TankReading
// ============================================

struct TankReading {
  int16_t rawADC;          // raw ADS1115 ADC value (16-bit signed)
  float   voltage;         // EMA-smoothed voltage (V)
  float   levelMeters;     // liquid level (m)
  float   levelCm;         // liquid level (cm)
  float   levelPercent;    // fill percentage (0–100)
  float   volumeLiters;    // volume (L)
  float   volumeGallons;   // volume (US gal)
  bool    valid;           // false if begin() not called or ADS1115 not found
};

// ============================================
// TankSensor
// ============================================

class TankSensor {
public:
  // channel    — ADS1115 input channel (0–3)   (default 0 = A0)
  // i2cAddr    — ADS1115 I2C address            (default 0x48)
  // eepromAddr — EEPROM start byte              (default 0)
  explicit TankSensor(uint8_t channel = TANK_DEFAULT_ADS_CHANNEL,
                      uint8_t i2cAddr  = TANK_DEFAULT_ADS_ADDRESS,
                      uint8_t eepromAddr = 0);

  // Call once in setup() — initialises I2C and ADS1115.
  // Returns false if the ADS1115 is not detected on the bus.
  bool begin();

  // Take a reading
  TankReading read();

  // Force EMA to re-seed on next read (call after changing channel or gain)
  void resetSmoothing();

  // ----------------------------------------
  // Sensor calibration
  // ----------------------------------------
  void setVoltageMin(float v);       // voltage at empty (4 mA × shunt Ω)
  void setVoltageMax(float v);       // voltage at full  (20 mA × shunt Ω)
  void setSensorRange(float meters); // sensor rated max range (m)
  void setEmaAlpha(float alpha);     // 0.05 = heavy smoothing, 1.0 = raw

  // ----------------------------------------
  // Tank dimensions
  // ----------------------------------------
  void setTankDiameter(float meters); // cylindrical tank inner diameter (m)
  void setTankHeight(float meters);   // usable fill height (m)

  // ----------------------------------------
  // ADS1115 configuration
  // ----------------------------------------
  void setAdsChannel(uint8_t channel); // change channel at runtime (resets smoothing)
  void setAdsGain(adsGain_t gain);     // PGA gain setting (resets smoothing)

  // ----------------------------------------
  // All-in-one calibration helper
  // ----------------------------------------
  void configure(float voltMin, float voltMax, float sensorRange,
                 float tankDiam, float tankHeight, float emaAlpha);

  // ----------------------------------------
  // EEPROM persistence
  // ----------------------------------------
  void setEepromAddress(uint8_t addr); // change start address before load/save
  bool loadConfig();                   // returns true if valid data was found
  void saveConfig();

  // ----------------------------------------
  // Getters
  // ----------------------------------------
  float     getVoltageMin()    const;
  float     getVoltageMax()    const;
  float     getSensorRange()   const;
  float     getTankDiameter()  const;
  float     getTankHeight()    const;
  float     getEmaAlpha()      const;
  uint8_t   getAdsChannel()    const;
  adsGain_t getAdsGain()       const;
  uint8_t   getEepromAddress() const;

private:
  Adafruit_ADS1115 _ads;
  uint8_t   _channel;
  uint8_t   _i2cAddr;
  adsGain_t _gain;

  float   _voltageMin;
  float   _voltageMax;
  float   _sensorRange;
  float   _tankDiameter;
  float   _tankHeight;
  float   _emaAlpha;

  float   _tankRadius;
  float   _tankBaseArea;
  float   _smoothedVoltage;
  bool    _initialized;

  uint8_t _eepromAddr;

  void  _updateGeometry();
  float _mapFloat(float x, float inMin, float inMax, float outMin, float outMax) const;
};

#endif // TANK_SENSOR_H
