/*
 * ============================================
 * SubmersibleSensor - Hydrostatic Pressure (4-20mA) Sensor Library
 * ============================================
 *
 * Reads a 4-20mA submersible pressure/level sensor via an ADS1115
 * external 16-bit ADC (I2C) and converts the signal into level,
 * fill %, and volume.
 *
 * Two tank geometries are supported:
 *
 *   TANK_VERTICAL (default)
 *     Any tank with a constant cross-section (upright cylinder,
 *     rectangular, custom shape).
 *     Volume = baseSurface (m2) * level (m)
 *     Required params: baseSurface, tankHeight
 *
 *   TANK_HORIZONTAL_CYLINDER
 *     A cylinder lying on its side.
 *     Cross-section is a circular segment that varies with level.
 *     Volume = circular_segment_area(level) * tankLength
 *     Required params: tankHeight (= internal diameter), tankLength
 *     (baseSurface is not used in this mode)
 *
 * Features:
 *   - ADS1115 16-bit I2C ADC
 *   - Configurable ADS1115 channel (0-3) and I2C address
 *   - Configurable PGA gain setting
 *   - Exponential Moving Average (EMA) smoothing
 *   - Cylindrical tank volume calculation (litres + US gallons)
 *   - EEPROM save / load for all calibration values
 *   - Configurable EEPROM start address
 *
 * Dependency: Adafruit ADS1X15 library
 *   Install via Arduino Library Manager: "Adafruit ADS1X15"
 *
 * ============================================
 * TYPICAL WIRING
 * ============================================
 *
 *   12-32V PSU (+) -----> Sensor RED wire
 *
 *   Sensor BLACK/GREEN --+----> ADS1115 A0 (channel 0)
 *                        +----> 150ohm resistor ----> GND
 *                        +----> 10uF capacitor  ----> GND
 *
 *   12-32V PSU (-) -----> Common GND (shared with MCU)
 *
 *   ADS1115 ---> MCU
 *     VDD   ---> 3.3V or 5V
 *     GND   ---> GND
 *     SCL   ---> SCL  (D1 on Wemos D1 Mini)
 *     SDA   ---> SDA  (D2 on Wemos D1 Mini)
 *     ADDR  ---> GND  (I2C address 0x48, default)
 *            or  VDD  (0x49)
 *            or  SDA  (0x4A)
 *            or  SCL  (0x4B)
 *
 * ============================================
 * QUICK START
 * ============================================
 *
 *   #include <SubSensor.h>
 *
 *   SubSensor sensor;   // default: channel 0, addr 0x48, EEPROM 0
 *
 *   void setup() {
 *     if (!sensor.begin()) { while(1); }  // ADS1115 not found
 *
 *     if (!sensor.loadConfig()) {
 *       sensor.setAdsGain(GAIN_ONE);       // +/-4.096V suits 150ohm shunt
 *       sensor.setVoltageMin(0.60);        // 4 mA x 150 ohm
 *       sensor.setVoltageMax(3.00);        // 20 mA x 150 ohm
 *       sensor.setSensorRange(5.0);        // sensor rated range (m)
 *
 *       // --- Vertical tank (constant cross-section) ---
 *       sensor.setTankType(TANK_VERTICAL);
 *       sensor.setBaseSurface(0.0638);     // e.g. pi * (0.285/2)^2
 *       sensor.setTankHeight(0.365);
 *
 *       // --- OR: Horizontal cylinder ---
 *       // sensor.setTankType(TANK_HORIZONTAL_CYLINDER);
 *       // sensor.setTankHeight(0.60);     // internal diameter (m)
 *       // sensor.setTankLength(1.20);     // axial length (m)
 *
 *       sensor.saveConfig();
 *     }
 *   }
 *
 *   void loop() {
 *     TankReading r = sensor.read();
 *     if (r.valid) {
 *       Serial.print(r.levelCm);      Serial.println(" cm");
 *       Serial.print(r.volumeLiters); Serial.println(" L");
 *       Serial.print(r.fillPercent);  Serial.println(" %");
 *     }
 *     delay(2000);
 *   }
 *
 * ============================================
 * ADS1115 GAIN  (setAdsGain)
 * ============================================
 *
 *   Gain constant      Full-scale   Typical use
 *   -----------------  -----------  --------------------------------
 *   GAIN_TWOTHIRDS     +/-6.144 V   > 5 V shunt voltages
 *   GAIN_ONE           +/-4.096 V   150ohm shunt (0.6-3.0 V)  <- recommended
 *   GAIN_TWO           +/-2.048 V   100ohm shunt (0.4-2.0 V)
 *   GAIN_FOUR          +/-1.024 V   50ohm shunt  (0.2-1.0 V)
 *   GAIN_EIGHT         +/-0.512 V   Small shunt values
 *   GAIN_SIXTEEN       +/-0.256 V   Very small signals
 *
 *   WARNING: Never apply a voltage outside the selected range to ADS1115.
 *
 * ============================================
 * EEPROM LAYOUT  (32 bytes from eepromAddr)
 * ============================================
 *
 *   [addr+ 0] magic byte  (0xAD)        1 byte
 *   [addr+ 1] voltMin     (float)        4 bytes
 *   [addr+ 5] voltMax     (float)        4 bytes
 *   [addr+ 9] sensorRange (float)        4 bytes
 *   [addr+13] baseSurface (float)        4 bytes
 *   [addr+17] tankHeight  (float)        4 bytes
 *   [addr+21] tankLength  (float)        4 bytes
 *   [addr+25] emaAlpha    (float)        4 bytes
 *   [addr+29] adsGain     (uint16_t)     2 bytes
 *   [addr+31] tankType    (uint8_t)      1 byte
 *   Total: 32 bytes
 *
 */

#ifndef SUB_SENSOR_H
#define SUB_SENSOR_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

enum TankType : uint8_t {
  TANK_VERTICAL            = 0,
  TANK_HORIZONTAL_CYLINDER = 1
};

#define TANK_DEFAULT_VOLT_MIN      0.60f
#define TANK_DEFAULT_VOLT_MAX      3.00f
#define TANK_DEFAULT_SENSOR_RANGE  5.0f
#define TANK_DEFAULT_BASE_SURFACE  0.2463f
#define TANK_DEFAULT_HEIGHT        0.84f
#define TANK_DEFAULT_LENGTH        1.0f
#define TANK_DEFAULT_EMA_ALPHA     0.2f
#define TANK_DEFAULT_ADS_CHANNEL   0
#define TANK_DEFAULT_ADS_ADDRESS   0x48
#define TANK_DEFAULT_ADS_GAIN      GAIN_ONE
#define TANK_DEFAULT_TANK_TYPE     TANK_VERTICAL

struct TankReading {
  int16_t rawADC;
  float   voltage;
  float   levelMeters;
  float   levelCm;
  float   fillPercent;
  float   volumeLiters;
  float   volumeGallons;
  bool    valid;
};

class SubSensor {
public:
  explicit SubSensor(uint8_t channel    = TANK_DEFAULT_ADS_CHANNEL,
                     uint8_t i2cAddr    = TANK_DEFAULT_ADS_ADDRESS,
                     uint8_t eepromAddr = 0);

  bool begin();
  TankReading read();
  void resetSmoothing();

  void setVoltageMin(float v);
  void setVoltageMax(float v);
  void setSensorRange(float meters);
  void setEmaAlpha(float alpha);

  void setTankType(TankType type);
  void setBaseSurface(float m2);
  void setTankHeight(float meters);
  void setTankLength(float meters);

  void setAdsChannel(uint8_t channel);
  void setAdsGain(adsGain_t gain);

  void setSamplesPerRead(uint8_t n);
  uint8_t getSamplesPerRead() const;

  void configure(float voltMin, float voltMax, float sensorRange,
                 float baseSurface, float tankHeight, float emaAlpha);

  void setEepromAddress(uint8_t addr);
  bool loadConfig();
  void saveConfig();

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

private:
  Adafruit_ADS1115 _ads;
  uint8_t   _channel;
  uint8_t   _i2cAddr;
  adsGain_t _gain;

  float    _voltageMin;
  float    _voltageMax;
  float    _sensorRange;
  float    _baseSurface;
  float    _tankHeight;
  float    _tankLength;
  float    _emaAlpha;
  TankType _tankType;

  float _smoothedVoltage;
  bool  _initialized;

  uint8_t _eepromAddr;
  uint8_t _samplesPerRead;

  float _volumeFromLevel(float levelM) const;
  float _mapFloat(float x, float inMin, float inMax,
                  float outMin, float outMax) const;
};

#endif // SUB_SENSOR_H