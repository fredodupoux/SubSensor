#include <EEPROM.h>
#include "TankSensor.h"

// ============================================
// EEPROM layout  (internal — not in header)
// ============================================

namespace {
  const uint8_t CONFIG_MAGIC = 0xAC; // 0xAC: ADS1115-based layout

  struct ConfigData {
    uint8_t  magic;
    float    voltMin;
    float    voltMax;
    float    sensorRange;
    float    tankDiameter;
    float    tankHeight;
    float    emaAlpha;
    uint16_t adsGain;
    // total: 27 bytes
  };

  inline void eepromBegin(size_t size) {
#if defined(ESP8266) || defined(ESP32)
    EEPROM.begin(size);
#else
    (void)size;
#endif
  }
  inline void eepromCommitEnd() {
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
    EEPROM.end();
#endif
  }
}

// ============================================
// Constructor
// ============================================

TankSensor::TankSensor(uint8_t channel, uint8_t i2cAddr, uint8_t eepromAddr)
  : _channel(channel),
    _i2cAddr(i2cAddr),
    _gain(TANK_DEFAULT_ADS_GAIN),
    _voltageMin(TANK_DEFAULT_VOLT_MIN),
    _voltageMax(TANK_DEFAULT_VOLT_MAX),
    _sensorRange(TANK_DEFAULT_SENSOR_RANGE),
    _tankDiameter(TANK_DEFAULT_DIAMETER),
    _tankHeight(TANK_DEFAULT_HEIGHT),
    _emaAlpha(TANK_DEFAULT_EMA_ALPHA),
    _smoothedVoltage(-1.0f),
    _initialized(false),
    _eepromAddr(eepromAddr)
{
  _updateGeometry();
}

// ============================================
// begin
// ============================================

bool TankSensor::begin() {
  if (!_ads.begin(_i2cAddr)) return false;
  _ads.setGain(_gain);
  _initialized = true;
  return true;
}

// ============================================
// read
// ============================================

TankReading TankSensor::read() {
  TankReading r = {0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
  if (!_initialized) return r;

  r.rawADC = _ads.readADC_SingleEnded(_channel);
  float rawVolt = _ads.computeVolts(r.rawADC);

  if (_smoothedVoltage < 0.0f) {
    _smoothedVoltage = rawVolt;
  } else {
    _smoothedVoltage = (_emaAlpha * rawVolt) + ((1.0f - _emaAlpha) * _smoothedVoltage);
  }
  r.voltage = _smoothedVoltage;

  r.levelMeters = _mapFloat(r.voltage, _voltageMin, _voltageMax, 0.0f, _sensorRange);
  r.levelMeters = constrain(r.levelMeters, 0.0f, _tankHeight);

  r.levelCm      = r.levelMeters * 100.0f;
  r.levelPercent = (_tankHeight > 0.0f) ? (r.levelMeters / _tankHeight) * 100.0f : 0.0f;
  r.volumeLiters  = _tankBaseArea * r.levelMeters * 1000.0f;
  r.volumeGallons = r.volumeLiters * 0.264172f;
  r.valid = true;
  return r;
}

// ============================================
// resetSmoothing
// ============================================

void TankSensor::resetSmoothing() {
  _smoothedVoltage = -1.0f;
}

// ============================================
// Sensor calibration setters
// ============================================

void TankSensor::setVoltageMin(float v)    { _voltageMin  = v; }
void TankSensor::setVoltageMax(float v)    { _voltageMax  = v; }
void TankSensor::setSensorRange(float m)   { _sensorRange = m; }
void TankSensor::setEmaAlpha(float a)      { _emaAlpha    = constrain(a, 0.01f, 1.0f); }

// ============================================
// Tank dimension setters
// ============================================

void TankSensor::setTankDiameter(float m) { _tankDiameter = m; _updateGeometry(); }
void TankSensor::setTankHeight(float m)   { _tankHeight   = m; }

// ============================================
// ADS1115 setters
// ============================================

void TankSensor::setAdsChannel(uint8_t channel) {
  _channel = channel;
  resetSmoothing();
}

void TankSensor::setAdsGain(adsGain_t gain) {
  _gain = gain;
  if (_initialized) _ads.setGain(_gain);
  resetSmoothing();
}

// ============================================
// All-in-one configure
// ============================================

void TankSensor::configure(float voltMin, float voltMax, float sensorRange,
                            float tankDiam, float tankHeight, float emaAlpha) {
  _voltageMin   = voltMin;
  _voltageMax   = voltMax;
  _sensorRange  = sensorRange;
  _tankDiameter = tankDiam;
  _tankHeight   = tankHeight;
  _emaAlpha     = constrain(emaAlpha, 0.01f, 1.0f);
  _updateGeometry();
}

// ============================================
// EEPROM address setter
// ============================================

void TankSensor::setEepromAddress(uint8_t addr) { _eepromAddr = addr; }

// ============================================
// EEPROM load
// ============================================

bool TankSensor::loadConfig() {
  size_t sz = (size_t)_eepromAddr + sizeof(ConfigData);
  eepromBegin(sz);

  ConfigData cfg;
  EEPROM.get(_eepromAddr, cfg);

#if defined(ESP8266) || defined(ESP32)
  EEPROM.end();
#endif

  if (cfg.magic != CONFIG_MAGIC) return false;

  _voltageMin   = cfg.voltMin;
  _voltageMax   = cfg.voltMax;
  _sensorRange  = cfg.sensorRange;
  _tankDiameter = cfg.tankDiameter;
  _tankHeight   = cfg.tankHeight;
  _emaAlpha     = cfg.emaAlpha;
  _gain         = (adsGain_t)cfg.adsGain;
  if (_initialized) _ads.setGain(_gain);
  _updateGeometry();
  return true;
}

// ============================================
// EEPROM save
// ============================================

void TankSensor::saveConfig() {
  ConfigData cfg;
  cfg.magic        = CONFIG_MAGIC;
  cfg.voltMin      = _voltageMin;
  cfg.voltMax      = _voltageMax;
  cfg.sensorRange  = _sensorRange;
  cfg.tankDiameter = _tankDiameter;
  cfg.tankHeight   = _tankHeight;
  cfg.emaAlpha     = _emaAlpha;
  cfg.adsGain      = (uint16_t)_gain;

  size_t sz = (size_t)_eepromAddr + sizeof(ConfigData);
  eepromBegin(sz);
  EEPROM.put(_eepromAddr, cfg);
  eepromCommitEnd();
}

// ============================================
// Getters
// ============================================

float     TankSensor::getVoltageMin()    const { return _voltageMin;   }
float     TankSensor::getVoltageMax()    const { return _voltageMax;   }
float     TankSensor::getSensorRange()   const { return _sensorRange;  }
float     TankSensor::getTankDiameter()  const { return _tankDiameter; }
float     TankSensor::getTankHeight()    const { return _tankHeight;   }
float     TankSensor::getEmaAlpha()      const { return _emaAlpha;     }
uint8_t   TankSensor::getAdsChannel()    const { return _channel;      }
adsGain_t TankSensor::getAdsGain()       const { return _gain;         }
uint8_t   TankSensor::getEepromAddress() const { return _eepromAddr;   }

// ============================================
// Private helpers
// ============================================

void TankSensor::_updateGeometry() {
  _tankRadius   = _tankDiameter / 2.0f;
  _tankBaseArea = 3.14159265f * _tankRadius * _tankRadius;
}

float TankSensor::_mapFloat(float x, float inMin, float inMax,
                             float outMin, float outMax) const {
  if (inMax == inMin) return outMin;
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
