#include <math.h>
#include <EEPROM.h>
#include "TankSensor.h"

// ============================================
// EEPROM layout  (internal)
// ============================================

namespace {
  const uint8_t CONFIG_MAGIC = 0xAD;

  struct ConfigData {
    uint8_t  magic;
    float    voltMin;
    float    voltMax;
    float    sensorRange;
    float    baseSurface;
    float    tankHeight;
    float    tankLength;
    float    emaAlpha;
    uint16_t adsGain;
    uint8_t  tankType;
    // total: 32 bytes
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
    _baseSurface(TANK_DEFAULT_BASE_SURFACE),
    _tankHeight(TANK_DEFAULT_HEIGHT),
    _tankLength(TANK_DEFAULT_LENGTH),
    _emaAlpha(TANK_DEFAULT_EMA_ALPHA),
    _tankType(TANK_DEFAULT_TANK_TYPE),
    _smoothedVoltage(-1.0f),
    _initialized(false),
    _eepromAddr(eepromAddr),
    _samplesPerRead(11)
{}

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

  float smoothedVolt = -1.0f;
  for (uint8_t i = 0; i < _samplesPerRead; i++) {
    r.rawADC = _ads.readADC_SingleEnded(_channel);
    float rawVolt = _ads.computeVolts(r.rawADC);

    if (smoothedVolt < 0.0f) {
      smoothedVolt = rawVolt;
    } else {
      smoothedVolt = (_emaAlpha * rawVolt) + ((1.0f - _emaAlpha) * smoothedVolt);
    }
  }
  r.voltage = smoothedVolt;

  r.levelMeters = _mapFloat(r.voltage, _voltageMin, _voltageMax, 0.0f, _sensorRange);
  r.levelMeters = constrain(r.levelMeters, 0.0f, _tankHeight);

  r.levelCm      = r.levelMeters * 100.0f;
  r.fillPercent  = (_tankHeight > 0.0f) ? (r.levelMeters / _tankHeight) * 100.0f : 0.0f;
  r.volumeLiters  = _volumeFromLevel(r.levelMeters);
  r.volumeGallons = r.volumeLiters * 0.264172f;
  r.valid = true;
  return r;
}

// ============================================
// resetSmoothing
// ============================================

void TankSensor::resetSmoothing() {
  // No longer needed with internal multi-sampling approach,
  // but kept for API compatibility
}

// ============================================
// Sensor calibration setters
// ============================================

void TankSensor::setVoltageMin(float v)  { _voltageMin  = v; }
void TankSensor::setVoltageMax(float v)  { _voltageMax  = v; }
void TankSensor::setSensorRange(float m) { _sensorRange = m; }
void TankSensor::setEmaAlpha(float a)    { _emaAlpha = constrain(a, 0.01f, 1.0f); }

// ============================================
// Tank geometry setters
// ============================================

void TankSensor::setTankType(TankType type)  { _tankType    = type; }
void TankSensor::setBaseSurface(float m2)    { _baseSurface = m2;   }
void TankSensor::setTankHeight(float m)      { _tankHeight  = m;    }
void TankSensor::setTankLength(float m)      { _tankLength  = m;    }

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
// Sampling configuration
// ============================================

void TankSensor::setSamplesPerRead(uint8_t n) {
  _samplesPerRead = (n > 0) ? n : 1;
}

uint8_t TankSensor::getSamplesPerRead() const {
  return _samplesPerRead;
}

// ============================================
// All-in-one configure (vertical tanks)
// ============================================

void TankSensor::configure(float voltMin, float voltMax, float sensorRange,
                            float baseSurface, float tankHeight, float emaAlpha) {
  _voltageMin  = voltMin;
  _voltageMax  = voltMax;
  _sensorRange = sensorRange;
  _baseSurface = baseSurface;
  _tankHeight  = tankHeight;
  _emaAlpha    = constrain(emaAlpha, 0.01f, 1.0f);
  _tankType    = TANK_VERTICAL;
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

  _voltageMin  = cfg.voltMin;
  _voltageMax  = cfg.voltMax;
  _sensorRange = cfg.sensorRange;
  _baseSurface = cfg.baseSurface;
  _tankHeight  = cfg.tankHeight;
  _tankLength  = cfg.tankLength;
  _emaAlpha    = cfg.emaAlpha;
  _gain        = (adsGain_t)cfg.adsGain;
  _tankType    = (TankType)cfg.tankType;
  if (_initialized) _ads.setGain(_gain);
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
  cfg.baseSurface  = _baseSurface;
  cfg.tankHeight   = _tankHeight;
  cfg.tankLength   = _tankLength;
  cfg.emaAlpha     = _emaAlpha;
  cfg.adsGain      = (uint16_t)_gain;
  cfg.tankType     = (uint8_t)_tankType;

  size_t sz = (size_t)_eepromAddr + sizeof(ConfigData);
  eepromBegin(sz);
  EEPROM.put(_eepromAddr, cfg);
  eepromCommitEnd();
}

// ============================================
// Getters
// ============================================

float     TankSensor::getVoltageMin()    const { return _voltageMin;  }
float     TankSensor::getVoltageMax()    const { return _voltageMax;  }
float     TankSensor::getSensorRange()   const { return _sensorRange; }
TankType  TankSensor::getTankType()      const { return _tankType;    }
float     TankSensor::getBaseSurface()   const { return _baseSurface; }
float     TankSensor::getTankHeight()    const { return _tankHeight;  }
float     TankSensor::getTankLength()    const { return _tankLength;  }
float     TankSensor::getEmaAlpha()      const { return _emaAlpha;    }
uint8_t   TankSensor::getAdsChannel()    const { return _channel;     }
adsGain_t TankSensor::getAdsGain()       const { return _gain;        }
uint8_t   TankSensor::getEepromAddress() const { return _eepromAddr;  }

// ============================================
// Private: volume calculation
// ============================================

float TankSensor::_volumeFromLevel(float levelM) const {
  if (_tankType == TANK_HORIZONTAL_CYLINDER) {
    // Circular segment area for a cylinder of diameter _tankHeight lying on its side.
    // h = liquid level (0 to diameter), r = internal radius.
    float r = _tankHeight * 0.5f;
    float h = constrain(levelM, 0.0f, _tankHeight);
    // Guard against h == 0 or h == diameter (acos domain edge)
    if (h <= 0.0f) return 0.0f;
    if (h >= _tankHeight) {
      return 3.14159265f * r * r * _tankLength * 1000.0f; // full cylinder
    }
    float area = r * r * acosf((r - h) / r) - (r - h) * sqrtf(2.0f * r * h - h * h);
    return area * _tankLength * 1000.0f; // m3 to litres
  }

  // TANK_VERTICAL: constant cross-section
  return _baseSurface * levelM * 1000.0f;
}

// ============================================
// Private: float map
// ============================================

float TankSensor::_mapFloat(float x, float inMin, float inMax,
                             float outMin, float outMax) const {
  if (inMax == inMin) return outMin;
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
