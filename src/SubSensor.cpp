#include <math.h>
#include <EEPROM.h>
#include "SubSensor.h"

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

SubSensor::SubSensor(uint8_t channel, uint8_t i2cAddr, uint8_t eepromAddr)
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

bool SubSensor::begin() {
  if (!_ads.begin(_i2cAddr)) return false;
  _ads.setGain(_gain);
  _initialized = true;
  return true;
}

TankReading SubSensor::read() {
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
  r.volumeLiters = _volumeFromLevel(r.levelMeters);
  r.volumeGallons = r.volumeLiters * 0.264172f;
  r.valid = true;
  return r;
}

void SubSensor::resetSmoothing() {
}

void SubSensor::setVoltageMin(float v)  { _voltageMin  = v; }
void SubSensor::setVoltageMax(float v)  { _voltageMax  = v; }
void SubSensor::setSensorRange(float m) { _sensorRange = m; }
void SubSensor::setEmaAlpha(float a)    { _emaAlpha = constrain(a, 0.01f, 1.0f); }

void SubSensor::setTankType(TankType type)  { _tankType    = type; }
void SubSensor::setBaseSurface(float m2)    { _baseSurface = m2;   }
void SubSensor::setTankHeight(float m)      { _tankHeight  = m;    }
void SubSensor::setTankLength(float m)      { _tankLength  = m;    }

void SubSensor::setAdsChannel(uint8_t channel) {
  _channel = channel;
  resetSmoothing();
}

void SubSensor::setAdsGain(adsGain_t gain) {
  _gain = gain;
  if (_initialized) _ads.setGain(_gain);
  resetSmoothing();
}

void SubSensor::setSamplesPerRead(uint8_t n) {
  _samplesPerRead = (n > 0) ? n : 1;
}

uint8_t SubSensor::getSamplesPerRead() const {
  return _samplesPerRead;
}

void SubSensor::configure(float voltMin, float voltMax, float sensorRange,
                            float baseSurface, float tankHeight, float emaAlpha) {
  _voltageMin  = voltMin;
  _voltageMax  = voltMax;
  _sensorRange = sensorRange;
  _baseSurface = baseSurface;
  _tankHeight  = tankHeight;
  _emaAlpha    = constrain(emaAlpha, 0.01f, 1.0f);
  _tankType    = TANK_VERTICAL;
}

void SubSensor::setEepromAddress(uint8_t addr) { _eepromAddr = addr; }

bool SubSensor::loadConfig() {
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

void SubSensor::saveConfig() {
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

float     SubSensor::getVoltageMin()    const { return _voltageMin;  }
float     SubSensor::getVoltageMax()    const { return _voltageMax;  }
float     SubSensor::getSensorRange()   const { return _sensorRange; }
TankType  SubSensor::getTankType()      const { return _tankType;    }
float     SubSensor::getBaseSurface()   const { return _baseSurface; }
float     SubSensor::getTankHeight()    const { return _tankHeight;  }
float     SubSensor::getTankLength()    const { return _tankLength;  }
float     SubSensor::getEmaAlpha()      const { return _emaAlpha;    }
uint8_t   SubSensor::getAdsChannel()    const { return _channel;     }
adsGain_t SubSensor::getAdsGain()       const { return _gain;        }
uint8_t   SubSensor::getEepromAddress() const { return _eepromAddr;  }

float SubSensor::_volumeFromLevel(float levelM) const {
  if (_tankType == TANK_HORIZONTAL_CYLINDER) {
    float r = _tankHeight * 0.5f;
    float h = constrain(levelM, 0.0f, _tankHeight);
    if (h <= 0.0f) return 0.0f;
    if (h >= _tankHeight) {
      return 3.14159265f * r * r * _tankLength * 1000.0f;
    }
    float area = r * r * acosf((r - h) / r) - (r - h) * sqrtf(2.0f * r * h - h * h);
    return area * _tankLength * 1000.0f;
  }

  return _baseSurface * levelM * 1000.0f;
}

float SubSensor::_mapFloat(float x, float inMin, float inMax,
                             float outMin, float outMax) const {
  if (inMax == inMin) return outMin;
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}