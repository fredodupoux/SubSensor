/*
 * TankSensor -- BasicReading example
 *
 * Demonstrates reading a 4-20mA submersible level sensor via an ADS1115
 * 16-bit I2C ADC, restoring calibration from EEPROM on boot, and printing
 * a live reading to Serial.
 *
 * Requires: Adafruit ADS1X15 library (install via Library Manager)
 *
 * Hardware wiring (Wemos D1 Mini / ESP8266):
 *   ADS1115 VDD  --> 3.3V
 *   ADS1115 GND  --> GND
 *   ADS1115 SCL  --> D1
 *   ADS1115 SDA  --> D2
 *   ADS1115 ADDR --> GND   (I2C address 0x48)
 *
 *   Sensor 4-20mA output --> 150ohm shunt resistor --> GND
 *   ADS1115 A0 tapped at the junction (sensor wire / top of shunt)
 *   10uF capacitor across the shunt for noise filtering
 *   Common GND between PSU and MCU
 */

#include <TankSensor.h>

// Construct with ADS1115 channel 0, I2C address 0x48, EEPROM start 0.
// Examples of other configurations:
//   TankSensor tank(1);          // channel 1
//   TankSensor tank(0, 0x49);    // channel 0, alternate I2C address
//   TankSensor tank(0, 0x48, 32); // channel 0, EEPROM offset 32
TankSensor tank;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!tank.begin()) {
    Serial.println("ERROR: ADS1115 not found! Check wiring and I2C address.");
    while (1) delay(100);
  }

  // Attempt to restore saved calibration from EEPROM.
  if (tank.loadConfig()) {
    Serial.println("Calibration loaded from EEPROM.");
  } else {
    // No saved data -- set calibration for your hardware and save it.
    Serial.println("No saved calibration -- applying defaults.");

    // ADS1115 gain: GAIN_ONE covers +/-4.096V, suits a 150ohm shunt (0.6-3.0V)
    tank.setAdsGain(GAIN_ONE);

    // Sensor calibration: measure actual voltages at 4mA and 20mA and set here
    tank.setVoltageMin(0.60);    // 4 mA x 150 ohm = 0.60 V  (empty)
    tank.setVoltageMax(3.00);    // 20 mA x 150 ohm = 3.00 V  (full sensor range)
    tank.setSensorRange(5.0);    // sensor rated range in metres

    // Tank geometry
    tank.setTankDiameter(0.285); // inner diameter in metres  (28.5 cm)
    tank.setTankHeight(0.365);   // usable height in metres   (36.5 cm)

    // Smoothing (0.05 = heavy, 1.0 = no smoothing)
    tank.setEmaAlpha(0.2);

    tank.saveConfig();           // persist to EEPROM for next boot
    Serial.println("Calibration saved to EEPROM.");
  }

  Serial.println("Tank sensor ready.");
  Serial.println("------------------------------");
}

void loop() {
  TankReading r = tank.read();

  if (r.valid) {
    Serial.print("ADC: ");       Serial.print(r.rawADC);
    Serial.print("  Voltage: "); Serial.print(r.voltage, 3); Serial.print(" V");
    Serial.print("  Level: ");   Serial.print(r.levelCm, 1);  Serial.print(" cm");
    Serial.print("  Fill: ");    Serial.print(r.levelPercent, 1); Serial.print(" %");
    Serial.print("  Volume: ");  Serial.print(r.volumeLiters, 1); Serial.println(" L");
  } else {
    Serial.println("Sensor not ready -- call tank.begin() in setup().");
  }

  delay(2000);
}
