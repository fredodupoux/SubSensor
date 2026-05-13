/*
 * SubmersibleSensor -- BasicReading example
 *
 * Demonstrates reading a 4-20mA submersible hydrostatic pressure sensor (TL-136-like) via an ADS1115
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
 *   Sensor supply: typically 12-32V DC for TL-136-like hydrostatic sensors
 *   ADS1115 A0 tapped at the junction (sensor wire / top of shunt)
 *   10uF capacitor across the shunt for noise filtering
 *   Common GND between PSU and MCU
 */

#include <SubSensor.h>

// Construct with ADS1115 channel 0, I2C address 0x48, EEPROM start 0.
// Other examples:
//   SubSensor sensor(1);           // channel 1
//   SubSensor sensor(0, 0x49);     // alternate I2C address
//   SubSensor sensor(0, 0x48, 32); // custom EEPROM offset
SubSensor sensor;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!sensor.begin()) {
    Serial.println("ERROR: ADS1115 not found! Check wiring and I2C address.");
    while (1) delay(100);
  }

  if (sensor.loadConfig()) {
    Serial.println("Calibration loaded from EEPROM.");
  } else {
    Serial.println("No saved calibration -- applying defaults.");

    // ADS1115 gain: GAIN_ONE covers +/-4.096V, suits a 150ohm shunt (0.6-3.0V)
    sensor.setAdsGain(GAIN_ONE);

    // Sensor calibration: measure actual voltages at 4mA and 20mA
    sensor.setVoltageMin(0.60);   // 4 mA x 150 ohm = 0.60 V  (empty)
    sensor.setVoltageMax(3.00);   // 20 mA x 150 ohm = 3.00 V  (full)
    sensor.setSensorRange(5.0);   // sensor rated range in metres

    // -----------------------------------------------------------------
    // Option A: Vertical tank (any constant cross-section)
    //   Set the base surface area in m2 and the usable fill height.
    //   Works for upright cylinders, rectangular tanks, or any shape
    //   where the cross-section is constant -- just measure or compute
    //   the base area once.
    // -----------------------------------------------------------------
    sensor.setTankType(TANK_VERTICAL);
    sensor.setBaseSurface(0.0638); // pi * (0.285m / 2)^2  (28.5cm diameter)
    sensor.setTankHeight(0.365);   // usable height in metres (36.5 cm)

    // -----------------------------------------------------------------
    // Option B: Horizontal cylinder (uncomment to use)
    //   tankHeight = internal diameter of the cylinder (also the max fill).
    //   tankLength = axial length of the cylinder.
    //   The library computes the circular segment area at each level.
    // -----------------------------------------------------------------
    // sensor.setTankType(TANK_HORIZONTAL_CYLINDER);
    // sensor.setTankHeight(0.60);  // internal diameter: 60 cm
    // sensor.setTankLength(1.20);  // axial length:     120 cm

    // Smoothing (0.05 = heavy, 1.0 = no smoothing)
    sensor.setEmaAlpha(0.2);

    sensor.saveConfig();
    Serial.println("Calibration saved to EEPROM.");
  }

  Serial.println("Tank sensor ready.");
  Serial.println("------------------------------");
}

void loop() {
  TankReading r = sensor.read();

  if (r.valid) {
    Serial.print("ADC: ");       Serial.print(r.rawADC);
    Serial.print("  Voltage: "); Serial.print(r.voltage, 3);      Serial.print(" V");
    Serial.print("  Level: ");   Serial.print(r.levelCm, 1);      Serial.print(" cm");
    Serial.print("  Fill: ");    Serial.print(r.fillPercent, 1);   Serial.print(" %");
    Serial.print("  Volume: ");  Serial.print(r.volumeLiters, 1);  Serial.println(" L");
  } else {
    Serial.println("Sensor not ready -- call sensor.begin() in setup().");
  }

  delay(2000);
}
