/*
 * Micromouse - OLED I2C Scanner
 * Target: STM32F401CCU6 Black Pill
 *
 * Existing PCB connections - DO NOT CHANGE:
 *   I2C SCL -> PB8
 *   I2C SDA -> PB9
 *
 * This test does not use motors, encoders, IMU, VL53L0X, DIP switches,
 * or the KEY button. It only checks whether an I2C device is visible.
 */

#include <Arduino.h>
#include <Wire.h>

#define OLED_SCL PB8
#define OLED_SDA PB9

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("STM32F401 OLED I2C SCANNER");
    Serial.println("================================");
    Serial.println("SCL = PB8");
    Serial.println("SDA = PB9");
    Serial.println();

    Wire.setSCL(OLED_SCL);
    Wire.setSDA(OLED_SDA);
    Wire.begin();
    Wire.setClock(100000);   // Start conservatively at 100 kHz

    delay(100);

    uint8_t found = 0;

    Serial.println("Scanning I2C bus...");

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at 0x");
            if (address < 16) Serial.print('0');
            Serial.println(address, HEX);
            found++;
        }
    }

    Serial.println();

    if (found == 0)
    {
        Serial.println("NO I2C DEVICES FOUND!");
        Serial.println();
        Serial.println("Check ONLY these items:");
        Serial.println("1. OLED VCC -> 3.3V");
        Serial.println("2. OLED GND -> GND");
        Serial.println("3. OLED SCL -> PB8");
        Serial.println("4. OLED SDA -> PB9");
        Serial.println("5. OLED module is powered");
        Serial.println("6. SDA/SCL have pull-ups if required");
    }
    else
    {
        Serial.print("Devices found: ");
        Serial.println(found);
        Serial.println();
        Serial.println("For a normal SSD1306 OLED, expect:");
        Serial.println("0x3C or 0x3D");
    }

    Serial.println();
    Serial.println("Scan complete.");
}

void loop()
{
    // Repeat the scan every 3 seconds so the result can be observed
    // without resetting the STM32.
    delay(3000);

    Serial.println();
    Serial.println("--- Rescanning I2C ---");

    uint8_t found = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("Found: 0x");
            if (address < 16) Serial.print('0');
            Serial.println(address, HEX);
            found++;
        }
    }

    if (found == 0)
        Serial.println("No I2C devices found.");
}
