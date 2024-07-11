#include <Arduino.h>
#include <Wire.h>
#include <time.h>

#include <Adafruit_VEML7700.h>
#include <Adafruit_BME280.h>

#include "Custom_AHT21.h"
#include "Custom_ENS160.h"

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define SYSTEM_LED_PIN 2

long lastMsg = 0;

I2cInterface i2c;

CustomAHT21 aht21;
CustomENS160 ens160;
Adafruit_VEML7700 veml;
Adafruit_BME280 bme;

void initializeDebugging()
{
  Serial.begin(115200);
  if (DEBUG == 1)
  {
    ens160.ens.enableDebugging(Serial);
  }
}

void initializeI2C()
{
  Wire.begin();
  i2c.begin(Wire, ENS160_I2C_ADDRESS);
}

void initializeSensors()
{
  // initialize ENS160 sensor
  debugln("[initializeSensors] Initializing ENS160.");
  while (!ens160.ens.begin(&i2c))
  {
    debug(".");
    delay(1000);
  }
  ens160.ens.startStandardMeasure();
  // ens160.begin(&i2c);
  debugln("[initializeSensors] ENS160 initialized successfully.");

  // // initialize AHT21 sensor
  debugln("[initializeSensors] Initializing AHT21.");
  aht21.begin();
  debugln("[initializeSensors] AHT21 initialized successfully.");

  // debugln("[initializeSensors] Initializing VEML7700.");
  // while (!veml.begin())
  // {
  //   debug(".");
  //   delay(1000);
  // }
  // veml.setGain(VEML7700_GAIN_1); // 1/8, 1/4, 1, 2
  // veml.setIntegrationTime(VEML7700_IT_100MS);
  // debugln("[initializeSensors] VEML7700 initialized successfully.");

  debugln("[initializeSensors] Initializing bme.");

  // Default settings from datasheet
    while (!bme.begin(0x76))
  {
    debug(".");
    delay(1000);
  }
  debugln("[initializeSensors] bme initialized successfully.");
}

void setup()
{
  debugln("Starting setup...");
  pinMode(SYSTEM_LED_PIN, OUTPUT);
  digitalWrite(SYSTEM_LED_PIN, HIGH);

  initializeDebugging();
  initializeI2C();
  initializeSensors();

  debugln("Setup complete.");
}

void loop()
{
  long now = millis();
  if (now == 0 || now - lastMsg > 15000)
  {
    AHT21Data aht21d = aht21.read();
    String aht21log = "[loop] AHT21: TEMP: " + String(aht21d.temp) + "\tHUM: " + String(aht21d.humidity);
    debugln(aht21log.c_str());

    ENS160Data ens160d = ens160.read();
    String ens160log = "[loop] ENS160: ECO2: " + String(ens160d.eco2) + "\tTVOC: " + String(ens160d.tvoc);
    debugln(ens160log.c_str());

    // float lux = veml.readLux();
    // debug(F("Lux: "));
    // debugln(lux);

    lastMsg = now;

    // Serial.print("Temperature = ");
    // Serial.print(bme.readTemperature());
    // Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    // debugln(aht21log.c_str());
    // debugln(ens160log.c_str());

    // json["eco2"] = ens160d.eco2;
    // json["tvoc"] = ens160d.tvoc;
    // json["humidity"] = aht21d.humidity;
    // json["temperature"] = aht21d.temp;
  }

  // call loop() regularly to maintain MQTT connection and handle incoming messages
  // client.loop();
}
