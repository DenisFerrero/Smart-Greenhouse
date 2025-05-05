#include <WiFi.h>
#include "esp_bt.h"
#include "DHT.h"
#include <RTClib.h>
#include <configuration.h>
#include "FS.h"
#include "SD_MMC.h"
#include <Wire.h>

DHT humidity_Sensor(HUMIDITY_SENSOR_PIN, DHT21);
RTC_DS3231 RTC_Sensor;

void printVariables () {
  Serial.println("--- Environmental variables ---");
  
  #pragma region Data
  Serial.println("[Data configurations]");
  Serial.print("Save data: ");
  #ifdef DATA_ENABLE
    Serial.println("1");
  #endif
  #ifndef DATA_ENABLE
    Serial.println("0");
  #endif

  #ifdef DATA_ENABLE
    Serial.print("Data store mode: ");
    #if DATA_STORE_MODE == 1
      Serial.println("1. SD card");
    #endif

    Serial.print("Data gathering rate: ");
    Serial.println(DATA_RATE);
    
    Serial.print("Data filename: ");
    Serial.println(DATA_FILENAME);
  #endif

  #pragma endregion Data

  #pragma region Moisture
  Serial.println("[Moisture]");
  Serial.print("Moisture enable: ");

  #ifdef MOISTURE_ENABLE
    Serial.println("1");
  #endif
  #ifndef MOISTURE_ENABLE
    Serial.println("0");
  #endif

  #ifdef MOISTURE_ENABLE
    Serial.print("Moisture sensor pin: ");
    Serial.println(MOISTURE_SENSOR_PIN);

    Serial.print("Moisture threshold: ");
    Serial.println(MOISTURE_THRESHOLD);

    Serial.print("Moisture minimum level (air): ");
    Serial.println(MOISTURE_LEVEL_MIN);

    Serial.print("Moisture maximum level (water): ");
    Serial.println(MOISTURE_LEVEL_MAX);

    Serial.print("Moisture water pump pin: ");
    Serial.println(MOISTURE_WATER_PUMP_PIN);

    Serial.print("Moisture water pump time: ");
    Serial.println(MOISTURE_WATER_PUMP_TIME);
  #endif

  #pragma endregion Moisture

  #pragma region Humidity
  Serial.println("[Humidity]");
  Serial.print("Humidity enable: ");

  #ifdef HUMIDITY_ENABLE
    Serial.println("1");
  #endif
  #ifndef HUMIDITY_ENABLE
    Serial.println("0");
  #endif

  #ifdef HUMIDITY_ENABLE
    Serial.print("Humidity sensor pin: ");
    Serial.println(HUMIDITY_SENSOR_PIN);

    Serial.print("Humidity threshold: ");
    Serial.println(HUMIDITY_THRESHOLD);

    Serial.print("Humidity fans pin: ");
    Serial.println(HUMIDITY_FANS_PIN);
  #endif

  #pragma endregion Humidity

  #pragma region Light control
  Serial.println("[Light control]");
  Serial.print("Light control enable: ");

  #ifdef LIGHT_CONTROL_ENABLE
    Serial.println("1");
  #endif
  #ifndef LIGHT_CONTROL_ENABLE
    Serial.println("0");
  #endif

  #ifdef LIGHT_CONTROL_ENABLE
    Serial.print("Light control pin: ");
    Serial.println(LIGHT_CONTROL_SENSOR_PIN);

    Serial.print("Light control threshold: ");
    Serial.println(LIGHT_CONTROL_THRESHOLD);

    Serial.print("Light control LEDs pin: ");
    Serial.println(LIGHT_CONTROL_LEDS_PIN);
  #endif

  #pragma endregion Light control

  #pragma region RTC
  Serial.println("[RTC]");
  Serial.print("RTC enable: ");

  #ifdef RTC_ENABLE
    Serial.println("1");
  #endif
  #ifndef RTC_ENABLE
    Serial.println("0");
  #endif

  #ifdef RTC_ENABLE
    Serial.print("RTC interrupt pin: ");
    Serial.println(RTC_INTERRUPT_PIN);

    Serial.print("RTC sleep starts at: ");
    Serial.print(RTC_SLEEP_START);
    Serial.println(":00");

    Serial.print("RTC sleep ends at: ");
    Serial.print(RTC_SLEEP_END);
    Serial.println(":00");
  #endif

  #pragma endregion RTC

  Serial.println();
}

#pragma region Moisture

int moisture_GetLevel () {
   // Read raw sensor value
  int rawValue = analogRead(MOISTURE_SENSOR_PIN);
  
  // Convert to moisture percentage
  int moisturePercentage = map(rawValue, MOISTURE_LEVEL_MIN, MOISTURE_LEVEL_MAX, 0, 100);
  return constrain(moisturePercentage, 0, 100);
}

void moisture_PumpWater_ON () {
  digitalWrite(MOISTURE_WATER_PUMP_PIN, HIGH);
}

void moisture_PumpWater_OFF () {
  digitalWrite(MOISTURE_WATER_PUMP_PIN, LOW);
}

#pragma endregion Moisture

#pragma region Humidity

float humidity_GetLevel () {
  float read = humidity_Sensor.readHumidity();
  // Set 100 to always enable the fan when humidity is unreadable 
  if (isnan(read)) read = 0;

  return read;
}

void humidity_Fans_ON () {
  digitalWrite(HUMIDITY_FANS_PIN, HIGH);
}

void humidity_Fans_OFF () {
  digitalWrite(HUMIDITY_FANS_PIN, LOW);
}

#pragma endregion Humidity

#pragma region Light control

int lightControl_GetLevel () {
  int read = analogRead(LIGHT_CONTROL_SENSOR_PIN);
  return read;
}

void lightControl_LEDs_ON () {
  digitalWrite(LIGHT_CONTROL_LEDS_PIN, HIGH);
}

void lightControl_LEDs_OFF () {
  digitalWrite(LIGHT_CONTROL_LEDS_PIN, LOW);
}

#pragma endregion Light control

#pragma region RTC

void RTC_Sleep () {
  // Get tomorrow date by adding 1 day to now
  DateTime tomorrow = RTC_Sensor.now() + TimeSpan(1, 0, 0, 0);
  // Set Alarm 1 for wake up
  DateTime nextWake = DateTime(tomorrow.year(), tomorrow.month(), tomorrow.day(), RTC_SLEEP_END, 0, 0);
  RTC_Sensor.setAlarm1(nextWake, DS3231_A1_Hour);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)(RTC_INTERRUPT_PIN), 0); // Wake on INT LOW
  Serial.print("Going to sleep until ");
  Serial.print(RTC_SLEEP_END);
  Serial.println(":00...");
  delay(1000);
  esp_deep_sleep_start();
}

// Event attacched to an alarm to turn off the ESP32
void IRAM_ATTR RTC_Alarm() {
  detachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN)); // Prevent retrigger
  RTC_Sensor.clearAlarm(2); // Clear Alarm 2 flag

  RTC_Sleep();
}

#pragma endregion RTC

#pragma region Data

File data_GetFile (String filename) {
  File file = SD_MMC.open(filename);
  // File not exists, create it and append the header
  if (!file) {
    file = SD_MMC.open(filename, FILE_WRITE);
    file.println("Timestamp,Moisture,Humidity,Light"); // CSV header
  }

  // Close file and open it in append mode
  file.close();
  file = SD_MMC.open(filename, FILE_APPEND);

  return file;
}

void data_Store(int moisture, int humidity, float light) {
  #if DATA_STORE_MODE == 1
    Serial.print("Store data: moisture = ");
    Serial.print(moisture);
    Serial.print(", humidity = ");
    Serial.print(humidity);
    Serial.print(", light = ");
    Serial.println(light);
  
    DateTime now = RTC_Sensor.now();
    String date = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day());
    // Use a daily file rotation to avoid huge files and a daily analysis
    String filename = String("/") + DATA_FILENAME + "_" + date + ".csv";
    File file = data_GetFile(filename);

    if (!file) {
      Serial.print("Cannot open ");
      Serial.print(filename);
      Serial.println(" to save the data");

      return;
    }

    String timestamp = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()); 
    String data = timestamp + "," + String(moisture) + "," + String(humidity) + "," + String(light);

    file.println(data);

    file.close();

  #endif
}

#pragma endregion Data

void startup () {
  #ifdef DATA_ENABLE
    #if DATA_STORE_MODE == 1
      // When saving on SD card:
      // - Turn off WiFi
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      // - Turn off Bluetooth
      btStop();
    #endif
  #endif

  // Moisture
  #ifdef MOISTURE_ENABLE
    // Turn-off water pump
    pinMode(MOISTURE_WATER_PUMP_PIN, OUTPUT);
    digitalWrite(MOISTURE_WATER_PUMP_PIN, LOW);
  #endif

  // Humdity
  #ifdef HUMIDITY_ENABLE
    // Turn-off fans
    pinMode(HUMIDITY_FANS_PIN, OUTPUT);
    digitalWrite(HUMIDITY_FANS_PIN, LOW);

    // Initialize humidity sensor
    humidity_Sensor.begin();
  #endif 

  // Light control
  #ifdef LIGHT_CONTROL_ENABLE
    // Turn-off the LEDs
    pinMode(LIGHT_CONTROL_LEDS_PIN, OUTPUT);
    digitalWrite(LIGHT_CONTROL_LEDS_PIN, LOW);
  #endif

  // RTC
  #ifdef RTC_ENABLE
    Wire.begin(RTC_SDA_PIN, RTC_SDA_SCL)
    // Cannot start the RTC module
    if (!RTC_Sensor.begin()) {
      Serial.println("Couldn't find RTC");
      while (1) delay(1000);
    }
    if (RTC_Sensor.lostPower()) {
      // Set to compile time:
      RTC_Sensor.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    // Not used feature
    RTC_Sensor.disable32K();
    // Set interrupt pin as pullup
    pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);

    // Clear any previous alarms
    RTC_Sensor.clearAlarm(1);
    RTC_Sensor.clearAlarm(2);
    RTC_Sensor.writeSqwPinMode(DS3231_OFF);
    RTC_Sensor.disableAlarm(2);

    DateTime now = RTC_Sensor.now();
    // Check if in the period of sleep
    bool goSleep = (now.hour() >= RTC_SLEEP_START || now.hour() < RTC_SLEEP_END);

    if (goSleep) {
      RTC_Sleep();
    } else {
      // Set the alarm for the sleep to start
      DateTime nextSleep = DateTime(now.year(), now.month(), now.day(), RTC_SLEEP_START, 0, 0);
      RTC_Sensor.setAlarm2(nextSleep, DS3231_A2_Hour);
      attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), RTC_Alarm, FALLING);

      Serial.print("Modules will stay up until ");
      Serial.print(RTC_SLEEP_START);
      Serial.println(":00...");
    }
  #endif

  #ifdef DATA_ENABLE
    #ifndef RTC_ENABLE
      #error "Cannot store data without RTC module active"
    #endif

    #if DATA_STORE_MODE == 1
      if (!SD_MMC.begin()) {
        Serial.println("SD Card Mount Failed");
        while (1) delay (1000);
      }
    #endif
  #endif
}
