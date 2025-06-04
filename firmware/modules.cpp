#include "esp_bt.h"
#include <WiFi.h>
#include <configuration.h>
#include "DHT.h"
#include <vector>
#include <TinyGPSPlus.h>

DHT environment_Sensor(ENVIRONMENT_SENSOR_PIN, DHT21);
std::vector<String> phoneNumbers;

// Additional required parameters to init modem (for GPS/SIM). From https://randomnerdtutorials.com/lilygo-ttgo-t-a7670g-a7670e-a7670sa-esp32/
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#define LILYGO_T_CALL_A7670_V1_0
#define MODEM_BAUDRATE                      (115200)
#define MODEM_DTR_PIN                       (14)
#define MODEM_TX_PIN                        (26)
#define MODEM_RX_PIN                        (25)
// The modem boot pin needs to follow the startup sequence.
#define BOARD_PWRKEY_PIN                    (4)
#define BOARD_LED_PIN                       (12)
// There is no modem power control, the LED Pin is used as a power indicator here.
#define BOARD_POWERON_PIN                   (BOARD_LED_PIN)
#define MODEM_RING_PIN                      (13)
#define MODEM_RESET_PIN                     (27)
#define MODEM_RESET_LEVEL                   LOW
#define SerialAT                            Serial1

#define MODEM_GPS_ENABLE_GPIO               (-1)
#define MODEM_GPS_ENABLE_LEVEL              (-1)

#ifndef TINY_GSM_MODEM_A7670
  #define TINY_GSM_MODEM_A7670
#endif

#include <TinyGsmClient.h>

TinyGsm modem(SerialAT);
TinyGsmClient gsmNet(modem);

#include <TinyGPSPlus.h>
TinyGPSPlus gps;

void printVariables () {
  #ifdef DEBUG
    Serial.println("--- Environmental variables ---");
    
    #pragma region Data
    Serial.println("[Data configurations]");
    Serial.print("Save data: ");

    #ifdef DATA_ENABLE  
      Serial.println("1");

      Serial.print("Data gathering rate: ");
      Serial.println(DATA_RATE);
      
      Serial.print("SIM APN: ");
      Serial.println(DATA_STORE_SIM_APN);
      
      Serial.print("SIM PIN: ");
      Serial.println(DATA_STORE_SIM_PIN);
    
      Serial.print("SIM target phone numbers: ");
      Serial.println(DATA_STORE_SIM_TARGET_NUMBERS);
    #endif
    #ifndef DATA_ENABLE
      Serial.println("0");
    #endif

    #pragma endregion Data

    #pragma region Moisture
    Serial.println("[Moisture]");
    Serial.print("Moisture enable: ");

    #ifdef MOISTURE_ENABLE
      Serial.println("1");

      Serial.print("Moisture sensor pin: ");
      Serial.println(MOISTURE_SENSOR_PIN);

      Serial.print("Moisture threshold: ");
      Serial.println(MOISTURE_THRESHOLD);

      Serial.print("Moisture minimum level (in air): ");
      Serial.println(MOISTURE_LEVEL_MIN);

      Serial.print("Moisture maximum level (in water): ");
      Serial.println(MOISTURE_LEVEL_MAX);

      Serial.print("Moisture water pump pin: ");
      Serial.println(MOISTURE_WATER_PUMP_PIN);

      Serial.print("Moisture water pump time: ");
      Serial.println(MOISTURE_WATER_PUMP_TIME);

      Serial.print("Moisture water pump minimum delay between each trigger: ");
      Serial.println(MOISTURE_WATER_PUMP_MINIMUM_DELAY);
    #endif
    #ifndef MOISTURE_ENABLE
      Serial.println("0");
    #endif

    #pragma endregion Moisture

    #pragma region Environment
    Serial.println("[Environment]");
    Serial.print("Environment enable: ");

    #ifdef ENVIRONMENT_ENABLE
      Serial.println("1");
      
      Serial.print("Environment sensor pin: ");
      Serial.println(ENVIRONMENT_SENSOR_PIN);

      Serial.print("Environment fans pin: ");
      Serial.println(ENVIRONMENT_FANS_PIN);

      Serial.print("Environment fans time ON: ");
      Serial.println(ENVIRONMENT_FANS_TIME);

      Serial.print("Environment humidity enable: ");
      #ifdef ENVIRONMENT_HUMIDITY_ENABLE
        Serial.println("1");

        Serial.print("Environment humidity threshold: ");
        Serial.println(ENVIRONMENT_HUMIDITY_THRESHOLD);  
      #endif
      #ifndef ENVIRONMENT_HUMIDITY_ENABLE
        Serial.println("0");
      #endif
      
      Serial.print("Environment temperature enable: ");
      #ifdef ENVIRONMENT_TEMPERATURE_ENABLE
        Serial.println("1");

        Serial.print("Environment temperature threshold: ");
        Serial.println(ENVIRONMENT_TEMPERATURE_THRESHOLD);  
      #endif
      #ifndef ENVIRONMENT_TEMPERATURE_ENABLE
        Serial.println("0");
      #endif

    #endif
    #ifndef ENVIRONMENT_ENABLE
      Serial.println("0");
    #endif

    #pragma endregion Environment

    #pragma region Light
    Serial.println("[Light]");
    Serial.print("Light enable: ");

    #ifdef LIGHT_ENABLE
      Serial.println("1");

      Serial.print("Light sensor pin: ");
      Serial.println(LIGHT_SENSOR_PIN);

      Serial.print("Light threshold: ");
      Serial.println(LIGHT_THRESHOLD);

      Serial.print("Light LEDs pin: ");
      Serial.println(LIGHT_LEDS_PIN);
      
      Serial.print("Light LEDs time ON: ");
      Serial.println(LIGHT_LEDS_TIME);
    #endif
    #ifndef LIGHT_ENABLE
      Serial.println("0");
    #endif

    #pragma endregion Light

    #pragma region Sleep
    Serial.println("[Sleep]");
    Serial.print("Sleep enable: ");

    #ifdef SLEEP_ENABLE
      Serial.println("1");

      Serial.print("Sleep start time: ");
      Serial.println(SLEEP_START_TIME);

      Serial.print("Sleep end time: ");
      Serial.println(SLEEP_END_TIME);

      Serial.print("Sleep timezone offset: ");
      Serial.println(SLEEP_TIMEZONE_OFFSET);
    #endif
    #ifndef SLEEP_ENABLE
      Serial.println("0");
    #endif

    #pragma endregion Sleep

    Serial.println();
  #endif
}

/*
 * NOTES:
 * - The relay logic is reversed so the signal must be turned off (LOW) to trigger the device, and set on (HIGH) to turn it off
 */

#pragma region Moisture

float moisture_GetLevel () {
   // Read raw sensor value
  float rawValue = analogRead(MOISTURE_SENSOR_PIN);
  
  // Convert to moisture percentage
  float moisturePercentage = map(rawValue, MOISTURE_LEVEL_MIN, MOISTURE_LEVEL_MAX, 0, 100);
  return constrain(moisturePercentage, 0, 100);
}

void moisture_PumpWater_ON () {
  digitalWrite(MOISTURE_WATER_PUMP_PIN, LOW);
}

void moisture_PumpWater_OFF () {
  digitalWrite(MOISTURE_WATER_PUMP_PIN, HIGH);
}

#pragma endregion Moisture

#pragma region Environment

float environment_Humidity_GetLevel () {
  float read = environment_Sensor.readHumidity();
  if (isnan(read)) read = 0;

  return read;
}

float environment_Temperature_GetLevel () {
  float read = environment_Sensor.readTemperature();
  if (isnan(read)) read = 0;

  return read;
}

void environment_Fans_ON () {
  digitalWrite(ENVIRONMENT_FANS_PIN, LOW);
}

void environment_Fans_OFF () {
  digitalWrite(ENVIRONMENT_FANS_PIN, HIGH);
}

#pragma endregion Environment

#pragma region Light

float light_GetLevel () {
   // Read raw sensor value
  float rawValue = analogRead(LIGHT_SENSOR_PIN);
  
  // Convert to light percentage
  float lightPercentage = map(rawValue, LIGHT_LEVEL_MIN, LIGHT_LEVEL_MAX, 0, 100);
  return constrain(lightPercentage, 0, 100);
}

void light_LEDs_ON () {
  digitalWrite(LIGHT_LEDS_PIN, LOW);
}

void light_LEDs_OFF () {
  digitalWrite(LIGHT_LEDS_PIN, HIGH);
}

#pragma endregion Light

#pragma region Data


// Returns current hour
short getCurrentHour() {
  // When starting up will happen that the GPS is not reachable and return all 0 (satellites = 0). In that case return -1
  if(gps.satellites.value() == 0) return -1;

  short hour = gps.time.hour() + SLEEP_TIMEZONE_OFFSET;
  // If the value of hours exceed the 24h. For example in case of 11PM + timezone 2h = 25 => 01AM
  if (hour >= 24) return hour -24;
  else return hour;

}

void data_Store(float moisture, float humidity, float temperature, float light) {
  #ifdef DATA_ENABLE
    #ifdef DEBUG
      Serial.print("Store data: moisture = ");
      Serial.print(moisture);
      Serial.print(", humidity = ");
      Serial.print(humidity);
      Serial.print(", temperature = ");
      Serial.print(temperature);
      Serial.print(", light = ");
      Serial.println(light);

      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value()); // Number of satellites in use (u32)

	    String ret = String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()) + " " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      Serial.print("UTC Time: ");
      Serial.println(ret);
    #endif

    String smsMessage = "Periodic greenhouse report.\nMoisture level: " + String(moisture) + "\nHumidity level: " + String(humidity) + "\nTemperature: " + String(temperature) + "\nLight: " + String(light);
    for (const auto& num : phoneNumbers) {
      String message = modem.sendSMS(num, smsMessage) ? "SMS sent successfully to " : "SMS failed to sent to ";
      Serial.print(message);
      Serial.println(num);
    }
  #endif
  
}

#pragma endregion Data

#pragma region Sleep

void sleep_Check () {
  short hour = getCurrentHour();

  if (hour == -1) return;

  // Normal time, for example 14 to 22
  #if SLEEP_START_TIME < SLEEP_END_TIME
    bool inside = hour >= SLEEP_START_TIME && hour <= SLEEP_END_TIME;
  #endif
  // Day overlap time, for example 22 to 06
  #if SLEEP_START_TIME > SLEEP_END_TIME
    bool inside = hour >= SLEEP_START_TIME || hour <= SLEEP_END_TIME;
  #endif

  if (inside) {
    #if SLEEP_START_TIME < SLEEP_END_TIME
      bool time = (SLEEP_END_TIME - SLEEP_START_TIME);
    #endif
    #if SLEEP_START_TIME > SLEEP_END_TIME
      bool time = ((24 - SLEEP_START_TIME) + SLEEP_END_TIME);
    #endif

    Serial.print("Go to sleep for ");
    Serial.print(time);
    Serial.println(" hours!");
    // https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
    // Wake up timer (time * minutes in an hour * seconds in an hour * microseconds in a second)
    esp_sleep_enable_timer_wakeup(time * 60 * 60 * 1000000ULL);
    // Go to sleep
    esp_deep_sleep_start();
  }
}

#pragma endregion Sleep

void startup () {
  // - Turn off WiFi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  // - Turn off Bluetooth
  btStop();

  // Start modem
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);

  // Set modem reset pin, reset modem
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);

  // Check if the modem is online
  Serial.print("Start modem...");

  int retry = 0;
  while (!modem.testAT(1000)) {
    Serial.print(".");
    if (retry++ > 10) {
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_PWRKEY_PIN, HIGH);
      delay(1000);
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      retry = 0;
    }
  }
  Serial.println();

  // Moisture
  #ifdef MOISTURE_ENABLE
    // Turn-off water pump
    pinMode(MOISTURE_WATER_PUMP_PIN, OUTPUT);
    moisture_PumpWater_OFF();
  #endif

  // Environment
  #ifdef ENVIRONMENT_ENABLE
    // Turn-off fans
    pinMode(ENVIRONMENT_FANS_PIN, OUTPUT);
    environment_Fans_OFF();

    // Initialize environment sensor
    environment_Sensor.begin();
  #endif 

  // Light control
  #ifdef LIGHT_ENABLE
    // Sensor pin
    pinMode(LIGHT_SENSOR_PIN, INPUT);
    // Turn-off the LEDs
    pinMode(LIGHT_LEDS_PIN, OUTPUT);
    light_LEDs_OFF();
  #endif

  // Store data
  #ifdef DATA_ENABLE
    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY) {
      sim = modem.getSimStatus();
      switch (sim) {
        case SIM_READY:
          Serial.println("SIM card online");
          break;
        case SIM_LOCKED:
          Serial.println("The SIM card is locked. Please unlock the SIM card first.");
          modem.simUnlock(DATA_STORE_SIM_PIN);
          break;
        default:
          break;
      }
      delay(1000);
    }

    if (!modem.setNetworkMode(MODEM_NETWORK_AUTO)) {
      Serial.println("Set network mode failed!");
    }
    String mode = modem.getNetworkModes();
    Serial.print("Current network mode : ");
    Serial.println(mode);

    Serial.printf("Set network apn : %s\n", DATA_STORE_SIM_APN);
    modem.sendAT(GF("+CGDCONT=1,\"IP\",\""), DATA_STORE_SIM_APN, "\"");
    if (modem.waitResponse() != 1) {
      Serial.println("Set network apn error !");
    }

    // Check network registration status and network signal status
    int16_t sq ;
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) {
      status = modem.getRegistrationStatus();
      switch (status) {
        case REG_UNREGISTERED:
        case REG_SEARCHING:
          sq = modem.getSignalQuality();
          Serial.printf("[%lu] Signal Quality:%d\n", millis() / 1000, sq);
          delay(1000);
          break;
        case REG_DENIED:
          Serial.println("Network registration was rejected, please check if the APN is correct");
          return ;
        case REG_OK_HOME:
          Serial.println("Online registration successful");
          break;
        case REG_OK_ROAMING:
          Serial.println("Network registration successful, currently in roaming mode");
          break;
        default:
          Serial.printf("Registration Status:%d\n", status);
          delay(1000);
          break;
      }
    }
    Serial.println();

    Serial.printf("Registration Status:%d\n", status);
    delay(1000);

    String ueInfo;
    if (modem.getSystemInformation(ueInfo)) {
      Serial.print("Inquiring UE system information:");
      Serial.println(ueInfo);
    }

    if (!modem.setNetworkActive()) {
      Serial.println("Enable network failed!");
    }

    Serial.println("Enabling GPS/GNSS/GLONASS");
    while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
        Serial.print(".");
    }

    Serial.println();
    Serial.println("GPS Enabled");

    modem.setGPSBaud(115200);

    modem.setGPSMode(3);    //GPS + BD

    modem.configNMEASentence(1, 1, 1, 1, 1, 1);

    modem.setGPSOutputRate(1);

    modem.enableNMEA();

    String ipAddress = modem.getLocalIP();
    Serial.print("Network IP:"); Serial.println(ipAddress);

    // Compile phone numbers
    int start = 0;
    
    String targets = DATA_STORE_SIM_TARGET_NUMBERS;
    for (int i = 0; i < targets.length(); i++) {
      if (targets[i] == ',') {
        phoneNumbers.push_back(targets.substring(start, i));
        start = i + 1;
      }
    }
    // Add last number (after last comma)
    if (start < targets.length()) {
      phoneNumbers.push_back(targets.substring(start));
    }
  #endif
}

// This custom version of delay() ensures that the gps object is being "fed".
void smartDelay(unsigned long ms)
{
  int ch = 0;
  unsigned long start = millis();
  do {
    while (SerialAT.available()) {
      ch = SerialAT.read();
      gps.encode(ch);
    }
  } while (millis() - start < ms);
}
