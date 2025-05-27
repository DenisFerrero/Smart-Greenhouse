#include "esp_bt.h"
#include <WiFi.h>
#include <configuration.h>

#ifdef ENVIRONMENT_ENABLE
  #include "DHT.h"
  DHT environment_Sensor(ENVIRONMENT_SENSOR_PIN, DHT21);
#endif

// Additional required parameters to init modem (for GPS/SIM). From https://randomnerdtutorials.com/lilygo-ttgo-t-a7670g-a7670e-a7670sa-esp32/
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define LILYGO_T_A7670
#if defined(LILYGO_T_A7670)
  #define MODEM_BAUDRATE                      (115200)
  #define MODEM_DTR_PIN                       (25)
  #define MODEM_TX_PIN                        (26)
  #define MODEM_RX_PIN                        (25)
  // The modem boot pin needs to follow the startup sequence.
  #define BOARD_PWRKEY_PIN                    (4)
  #define BOARD_ADC_PIN                       (35)
  // The modem power switch must be set to HIGH for the modem to supply power.
  #define BOARD_POWERON_PIN                   (12)
  #define MODEM_RING_PIN                      (33)
  #define MODEM_RESET_PIN                     (5)
  #define BOARD_MISO_PIN                      (2)
  #define BOARD_MOSI_PIN                      (15)
  #define BOARD_SCK_PIN                       (14)
  #define BOARD_SD_CS_PIN                     (13)
  #define BOARD_BAT_ADC_PIN                   (35)
  #define MODEM_RESET_LEVEL                   HIGH
  #define SerialAT                            Serial1
  #define MODEM_GPS_ENABLE_GPIO               (-1)
  #define MODEM_GPS_ENABLE_LEVEL              (-1)
  #ifndef TINY_GSM_MODEM_A7670
    #define TINY_GSM_MODEM_A7670
  #endif
#endif

#include <TinyGsmClient.h>

TinyGsm modem(SerialAT);
TinyGsmClient gsmNet(modem);

void printVariables () {
  #if START_UP_PRINT_ENV == 1
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
    #endif
    #ifndef LIGHT_ENABLE
      Serial.println("0");
    #endif

    #pragma endregion Light

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
  float lightPercentage = map(rawValue, LIGHT_LEVEL_MIN, LIGHT_LEVEL_MIN, 0, 100);
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

void data_Store(float moisture, float humidity, float temperature, float light) {
  #ifdef DATA_ENABLE
    Serial.print("Store data: moisture = ");
    Serial.print(moisture);
    Serial.print(", humidity = ");
    Serial.print(humidity);
    Serial.print(", temperature = ");
    Serial.print(temperature);
    Serial.print(", light = ");
    Serial.println(light);

    // TODO Send SMS
  #endif
  
}

#pragma endregion Data

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
  Serial.println("Start modem...");

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
  #ifdef LIGHT_CONTROL_ENABLE
    // Sensor pin
    pinMode(LIGHT_CONTROL_SENSOR_PIN, INPUT);
    // Turn-off the LEDs
    pinMode(LIGHT_CONTROL_LEDS_PIN, OUTPUT);
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
    delay(5000);

    String ipAddress = modem.getLocalIP();
    Serial.print("Network IP:"); Serial.println(ipAddress);
  #endif
}
