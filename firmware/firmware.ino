#include <modules.h>

long moisture_PumpWater_TimeOn = -1;
long moisture_PumpWater_LastTrigger = -1;

long humidity_Fans_TimeOn = -1;
// Set equal to DATA_RATE to trigger first time in the loop
long data_LastLog = DATA_RATE;

void setup() {
  Serial.begin(115200);

  #if START_UP_PRINT_ENV == 1
  printVariables();
  #endif

  startup();
}

void loop() {
  int moisture = -1;
  int humidity = -1;
  float light = -1;

  #ifdef MOISTURE_ENABLE
    moisture = moisture_GetLevel();
    // Water level under the threshold, turn on the if TimeOn is -1 (= pump is off)
    // Consider also that must pass a minimum time between one pump to another 
    if (moisture >= MOISTURE_THRESHOLD && moisture_PumpWater_TimeOn == -1 && (millis() - moisture_PumpWater_LastTrigger) > MOISTURE_WATER_PUMP_MINIMUM_DELAY) {
      moisture_PumpWater_TimeOn = millis();
      moisture_PumpWater_ON();

      Serial.print("Water level under threshold: ");
      Serial.print(moisture);
      Serial.println("%. Turning on the pump...");
    // Water level is below threshold but to stop the pump is also necessary to wait for the minimum time of the pump to stay on
    } else if (moisture < MOISTURE_THRESHOLD && (millis() - moisture_PumpWater_TimeOn) > MOISTURE_WATER_PUMP_TIME) {
      moisture_PumpWater_OFF();
      moisture_PumpWater_TimeOn = -1;
      moisture_PumpWater_LastTrigger = millis();
    }
  #endif

  #ifdef HUMIDITY_ENABLE
    humidity = humidity_GetLevel();
    // Humidity reached threshold, turn on fans if TimeOn is -1 (= fans off)
    if (humidity >= HUMIDITY_THRESHOLD && humidity_Fans_TimeOn == -1) {
      humidity_Fans_TimeOn = millis();
      humidity_Fans_ON();

      Serial.print("Humidity detected: ");
      Serial.print(humidity);
      Serial.println("%. Turning on the fans...");
    // Humidity is below threshold but to stop the fans is also necessary to wait for the minimum time of the fans to stay on
    } else if (humidity < HUMIDITY_THRESHOLD && (millis() - humidity_Fans_TimeOn) > HUMIDITY_FANS_MINIMUM_TIME) {
      humidity_Fans_OFF();
      humidity_Fans_TimeOn = -1;
    }
  #endif

  #ifdef LIGHT_CONTROL_ENABLE
    light = lightControl_GetLevel();

    if (light < LIGHT_CONTROL_THRESHOLD) {
      lightControl_LEDs_ON();
    } else {
      lightControl_LEDs_OFF();
    }
  #endif

  #ifdef DATA_ENABLE
  // Store data each DATA_RATE milliseconds
  if (millis() - data_LastLog >= DATA_RATE) {
    data_Store(moisture, humidity, light);
    data_LastLog = millis();
  }
  #endif
}
