#include <modules.h>

long moisture_PumpWater_TimeOn = -1;
long moisture_PumpWater_LastTrigger = -1;

long environment_Fans_TimeOn = -1;

bool light_isOn = false;
long leds_TimeOn = -1;
long leds_LastTrigger = -1;

long data_LastLog = -1;

unsigned long counterTime = 0;

float moisture = -1;
float humidity = -1;
float temperature = -1;
float light = -1;

void setup() {
  Serial.begin(115200);
  // Wait for serial to bind correctly
  delay(5 * 1000);
  printVariables();

  startup();

  counterTime = millis();
}

void loop() {
  moisture = moisture_GetLevel();
  humidity = environment_Humidity_GetLevel();
  temperature = environment_Temperature_GetLevel();
  light = light_GetLevel();

  #ifdef MOISTURE_ENABLE
    // Water level is under the threshold
    bool moisture_underThreshold = moisture <= MOISTURE_THRESHOLD;
    // The water pump is off
    bool waterPump_isOff = moisture_PumpWater_TimeOn == -1;
    // Minimum time between one pump to another is elapsed. At first init check also if -1 to prevent trigger for the first time after MOISTURE_WATER_PUMP_MINIMUM_DELAY ms  
    bool waterPump_elapsedMinimumTime = (moisture_PumpWater_LastTrigger == -1 || (millis() - moisture_PumpWater_LastTrigger) > MOISTURE_WATER_PUMP_MINIMUM_DELAY); 
    // Minimum time of which the water pump must be kept on after powering on
    bool waterPump_elapsedRunningTime = (moisture_PumpWater_TimeOn != -1 && (millis() - moisture_PumpWater_TimeOn) > MOISTURE_WATER_PUMP_TIME);

    if (moisture_underThreshold && waterPump_isOff && waterPump_elapsedMinimumTime) {
      moisture_PumpWater_TimeOn = millis();
      moisture_PumpWater_ON();

      Serial.print("Water level under threshold: ");
      Serial.print(moisture);
      Serial.println("%. Turning on the pump...");
    }
    // Do not consider the threshold because the water needs time to be absorbed by the terrain 
    else if (waterPump_elapsedRunningTime) {
      moisture_PumpWater_OFF();
      moisture_PumpWater_TimeOn = -1;
      moisture_PumpWater_LastTrigger = millis();
    }
  #endif

  #ifdef ENVIRONMENT_ENABLE
    // Humidity is over the allowed threshold
    bool humidity_overThreshold = false;
    // Temperature is over the allowed threshold
    bool temperature_overThreshold = false;

    // Calculate it only if the humidity detection is enabled
    #ifdef ENVIRONMENT_HUMIDITY_ENABLE
      humidity_overThreshold = humidity >= ENVIRONMENT_HUMIDITY_THRESHOLD;
    #endif
    // Calculate it only if the temperature detection is enabled
    #ifdef ENVIRONMENT_TEMPERATURE_ENABLE
      temperature_overThreshold = temperature >= ENVIRONMENT_TEMPERATURE_THRESHOLD;
    #endif

    // The fans are off
    bool fans_areOff = environment_Fans_TimeOn == -1;
    // Minimum time of which the fans must be kept on after powering on
    bool fans_elapsedRunningTime = (environment_Fans_TimeOn != -1 && (millis() - environment_Fans_TimeOn) > ENVIRONMENT_FANS_TIME);

    // Humidity or temperature reached the threshold, turn on fans if they are off
    if ((humidity_overThreshold || temperature_overThreshold) && fans_areOff) {
      environment_Fans_TimeOn = millis();
      environment_Fans_ON();

      Serial.print("Humidity detected: ");
      Serial.print(humidity);
      Serial.println("%. Temperature detected: ");
      Serial.print(temperature);
      Serial.println("°C. Turning on the fans...");
    // The fans are on and the running minimum time is elapsed
    } else if (fans_elapsedRunningTime) {
      environment_Fans_OFF();
      environment_Fans_TimeOn = -1;
    }
  #endif

  #ifdef LIGHT_ENABLE
    bool light_underThreshold = light < LIGHT_THRESHOLD;
    // Minimum time between one trigger to another is elapsed. At first init check also if -1 to prevent trigger for the first time after LIGHT_LEDS_MINIMUM_DELAY ms  
    bool light_elapsedMinimumTime = (leds_LastTrigger == -1 || (millis() - leds_LastTrigger) > LIGHT_LEDS_MINIMUM_DELAY); 
    // Minimum time of which the leds must be on
    bool light_elapsedRunningTime = light_isOn && millis() - leds_TimeOn > LIGHT_LEDS_TIME;

    if (light_underThreshold && !light_isOn && light_elapsedMinimumTime) {
      Serial.print("Light level under threshold: ");
      Serial.print(light);
      Serial.println("%. Turning on the LEDs...");

      light_LEDs_ON();
      light_isOn = true;
      leds_TimeOn = millis();
    } else if (!light_underThreshold && light_isOn && light_elapsedRunningTime) {
      light_LEDs_OFF();
      light_isOn = false;
      leds_TimeOn = -1;
      leds_LastTrigger = millis();
    }
  #endif

  #ifdef DATA_ENABLE
    // Store data each DATA_RATE milliseconds. Also -1 condition to store data at first startup
    if ((millis() - data_LastLog >= DATA_RATE) || data_LastLog == -1) {
      data_Store(moisture, humidity, temperature, light);
      data_LastLog = millis();
    }
  #endif

  smartDelay(1);

  #ifdef DEBUG
    // Every second log the data over the serial
    if((counterTime + 1000) < millis()){
      Serial.print("Moisture: ");
      Serial.println(moisture);

      Serial.print("Humidity: ");
      Serial.println(humidity);

      Serial.print("Temperature: ");
      Serial.println(temperature);

      Serial.print("Light: ");
      Serial.println(light);

      counterTime = millis();
    }
  #endif

  #ifdef SLEEP_ENABLE
    sleep_Check();
  #endif
}
