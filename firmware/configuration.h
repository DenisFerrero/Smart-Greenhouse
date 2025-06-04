// Enable debug logs
#define DEBUG

// Store data periodically
#define DATA_ENABLE

#ifdef DATA_ENABLE
  // Store data each X milliseconds. Default 10*1000 (10 seconds)
  #define DATA_RATE 24*60*60*1000
  // SIM APN
  #define DATA_STORE_SIM_APN "iliad"
  // SIM PIN
  #define DATA_STORE_SIM_PIN "1234"
  // Phone numbers to send the message, multiple numbers separated by comma
  #define DATA_STORE_SIM_TARGET_NUMBERS "+393925423403,+393311261809,+393457998194,+393429724636,+393505435439"
#endif

// Enable terrain moisture check
#define MOISTURE_ENABLE

#ifdef MOISTURE_ENABLE
  // Sensor pin
  #define MOISTURE_SENSOR_PIN 4
  // Moisture level (in percentage) on which the pump must be enabled
  #define MOISTURE_THRESHOLD 30
  // Min/Max moisture levels. Min = Level in air, Max = Level in full water
  #define MOISTURE_LEVEL_MIN 615
  #define MOISTURE_LEVEL_MAX 390
  // Water pump relay pin
  #define MOISTURE_WATER_PUMP_PIN 32
  // Water pumping time in milliseconds
  #define MOISTURE_WATER_PUMP_TIME 5000
  // After pumping wait X milliseconds to the next one.
  // Let the terrain absorb the water and prevent over-water situation. Default 5*60*1000 (5 minutes)
  #define MOISTURE_WATER_PUMP_MINIMUM_DELAY 5*60*1000
#endif


// Enable environment check
#define ENVIRONMENT_ENABLE

#ifdef ENVIRONMENT_ENABLE
  // Sensor pin
  #define ENVIRONMENT_SENSOR_PIN 15
  // Fans relay pin
  #define ENVIRONMENT_FANS_PIN 33
  // To prevent to continually turning on and off the fans when trying to reduce the humidity
  // keep them up for a minimum time (in milliseconds). Default 30*1000 (30 seconds)
  #define ENVIRONMENT_FANS_TIME 30*1000

  // Enable/Disable humidity level
  #define ENVIRONMENT_HUMIDITY_ENABLE

  #ifdef ENVIRONMENT_HUMIDITY_ENABLE
    // Humidity level (in percentage) on which the fans must be enabled
    #define ENVIRONMENT_HUMIDITY_THRESHOLD 25
  #endif

  // Enable/Disable temperature level
  #define ENVIRONMENT_TEMPERATURE_ENABLE

  #ifdef ENVIRONMENT_TEMPERATURE_ENABLE
  // Temperature level (in °C) on which the fans must be enabled
    #define ENVIRONMENT_TEMPERATURE_THRESHOLD 25
  #endif
#endif

// Enable light control
#define LIGHT_ENABLE

#ifdef LIGHT_ENABLE
  // Sensor pin
  #define LIGHT_SENSOR_PIN 36
  // Light level on which the led must be enabled
  #define LIGHT_THRESHOLD 25
  // Min/Max light levels. Min = Level in the dark, Max = Level in full light
  #define LIGHT_LEVEL_MIN 4000
  #define LIGHT_LEVEL_MAX 2000
  // LEDs relay pin
  #define LIGHT_LEDS_PIN 21
  // Keep the leds on for a minimum time (in milliseconds). Default 2*1000 (2 seconds)
  #define LIGHT_LEDS_TIME 2*1000
#endif

// Enable sleep mode. Turn off the device at target hour and back on at target hour
#define SLEEP_ENABLE

#ifdef SLEEP_ENABLE
  // Sleep starts at
  #define SLEEP_START_TIME 23
  // Sleep ends at
  #define SLEEP_END_TIME 5
  // When gathering the time using GPS it's in in UTC, so shift it using an integer
  #define SLEEP_TIMEZONE_OFFSET 2
#endif