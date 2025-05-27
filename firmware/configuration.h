// Print those variables at startup
#define START_UP_PRINT_ENV 1

// Store data periodically
#define DATA_ENABLE
// Store data each X milliseconds. Default 10*1000 (10 seconds)
#define DATA_RATE 10*1000
// SIM APN
#define DATA_STORE_SIM_APN "iliad"
// SIM PIN
#define DATA_STORE_SIM_PIN "1234"
// Phone numbers to send the message, multiple numbers separated by comma
#define DATA_STORE_SIM_TARGET_NUMBERS "+393925423403"

// Enable terrain moisture check
#define MOISTURE_ENABLE
// Sensor pin
#define MOISTURE_SENSOR_PIN 0
// Moisture level (in percentage) on which the pump must be enabled
#define MOISTURE_THRESHOLD 30
// Min/Max moisture levels. Min = Level in air, Max = Level in full water
#define MOISTURE_LEVEL_MIN 2000
#define MOISTURE_LEVEL_MAX 1000
// Water pump relay pin
#define MOISTURE_WATER_PUMP_PIN 0
// Water pumping time in milliseconds
#define MOISTURE_WATER_PUMP_TIME 5000
// After pumping wait X milliseconds to the next one.
// Let the terrain absorb the water and prevent over-water situation. Default 5*60*1000 (5 minutes)
#define MOISTURE_WATER_PUMP_MINIMUM_DELAY 5*60*1000

// Enable environment check
#define ENVIRONMENT_ENABLE
// Sensor pin
#define ENVIRONMENT_SENSOR_PIN 0
// Fans relay pin
#define ENVIRONMENT_FANS_PIN 0
// To prevent to continually turning on and off the fans when trying to reduce the humidity
// keep them up for a minimum time (in milliseconds). Default 30*1000 (30 seconds)
#define ENVIRONMENT_FANS_TIME 30*1000
// Enable/Disable humidity level
#define ENVIRONMENT_HUMIDITY_ENABLE
// Humidity level (in percentage) on which the fans must be enabled
#define ENVIRONMENT_HUMIDITY_THRESHOLD 30
// Enable/Disable temperature level
#define ENVIRONMENT_TEMPERATURE_ENABLE
// Temperature level (in °C) on which the fans must be enabled
#define ENVIRONMENT_TEMPERATURE_THRESHOLD 25

// Enable light control
#define LIGHT_ENABLE
// Sensor pin
#define LIGHT_SENSOR_PIN 0
// Light level on which the led must be enabled
#define LIGHT_THRESHOLD 30
// Min/Max light levels. Min = Level in the dark, Max = Level in full light
#define LIGHT_LEVEL_MIN 2000
#define LIGHT_LEVEL_MAX 1000
// LEDs relay pin
#define LIGHT_LEDS_PIN 0