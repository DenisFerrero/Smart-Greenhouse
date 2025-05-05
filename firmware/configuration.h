// Print those variables at startup
#define START_UP_PRINT_ENV 1

// Store data periodically
#define DATA_ENABLE
// Store data each X milliseconds. Default 10*1000 (10 seconds)
#define DATA_RATE 10*1000
// 1. Save data on SD
#define DATA_STORE_MODE 1
// Data filename. Default data
#define DATA_FILENAME "data"

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

// Enable humidity environment check
#define HUMIDITY_ENABLE
// Sensor pin
#define HUMIDITY_SENSOR_PIN 0
// Humidity level (in percentage) on which the fans must be enabled
#define HUMIDITY_THRESHOLD 30
// Fans relay pin
#define HUMIDITY_FANS_PIN 0
// To prevent to continually turning on and off the fans when trying to reduce the humidity
// keep them up for a minimum time (in milliseconds). Default 30*1000 (30 seconds)
#define HUMIDITY_FANS_MINIMUM_TIME 30*1000

// Enable light control
#define LIGHT_CONTROL_ENABLE
// Sensor pin
#define LIGHT_CONTROL_SENSOR_PIN 0
// Light level on which the led must be enabled
#define LIGHT_CONTROL_THRESHOLD 3000
// LEDs relay pin
#define LIGHT_CONTROL_LEDS_PIN 0

// Enable RTC. The module is used to disable the services during the night period to let the plants rest
#define RTC_ENABLE
// SDA and SCL pin
#define RTC_SDA_PIN 0
#define RTC_SCL_PIN 0
// RTC interrupt pin
#define RTC_INTERRUPT_PIN 0
// Set the time at which the modules must go offline
#define RTC_SLEEP_START 21
// Set the time at which the modules must go back online
#define RTC_SLEEP_END 5
