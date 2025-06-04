#include <configuration.h>

// Data enable
#ifdef DATA_ENABLE
  // Data rate
  #ifndef DATA_RATE
    #define DATA_RATE 10*1000
  #endif
  // Error if SIM APN is not defined
  #ifndef DATA_STORE_SIM_APN
    #error "Missing SIM APN"
  #endif
  // Set SIM pin if not already
  #ifndef DATA_STORE_SIM_PIN
    #define DATA_STORE_SIM_PIN ""
  #endif
  // Error if no phone number is provided
  #ifndef DATA_STORE_SIM_TARGET_NUMBERS
    #error "Missing phone numbers"
  #endif
#endif

#ifdef MOISTURE_ENABLE
  #ifndef MOISTURE_SENSOR_PIN
    #error "Moisture sensor is not defined!"
  #endif

  #ifndef MOISTURE_THRESHOLD
    #error "Moisture threshold is not defined!"
  #endif

  #ifndef MOISTURE_LEVEL_MIN
    #error "Moisture min level is not defined!"
  #endif

  #ifndef MOISTURE_LEVEL_MAX
    #error "Moisture max level is not defined!"
  #endif
  
  #ifndef MOISTURE_WATER_PUMP_PIN
    #error "Moisture water pump pin is not defined!"
  #endif

  #ifndef MOISTURE_WATER_PUMP_TIME
    #error "Moisture water pump time is not defined!"
  #endif

  #ifndef MOISTURE_WATER_PUMP_MINIMUM_DELAY
    #define MOISTURE_WATER_PUMP_MINIMUM_DELAY 5*60*1000
  #endif
#endif

#ifdef ENVIRONMENT_ENABLE
  #ifndef ENVIRONMENT_HUMIDITY_ENABLE
    #ifndef ENVIRONMENT_TEMPERATURE_ENABLE
      #error "Either humidity or temperature must be enable for the environment"
    #endif
  #endif


  #ifndef ENVIRONMENT_SENSOR_PIN
    #error "Environment sensor is not defined!"
  #endif
  
  #ifndef ENVIRONMENT_FANS_PIN
    #error "Environment fans pin is not defined!"
  #endif
  
  #ifndef ENVIRONMENT_FANS_TIME
    #define ENVIRONMENT_FANS_TIME 30*1000
  #endif

  #ifdef ENVIRONMENT_HUMIDITY_ENABLE
    #ifndef ENVIRONMENT_HUMIDITY_THRESHOLD
      #error "Humidity threshold is not defined!"
    #endif
  #endif

  #ifdef ENVIRONMENT_TEMPERATURE_ENABLE
    #ifndef ENVIRONMENT_TEMPERATURE_THRESHOLD
      #error "Temperature threshold is not defined!"
    #endif
  #endif
#endif

#ifdef LIGHT_ENABLE
  #ifndef LIGHT_SENSOR_PIN
    #error "Light sensor is not defined!"
  #endif
  
  #ifndef LIGHT_THRESHOLD
    #error "Light threshold is not defined!"
  #endif

  #ifndef LIGHT_LEVEL_MIN
    #error "Light level min is not defined!"
  #endif

  #ifndef LIGHT_LEVEL_MAX
    #error "Light level max is not defined!"
  #endif

  #ifndef LIGHT_LEDS_PIN
    #error "LEDs pin is not defined!"
  #endif

  #ifndef LIGHT_LEDS_TIME
    #define LIGHT_LEDS_TIME 2*1000
  #endif
#endif

#ifdef SLEEP_ENABLE
  #ifndef SLEEP_START_TIME
    #error "Sleep start time is not defined!"
  #endif
  #ifndef SLEEP_END_TIME
    #error "Sleep end time is not defined!"
  #endif
  #ifndef SLEEP_TIMEZONE_OFFSET
    #define SLEEP_TIMEZONE_OFFSET 0
  #endif
#endif
