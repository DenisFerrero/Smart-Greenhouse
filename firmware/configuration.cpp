#include <configuration.h>

// Data enable
#ifdef DATA_ENABLE
  // Data store mode
  #ifndef DATA_STORE_MODE
    #define DATA_STORE_MODE 1
  #endif
  
  #if DATA_STORE_MODE != 1
    #error "Invalid data store mode selected. Available solutions: 1. SD Card"
  #endif

  // Data rate
  #ifndef DATA_RATE
    #define DATA_RATE 10*1000
  #endif

  // Data filename
  #ifndef DATA_FILENAME
    #define DATA_FILENAME "data"
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

#ifdef HUMIDITY_ENABLE
  #ifndef HUMIDITY_SENSOR_PIN
    #error "Humidity sensor is not defined!"
  #endif
  
  #ifndef HUMIDITY_THRESHOLD
    #error "Humidity threshold is not defined!"
  #endif

  #ifndef HUMIDITY_FANS_PIN
    #error "Humidity fans pin is not defined!"
  #endif

  #ifndef HUMIDITY_FANS_MINIMUM_TIME
    #define HUMIDITY_FANS_MINIMUM_TIME 30*1000
  #endif
#endif

#ifdef LIGHT_CONTROL_ENABLE
  #ifndef LIGHT_CONTROL_SENSOR_PIN
    #error "Light sensor is not defined!"
  #endif
  
  #ifndef LIGHT_CONTROL_THRESHOLD
    #error "Light threshold is not defined!"
  #endif

  #ifndef LIGHT_CONTROL_LEDS_PIN
    #error "LEDs pin is not defined!"
  #endif
#endif

#ifdef RTC_ENABLE
  #ifndef RTC_INTERRUPT_PIN
    #error "RTC interrupt pin is not defined!";
  #endif

  #ifndef RTC_SLEEP_START
    #define RTC_SLEEP_START 21
  #endif
  
  #ifndef RTC_SLEEP_END
    #define RTC_SLEEP_END 5
  #endif
#endif

