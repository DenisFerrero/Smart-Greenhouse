#include <configuration.h>

void printVariables ();

float moisture_GetLevel();
void moisture_PumpWater_ON();
void moisture_PumpWater_OFF();

float environment_Humidity_GetLevel();
float environment_Temperature_GetLevel();
void environment_Fans_ON();
void environment_Fans_OFF();

float light_GetLevel();
void light_LEDs_ON();
void light_LEDs_OFF();

void data_Store(float moisture, float humidity, float temperature, float light);

void sleep_Check();

void startup ();
void smartDelay(unsigned long ms);
