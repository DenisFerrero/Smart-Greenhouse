#include <configuration.h>

void printVariables ();

int moisture_GetLevel();
void moisture_PumpWater_ON();
void moisture_PumpWater_OFF();

float humidity_GetLevel();
void humidity_Fans_ON();
void humidity_Fans_OFF();

float lightControl_GetLevel();
void lightControl_LEDs_ON();
void lightControl_LEDs_OFF();

void data_Store(int moisture, int humidity, float light);

void startup ();