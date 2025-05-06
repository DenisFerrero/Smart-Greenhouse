#define MOISTURE_PIN 0

void countdown () {
  Serial.print("3");
  delay(1000);
  Serial.print(", 2");
  delay(1000);
  Serial.println(", 1...");
  delay(1000);
}

int read () {
  Serial.println("Reading value...");
  delay(1000);
  int value = analogRead(MOISTURE_PIN);
  Serial.println("Done!");

  return value;
}

void setup() {
  Serial.begin(115200);

  Serial.println("-- Moisture calibration value --");
  Serial.println("Place the sensor in the air to get the MIN level. Test will start in 3 seconds");

  countdown();

  int min = read();

  Serial.println("Now place the sensor in full water to get the MAX level. Test will start in 3 seconds");

  countdown();

  int max = read();

  Serial.println();
  Serial.print("Minimum value detected: ");
  Serial.println(min);
  Serial.print("Maximum value detected: ");
  Serial.println(max);

  Serial.println("Quitting...");
  while(1) delay(1000);
}

void loop() { }
