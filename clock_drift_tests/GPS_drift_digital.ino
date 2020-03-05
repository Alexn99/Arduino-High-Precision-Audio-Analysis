#include <Adafruit_GPS.h>

int PPS_1 = 18;
int PPS_2 = 19;

#define GPSSerial Serial2

long int i;

Adafruit_GPS GPS(&GPSSerial);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  GPS.begin(9600);
  i = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(18)) {
      Serial.println("read 1: " + String(i));
  }
  if (digitalRead(19)) {
      Serial.println("read 2: " + String(i));
  }
  i++;
}
