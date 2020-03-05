
#include <Wire.h>
#include <Adafruit_GPS.h>

int PPS_1 = 18;
int PPS_2 = 19;

float interrupt_time_1;
float interrupt_time_2;
bool interrupt_flag_1 = false;
bool interrupt_flag_2 = false;

#define GPSSerial Serial2

Adafruit_GPS GPS(&GPSSerial);

void setup() {

    attachInterrupt(digitalPinToInterrupt(PPS_1), ISR_1, RISING);
    attachInterrupt(digitalPinToInterrupt(PPS_2), ISR_2, RISING);

    Serial.begin(9600);

    GPS.begin(9600);
}

void loop() {
    if (interrupt_flag_1) {
        Serial.println("PPS1 micros: " + String(interrupt_time_1) + ",");
        interrupt_flag_1 = false;
    }
    if (interrupt_flag_2) {
        Serial.println("PPS2 micros: " + String(interrupt_time_2) + ",");
        interrupt_flag_2 = false;
    }
}



void ISR_1 () {
    interrupt_time_1 = micros();
    interrupt_flag_1 = true;
}


void ISR_2 () {
    interrupt_time_1 = micros();
    interrupt_flag_1 = true;
}
