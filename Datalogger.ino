// PHYS 398 SP20 Group 1: Data Acquisition Software
// 

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_GPS.h>
#include <LiquidCrystal.h>
#include <Keypad.h>


// Set true for serial printing
bool DEBUG = true;


// Set BME sampling
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

// Set GPS interface
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);

// Initialize LCD
const int rs = 12, en = 11, d4 = 36, d5 = 34, d6 = 32, d7 = 30;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Initialize Keypad
const int ROWS = 4;
const int COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3',},
  {'4','5','6',},
  {'7','8','9',},
  {'*','0','#',}
};
byte rowPins[ROWS] = {31,33,35,37}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {2,3,18}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// Initialize BME 680
Adafruit_BME680 bme;

// Set SD chipSelect
const int chipSelect = 53;

// Set GPS PPS interrupt pin
int PPS_pin = 19

float interrupt_time;
bool interrupt_flag = false;

// PPS interrupt
void PPS_ISR () {
    interrupt_time = micros();
    interrupt_flag = true;
}

// SD output files
String sensorDataFile = "sensor_data.csv";
String clockDriftFile = "clock_drift.csv";

void setup() {

    if (DEBUG) {
        // Open serial communications and wait for port to open:
        Serial.begin(9600);
        while (!Serial) {
            ; // wait for serial port to connect. Needed for native USB port only
        }
        // Check BME 680 connection
        if (!bme.begin()) {
            Serial.print("Could not find a valid BME680 sensor, check wiring!");
            while (1);
        }
        // Check SD card connection
        Serial.print("Initializing SD card...");
        if (!SD.begin(chipSelect)) {
            Serial.println("Card failed, or not present");
            while (1);
        }
        Serial.println("SD card initialized.");
    }
    // Check BME/SD card connection
    else if (!SD.begin(chipSelect) || !bme.begin()) {
        while (1);
    }

    // Open GPS communications
    GPS.begin(9600);

    // Initialize BME 680 sensors
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    // Set PPS interrupt
    attachInterrupt(digitalPinToInterrupt(PPS_pin), PPS_ISR, RISING);

    // Record PPS signal timing until keypad "1" pressed
    // Used in post-processing for increasing timing accuracy
    File fileIO = SD.open(clockDriftFile, FILE_WRITE);
    char key = keypad.getKey();
    while (!(key == 1))  {
        // If PPS interrupt flag set, write interrupt time to file
        if (interrupt_flag) {
            fileIO.println(String(interrupt_time) + ',');
        }
        key = keypad.getKey();
    }
    fileIO.close();
    // Remove PPS interrupt
    detachInterrupt(digitalPinToInterrupt(PPS_pin));
}

void loop() {

    // Read current time (convert to seconds)
    String testTime = String(micros() / 1000000.);

    // Check for BME reading
    if (!bme.performReading()) {
        if (DEBUG) {
            Serial.print("Failed to perform BME 680 reading");
        }
        return;
    }

    // Disable interrupts while sampling sensors
    nointerrupts();

    // Sample microphone
    int sample = analogRead(7);
    double volts = (sample * 5.0) / 1024;  // convert to volts

    // BME 680
    // Read temperature (*C)
    String temperature = "Temperature (*C): " + String(bme.temperature);
    // Read pressure (hPa)
    String pressure = "Pressure (hPa): " + String(bme.pressure / 100.);
    // Read humidity (%)
    String humidity = "Humidity (%): " + String(bme.humidity);
    // Read gas resistance (KOhms)
    String gas = "Gas (KOhms): " + String(bme.gas_resistance / 1000.);
    // Read altitude (m)
    String altitude = "Altitude (m): " + String(bme.readAltitude(SEALEVELPRESSURE_HPA));
    // Microphone output (volts)
    String mic_volts = String(volts);

    // Re-enable interrupts
    interrupts();

    // Open SD file
    File fileIO = SD.open(sensorDataFile, FILE_WRITE);

    if (sensorDataFile) {
        // Write to SD
        fileIO.println("Time (s): " + testTime);
        fileIO.println(mic_volts);
        fileIO.println(temperature);
        fileIO.println(pressure);
        fileIO.println(humidity);
        fileIO.println(gas);
        fileIO.println(altitude);
        fileIO.println();
        if (DEBUG) {
            // Print to Serial
            Serial.println("Time (s): " + testTime);
            Serial.println(mic_volts);
            Serial.println(temperature);
            Serial.println(pressure);
            Serial.println(humidity);
            Serial.println(gas);
            Serial.println(altitude);
            Serial.println();
        }
        fileIO.close();
    }
    // Open file failure
    else {
        if (DEBUG) {
            Serial.println("Error opening file: " sensorDataFile);
        }
    }
}
