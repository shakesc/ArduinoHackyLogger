/*
xxxxxxxxxxx
*/

int debug = 1;  // set to zero if serial print not required
long loop_counter = 0;

#include <Arduino.h>
#include <millisDelay.h>
#include <loopTimer.h>  // check speed of main loop for debug
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Display library and class declaration
#include <SSD1306AsciiAvrI2c.h>
SSD1306AsciiAvrI2c display;

//declare SSD1306 OLED display variables
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET 4
#define OLED_I2C_ADDRESS 0x3C

// ADC library and class declaration
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
const int ADCintPin = 2;
int16_t adc0, adc1, adc2, adc3;
int adc0Normalised, adc0Offset;
float adc0Adjusted;


// The ADC module sends a interrupt signal to the Arduino when a conversion is completed.
// This way, we will read the ADC only if it is ready.
// We need to connect ALERT/RDY pin on the ADC1115 to pin ADCintPin on the Arduino.
volatile bool adcNewData = false;
void newDataReady() {
  adcNewData = true;
}

// WCS1700
#define MODEL 2        //WCS1700
#define SENSOR_PIN A0  //pin for reading sensor

#define ZERO_CURRENT_WAIT_TIME 5000  //wait for 5 seconds to allow zero current measurement
#define READ_DELAY 2

#define MEASUREMENT_ITERATION 4   // number of sensor readings to include in the instantaneous value
#define VOLTAGE_REFERENCE 5000.0  //5000mv is for 5V

// ==============================================================================================================
// WCS Sensitivity constants
float sensitivity[6] = {
  11.0,  //WCS1500
  22.0,  //WCS1600
  33.0,  //WCS1700
  66.0,  //WCS1800
  70.0   //WCS2800
};       // mV/A

int sensor_model = MODEL;

float span = 0.002312;
// 33mV/A and a bi-directional 70A sensor (140A range) is 0.033volt*140= 4.62volt span on a 5volt supply.
// 4.62/5 of the range of the ADC1115 A/D (65535), or ~60554 A/D values for 140Amp.  140/60554 = 0.002312
// https://forum.arduino.cc/t/wcs1700-noise-problem/546967/10

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;
// Setup a oneWire instance to communicate with any OneWire devices
// Pass our oneWire reference to Dallas Temperature sensor
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

// ==============================================================================================================
// Current sensor variables
float sensorCurrent = 0;
float current_zero = 0;  // current zero adjustment
float current_inst = 0;
float current_peak = 0;
float current_average = 0;
int AvCurrentIndex = 0;    // the index of the current reading
float AvCurrentTotal = 0;  // the running total

// array for averaging
const int AvCurrentNumReadings = 10;
float AvCurrentReadings[AvCurrentNumReadings];  // the readings from the analog input

// ==============================================================================================================
// temperature measurement variables
float sensorTemperature = 0;
float temp_inst = 0;
float temp_peak = 0;
float temp_average = 0;
int AvTempIndex = 0;  // the index of the temp reading
float AvTempTotal = 0;

// array for averaging
const int AvTemperatureNumReadings = 10;
float AvTempReadings[AvTemperatureNumReadings];  // the readings from the analog input

// ==============================================================================================================
// Indicator LED i/o
const int ledWarningPin = 7;
const int ledDangerPin = 8;

// ==============================================================================================================
// Setup encoder
// https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
// Rotary Encoder Inputs
#define encoderCLK 4
#define encoderDT 3
int encoderCounter = 0;
int currentStateCLK;
int lastStateCLK;
const int EncoderCounterMax = 2;  // number of use cases for display

// ==============================================================================================================
// Setup task loops
#define TEMPERATURE_READING_DELAY 1000  // milliseconds
millisDelay tempDelay;

// ==============================================================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Hacky Monitor");

  sensors.begin();  // Start up the library for temp measurement

  // setup external ADC
  ads.begin();
  ads.setGain(GAIN_TWOTHIRDS);                                 // +/- 6.144V  1 bit = 0.1875mV (default)
  ads.setDataRate(RATE_ADS1115_32SPS);                         //< 32 samples per second
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);  // set adc reading to continuous mode

  // Indicator LEDs
  pinMode(ledWarningPin, OUTPUT);
  pinMode(ledDangerPin, OUTPUT);
  digitalWrite(ledWarningPin, HIGH);
  digitalWrite(ledDangerPin, HIGH);

  //setup the display
  display.begin(&Adafruit128x64, OLED_I2C_ADDRESS, OLED_RESET);
  display.setFont(System5x7);
  display.clear();
  display.set2X();
  displaytext(1, "Hacky");
  displaytext(3, "Monitor");
  display.set1X();
  displaytext(6, "Reading Zero");

  delay(ZERO_CURRENT_WAIT_TIME);
  setZeroCurrent();

  readTemperatureSensor();

  // Initialize current average array using first reading. This means average normalizes quicker
  for (int thisReading = 0; thisReading < AvCurrentNumReadings; thisReading++) {
    AvCurrentTotal += current_inst;
    AvCurrentReadings[thisReading] = current_inst;
  }

  // Initialize temperature average array using first reading. This means average normalizes quicker
  for (int thisReading = 0; thisReading < AvTemperatureNumReadings; thisReading++) {
    AvTempTotal += sensorTemperature;
    AvTempReadings[thisReading] = sensorTemperature;
  }

  digitalWrite(ledWarningPin, LOW);
  digitalWrite(ledDangerPin, LOW);

  // The convention is ready on the falling edge of a pulse at the ALERT/RDY pin.
  attachInterrupt(digitalPinToInterrupt(ADCintPin), newDataReady, FALLING);

  // Setup decoder
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT);
  lastStateCLK = digitalRead(encoderCLK);  // Read the initial state of CLK

  // Call updateEncoder() when any high/low changed seen on interrupt 1 (pin 3)
  // attachInterrupt(1, updateEncoder, CHANGE);

  // Kick off the timers for reading sensors
  tempDelay.start(TEMPERATURE_READING_DELAY);
}


void loop() {
  // loopTimer.check(Serial);

  if (adcNewData = true) {
    loop_counter += 1;
    Serial.println((String) "LOOP " + loop_counter);
    current_peak = (current_inst > current_peak) ? current_inst : current_peak;
    current_average = averageCurrent();

    if (tempDelay.justFinished()) {
      readTemperatureSensor();
      temp_inst = sensorTemperature;
      temp_average = averageTemperature();
      temp_peak = (temp_inst > temp_peak) ? temp_inst : temp_peak;
      tempDelay.repeat();  // Start the timer again without drift
      // Serial.println("TEMP DELAY");
    }


    if (encoderCounter == 0) {
      display.clear();
      display.set1X();
      displayline(current_inst, 1, " Inst");
      displayline(current_average, 2, " Avg");
      displayline(current_peak, 3, " Peak");
      displayline(temp_average, 5, " Temp");
    }
    if (encoderCounter == 1) {
      display.clear();

      display.set1X();
      displaytext(1, "AMP");
      displaytext(4, "TEMP");
      display.set2X();
      displayline(current_inst, 2, "");
      displayline(temp_inst, 5, "");
      display.set1X();
    }
    if (encoderCounter == 2) {
      display.clear();

      display.set1X();
      displaytext(1, "AMP PEAK");
      displaytext(4, "TEMP PEAK");
      display.set2X();
      displayline(current_peak, 2, "");
      displayline(temp_peak, 5, "");
      display.set1X();
    }

    if (debug == 1) {
      Serial.println((String) "CURRENT INST: " + current_inst);
      Serial.println((String) "CURRENT AVG: " + current_average);
      Serial.println((String) "CURRENT RAW: " + adc0);
      Serial.println((String) "CURRENT NORM: " + current_inst);
      Serial.println((String) "TEMP INST: " + sensorTemperature);
      Serial.println((String) "TEMP AVG: " + temp_average);
      Serial.println("==============================");
    }
    adcNewData = false;
  }
}  

  void updateEncoder() {
    // Read the current state of CLK
    currentStateCLK = digitalRead(encoderCLK);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {

      // If the DT state is different than the CLK state then
      // the encoder is rotating CCW so decrement
      if (digitalRead(encoderDT) != currentStateCLK) {
        encoderCounter--;
      } else {
        // Encoder is rotating CW so increment
        encoderCounter++;
      }

      if (encoderCounter < 0) {
        encoderCounter = EncoderCounterMax;
      }
      if (encoderCounter > EncoderCounterMax) {
        encoderCounter = 0;
      }
    }
    // Remember last CLK state
    lastStateCLK = currentStateCLK;
  }

  float averageCurrent() {
    float current_average = 0;
    // subtract the last reading and advance array position:
    AvCurrentTotal = AvCurrentTotal - AvCurrentReadings[AvCurrentIndex];
    AvCurrentReadings[AvCurrentIndex] = current_inst;
    AvCurrentTotal += AvCurrentReadings[AvCurrentIndex];
    AvCurrentIndex += 1;

    if (AvCurrentIndex >= AvCurrentNumReadings) {
      AvCurrentIndex = 0;
    }
    current_average = AvCurrentTotal / AvCurrentNumReadings;
    return current_average;
  }

  float averageTemperature() {
    float temperature_average = 0;
    // subtract the last reading and advance array position:
    AvTempTotal = AvTempTotal - AvTempReadings[AvTempIndex];
    AvTempReadings[AvTempIndex] = sensorTemperature;
    AvTempTotal += AvTempReadings[AvTempIndex];
    AvTempIndex += 1;

    if (AvTempIndex >= AvTemperatureNumReadings) {
      AvTempIndex = 0;
    }
    temperature_average = AvTempTotal / AvTemperatureNumReadings;
    return temperature_average;
  }


  void setZeroCurrent() {
    int iteration = 100;
    long current_accumulator = 0;

    for (int i = 0; i < iteration; i++) {
      current_accumulator += ads.readADC_SingleEnded(0);
      delay(READ_DELAY);
    }
    adc0Offset = current_accumulator / iteration;  // Use this for calcs as quicker and less error than float
    current_zero = adc0Offset * span;              // correct for center value
    current_inst = current_zero;
  }

  void readExternalADC() {
    adc0 = ads.readADC_SingleEnded(0);
    adc0Normalised = adc0 - adc0Offset;    // remove voltage current
    current_inst = adc0Normalised * span;  // instantaneous current, adjusted for sensor
    adcNewData = true;
  }

  void readTemperatureSensor() {
    sensorTemperature += sensors.getTempCByIndex(0);
  }


  /****************************************************************************/
  /*  I : Value measured to display                                           */
  /*		Buffer holding the last saved measurment							*/
  /*		Line number at which display the value								*/
  /*		End of line (unit) to append to the line							*/
  /*  P : Format and display a measurment at the right line, only if changed	*/
  /*  O : /                                                                   */
  /****************************************************************************/
  void displayline(const float measurment, const uint8_t line_num, const char line_end[]) {
    char floatbuf[16] = { 0 };

    //format the line ([-]xxxxx.xxx [unit])
    dtostrf(measurment, 10, 3, floatbuf);
    strcat(floatbuf, line_end);

    //place the cursor and write the line
    display.setCursor(0, line_num);
    display.print(floatbuf);
  }

  void displaytext(const uint8_t line_num, const char line_end[]) {
    char floatbuf[16] = { 0 };

    strcat(floatbuf, line_end);

    //place the cursor and write the line
    display.setCursor(0, line_num);
    display.print(floatbuf);
  }
