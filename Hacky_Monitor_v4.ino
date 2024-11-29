/*
Hacky Monitor v4
*/

// https://learn.adafruit.com/thermistor/using-a-thermistor

int debug = 1;           // set to zero if serial print not required
int debug_hi = 1;        // set to zero if serial print not required
int debug_throttle = 0;  // set to zero if serial print not required
int debug_therm = 0;     // set to zero if serial print not required
int debug_batt = 0;
char monitor_version[] = "Hacky 1.03";

#include <Arduino.h>
#include <millisDelay.h>
#include <loopTimer.h>  // check speed of main loop for debug
#include <Wire.h>
#include <OneWire.h>
#include <SoftwareSerial.h>

// ==============================================================================================================
// MCP4728 4-Channel 12-bit I2C DAC
#include <Adafruit_MCP4728.h>
Adafruit_MCP4728 mcp;

// ==============================================================================================================
// Display library and class declaration
#include <SSD1306AsciiAvrI2c.h>
SSD1306AsciiAvrI2c display;

//declare SSD1306 OLED display variables
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET 4
#define OLED_I2C_ADDRESS 0x3C

// ==============================================================================================================
// ADC library and class declaration
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
const int ADCintPin = 2;

uint16_t adc0, adc1, adc2, adc3;
float adc0_volts, adc1_volts, adc2_volts, adc3_volts;
float adc_current, adc_steer, adc_tcouple, adc_throttle;

float adc_current_average, adc_steer_average, adc_tcouple_average, adc_throttle_average;

// array for averaging
const int AvADCReadings = 10;
float adc0_Readings[AvADCReadings];  // the readings from the analog input
float adc1_Readings[AvADCReadings];  // the readings from the analog input
float adc2_Readings[AvADCReadings];  // the readings from the analog input
float adc3_Readings[AvADCReadings];  // the readings from the analog input

int adc_index = 0;     // the index of the current reading
float adc0_total = 0;  // the running total
float adc1_total = 0;  // the running total
float adc2_total = 0;  // the running total
float adc3_total = 0;  // the running total

float adc0_average = 0;  // the running total
float adc1_average = 0;  // the running total
float adc2_average = 0;  // the running total
float adc3_average = 0;  // the running total

#define ZERO_CURRENT_WAIT_TIME 3000  //wait for 3 seconds to allow zero current measurement
#define READ_DELAY 50                // 20 ms between rearings

// ==============================================================================================================
// WCS Sensitivity constants
/* mV/A
  11.0,  //WCS1500
  22.0,  //WCS1600
  33.0,  //WCS1700
  66.0,  //WCS1800
  70.0   //WCS2800
*/

/*
33mV/A and a bi-directional 70A sensor (140A range) is 0.033volt*140= 4.62volt span on a 5volt supply.
ADC1115 A/D (65535) on 2/3x gain +/- 6.144V, 1 bit = 0.1875mV
ADC1115 A/D (65535) on 1x gain +/- 4.096V  1 bit = 0.125mV
Number of bits per Amp = 33/0.125 = 264 bits per Amp
Current calc = (ADC Value - ADC Zero) / 264
*/
uint16_t current_resolution = 264;

// Current sensor variables
float current_inst = 0;
float current_peak = 0;
float current_average = 0;
float current_fuse_loss = 0;  // current when fuse power went
uint16_t adc0_normalised;

float sensorCurrent = 0;
uint16_t current_zero = 0;

// voltage reading from fuse 3.2v x 12S minimum to 4.2v x 12S minimum
// use voltage divider of 10K and 100K
// =(12S x Nominal x divider) * 65535 / VDD. 3.2 nominal = 45755 , 4.2 nominal = 60053
const float low_voltage_threshold = 40;  // Volts

// ==============================================================================================================
// temperature measurement variables
float temp_inst = 0;
float temp_peak = 0;
float temp_average = 0;
float temp_fuse_loss = 0;  // temperature when fuse power went
// int AvTempIndex = 0;       // the index of the temp reading
// float AvTempTotal = 0;

// array for averaging
const int AvTemperatureNumReadings = 10;
float AvTempReadings[AvTemperatureNumReadings];  // the readings from the a

// ==============================================================================================================
// Indicator LED i/o
const int ledDangerPin = 7;
const int ledWarningPin = 8;
const float current_warning_limit = 40;      // turn on LED if over threshold
const float temperature_warning_limit = 50;  // turn on LED if over threshold
uint16_t vbatt_reading = 0;

// ==============================================================================================================
// Setup encoder
// https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
// Rotary Encoder Inputs
#define encoderCLK 3  // uses interrupt
#define encoderDT 4
int encoderCounter = 0;
int currentStateCLK;
int lastStateCLK;
const int EncoderCounterMax = 6;  // number of use cases for display


// ==============================================================================================================
#define FULL_VREF_RAW_VALUE 4095;
unsigned int supply_reading;

// ==============================================================================================================
unsigned int dac_throttle, dac_throttle_short, dac_throttle_left, dac_throttle_right = 0;
#define throttleLower 10;
#define throttleUpper 4000;

// ==============================================================================================================
// https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/
// https://learn.adafruit.com/thermistor/using-a-thermistor
// https://circuitdigest.com/microcontroller-projects/interfacing-Thermistor-with-arduino

// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

float ThermVolts = 0;
float logThermVolts;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


// ==============================================================================================================
#define DISPLAY_UPDATE_DELAY 2000  // milliseconds
millisDelay displayDelay;

#define ADC_UPDATE_DELAY 100  // milliseconds
millisDelay adcDelay;

// ==============================================================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(monitor_version);

  // Indicator LEDs
  pinMode(ledWarningPin, OUTPUT);
  pinMode(ledDangerPin, OUTPUT);
  digitalWrite(ledWarningPin, HIGH);  // Switch LEDs on to indicate startup of monitor
  digitalWrite(ledDangerPin, HIGH);

  if (debug == 1) {
    Serial.println("LEDS on");
  }

  // setup external ADC
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }


  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip");
    while (1)
      ;
  }

  Serial.println("Set initial DAC values");
  // Set channel initial values
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  mcp.saveToEEPROM();

  //setup the display
  display.begin(&Adafruit128x64, OLED_I2C_ADDRESS, OLED_RESET);
  display.setFont(System5x7);
  display.clear();
  display.set2X();
  displaytext(1, monitor_version);
  displaytext(3, "Monitor");
  display.set1X();
  displaytext(6, "Reading Zero");

  if (debug == 1) {
    Serial.println("Display on");
  }

  // ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 0.125mV
  ads.setGain(GAIN_TWOTHIRDS);           // +/- 6.144V  1 bit = 0.1875mV (default)
  ads.setDataRate(RATE_ADS1115_250SPS);  //< 32 samples per second
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);  // set adc reading to continuous mode

  delay(ZERO_CURRENT_WAIT_TIME);
  //***************************************

  initialise_adc_arrays();

  // Switch on output for Thermistor
  pinMode(A0, INPUT);
  delay(10);
  supply_reading = analogRead(A0);  // read the input pin
  Serial.println(supply_reading);   // debug value

  // Switch on output for Thermistor
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);

  pinMode(ADCintPin, INPUT_PULLUP);
  // The convention is ready on the falling edge of a pulse at the ALERT/RDY pin.
  // attachInterrupt(ADCintPin, newDataReady, FALLING);

  // Setup vbat input
  // pinMode(A7, INPUT);  // set pull-up on analog pin 7
  vbatt_reading = calc_vbatt();

  // Setup decoder
  pinMode(encoderCLK, INPUT_PULLUP);
  pinMode(encoderDT, INPUT_PULLUP);
  lastStateCLK = digitalRead(encoderCLK);  // Read the initial state of CLK
  attachInterrupt(digitalPinToInterrupt(encoderCLK), updateEncoder, CHANGE);

  // Kick off the timers for reading sensors

  displayDelay.start(DISPLAY_UPDATE_DELAY);
  // adcDelay.start(ADC_UPDATE_DELAY);

  // Determine the quiescent current for substration later
  setZeroCurrent();
  Serial.println((String) "CURRENT ZERO: " + current_zero);

  // Turn LEDs off to indicate ready
  digitalWrite(ledWarningPin, LOW);
  digitalWrite(ledDangerPin, LOW);
}

//=========================================================================================

void loop() {
  // loopTimer.check(Serial);

  if (ads.conversionComplete() == true) {
    ads.getLastConversionResults();
    read_adc_values();
    convert_thermistor();

    current_inst = (adc_current_average - current_zero) / current_resolution;  // remove quiescent current & adjust for sensor range
    current_peak = (current_inst > current_peak) ? current_inst : current_peak;

    temp_peak = (temp_inst > temp_peak) ? temp_inst : temp_peak;
  }
  throttle_management();
  //  adcDelay.repeat();
  vbatt_reading = calc_vbatt();

  if (vbatt_reading < low_voltage_threshold) {  // record values when fuse lost
    current_fuse_loss = current_inst;
    temp_fuse_loss = temp_inst;
  }

  if (current_inst > current_warning_limit) {
    digitalWrite(ledWarningPin, HIGH);  // Switch LEDs on to indicate startup of monitor
  }
  if (temp_inst > temperature_warning_limit) {
    digitalWrite(ledDangerPin, HIGH);  // Switch LEDs on to indicate startup of monitor
  }

  if (displayDelay.justFinished()) {
    updateDisplay();
  }
}
// end of loop

void updateDisplay() {
  display.clear();
  switch (encoderCounter) {
    case 1:
      display.set1X();
      displayline(current_inst, 1, " Inst");
      displayline(current_average, 2, " Avg");
      displayline(current_peak, 3, " Peak");
      displayline(vbatt_reading, 4, " Volt");
      displayline(temp_average, 5, " Temp");
      break;

    case 0:
      display.set1X();
      displaytext(1, "AMP");
      displaytext(4, "TEMP");
      display.set2X();
      displayline(current_inst, 2, "");
      displayline(temp_inst, 5, "");
      display.set1X();
      break;

    case 2:
      display.set1X();
      displaytext(1, "AMP PEAK");
      displaytext(4, "TEMP PEAK");
      display.set2X();
      displayline(current_peak, 2, "");
      displayline(temp_peak, 5, "");
      display.set1X();
      break;

    case 3:
      display.set1X();
      displaytext(1, "AMP BLOW");
      displaytext(4, "TEMP BLOW");
      display.set2X();
      displayline(current_fuse_loss, 2, "");
      displayline(temp_fuse_loss, 5, "");
      display.set1X();
      break;

    case 4:
      display.set1X();
      displayline(adc0_volts, 1, " ADC0");
      displayline(adc1_volts, 2, " ADC1");
      displayline(adc2_volts, 3, " ADC2");
      displayline(adc3_volts, 4, " ADC3");
      displayline(dac_throttle_left, 5, " DACl");
      displayline(dac_throttle_right, 6, " DACr");
      break;

    case 5:
      display.set1X();
      displaytext(1, "VOLT");
      display.set2X();
      displayline(vbatt_reading, 2, "");
      break;

    case 6:
      display.set2X();
      displayline(adc3_volts, 1, " ADC2");
      displayline(dac_throttle_left, 3, " DACl");
      displayline(dac_throttle_right, 5, " DACr");
      break;
  }

  displayDelay.repeat();  // Start the timer again without drift
}

void initialise_adc_arrays() {
  // Initialize current average array using first reading. This means average normalizes quicker

  for (int thisReading = 0; thisReading < AvADCReadings; thisReading++) {
    adc0_total += adc0;
    adc1_total += adc1;
    adc2_total += adc2;
    adc3_total += adc3;

    adc0_Readings[thisReading] = adc0;
    adc1_Readings[thisReading] = adc1;
    adc2_Readings[thisReading] = adc2;
    adc3_Readings[thisReading] = adc3;
  }
}

void read_adc_values() {

  adc0 = ads.readADC_SingleEnded(0);  // Current
  adc1 = ads.readADC_SingleEnded(1);  // Steer
  adc2 = ads.readADC_SingleEnded(2);  // Temp
  adc3 = ads.readADC_SingleEnded(3);  // Throttle

  adc0_volts = ads.computeVolts(adc0);
  adc1_volts = ads.computeVolts(adc1);
  adc2_volts = ads.computeVolts(adc2);
  adc3_volts = ads.computeVolts(adc3);

  adc0_total = adc0_total - adc0_Readings[adc_index];
  adc0_Readings[adc_index] = adc0;
  adc1_total = adc1_total - adc1_Readings[adc_index];
  adc1_Readings[adc_index] = adc1;
  adc2_total = adc2_total - adc2_Readings[adc_index];
  adc2_Readings[adc_index] = adc2;
  adc3_total = adc3_total - adc3_Readings[adc_index];
  adc3_Readings[adc_index] = adc3;

  adc0_total += adc0_Readings[adc_index];
  adc1_total += adc1_Readings[adc_index];
  adc2_total += adc2_Readings[adc_index];
  adc3_total += adc3_Readings[adc_index];

  adc0_average = adc0_total / AvADCReadings;
  adc1_average = adc1_total / AvADCReadings;
  adc2_average = adc2_total / AvADCReadings;
  adc3_average = adc3_total / AvADCReadings;

  adc_current = adc0_volts;
  adc_steer = adc1_volts;
  adc_tcouple = adc2_volts;
  adc_throttle = adc3_volts;

  adc_current_average = adc0_average;
  adc_steer_average = adc1_average;
  adc_tcouple_average = adc2_average;
  adc_throttle_average = adc3_average;

  adc_index += 1;
  if (adc_index >= AvADCReadings) {
    adc_index = 0;
  }

  if (debug_hi == 1) {

    Serial.println("ADC 0: " + String(adc0) + ", ADC 1: " + String(adc1) + ", ADC 2: " + String(adc2) + ", ADC 3: " + String(adc3));
    Serial.println("ADC C: " + String(adc_current) + ", ADC S: " + String(adc_steer) + ", ADC Te: " + String(adc_tcouple) + ", ADC Th: " + String(adc_throttle));
    Serial.println("ADC Av0: " + String(adc0_average) + ", Av1: " + String(adc1_average) + ", Av2: " + String(adc2_average) + ", Av3: " + String(adc3_average));
    Serial.println("DAC Left: " + String(dac_throttle_left) + ", DAC Right: " + String(dac_throttle_right));
    Serial.println("==============================");
  }
}

float calc_vbatt() {
uint16_t adc_battery;
uint16_t vbatt;
  // max reading is 1023
  // R1 = 10K, R2 = 100K. Max battery volts = 4.35 @ 12 cells = 52.2V
  // 52.2 * 10 / (10 +100) = 4.74
  // 55 = 1023

  adc_battery = analogRead(A7);
  vbatt = (adc_battery * 55) / 1023;

  if (debug_batt == 1) {

    Serial.println("ADC BATT: " + String(adc_battery) + ", VBATT: " + String(vbatt));
    Serial.println("==============================");
  }

  return vbatt;
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


void setZeroCurrent() {
  int iteration = 10;
  long current_accumulator = 0;

  for (int i = 0; i < iteration; i++) {
    current_accumulator += ads.readADC_SingleEnded(0);
    delay(READ_DELAY);
  }
  current_zero = current_accumulator / iteration;  // Use this for calcs as quicker and less error than float
  current_inst = current_zero;
}

void convert_thermistor() {
  float ThermRes, logR2;
  ThermRes = SERIESRESISTOR * ((4.78 / adc_tcouple) - 1);

  logR2 = log(ThermRes);
  temp_inst = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273;
  temp_average = temp_inst;
}


void throttle_management() {

  dac_throttle = int(trunc(adc3_average));  // move from 16bit to 12bit
  dac_throttle_short = dac_throttle >> 4;   // move from 16bit to 12bit

  dac_throttle_left = dac_throttle_short;
  dac_throttle_right = dac_throttle_short;

  mcp.setChannelValue(MCP4728_CHANNEL_A, dac_throttle_left);   // left
  mcp.setChannelValue(MCP4728_CHANNEL_B, dac_throttle_right);  // right
  mcp.saveToEEPROM();
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
