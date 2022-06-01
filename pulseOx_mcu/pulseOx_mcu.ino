/*
* pulseOx_mcu.ino
*
* script for driving LEDs and reading ADC for ENGS129 pulse oximeter final project
*
* modified from Professor Kofi Odame's example code, `streamADC01.ino` and
* `redIR.ino`.
*
* Ian Raphael
* ian.a.raphael.th@dartmouth.edu
*
* Sophie Lloyd
* sophie.lloyd.th@dartmouth.edu
*
* 2022.05.18
*/

// #include "arduinoFFT.h"
#include <TimerThree.h>
#include <SPI.h>
#include <math.h>
// #include <ADC.h>

#define DEBUG 1

#define baudRate 9600

// #define redThreshold 3850
// #define redMaxThreshold 3723
#define redMaxThreshold 3720
#define redMinThreshold 3350

#define lowThresh 2482
#define midThresh 2730
#define highThresh 2854

#define voltageFactor 0.0008058608059 // = 3.3/(2^12-1) for scaling binary voltage to decimal

#define SAMPLING_FREQ_EFF 26880 //Hz, found from oscilloscope for sampling freq of ADC with all code

// arduinoFFT FFT = arduinoFFT();
// double vReal[BUFFLENGTH];
// double vImag[BUFFLENGTH];

/********** LED driver constants **********/
const int IR = 24;
const int RED = 15;

int state = 0;
volatile int count = 0; // use volatile for shared variables
int countCopy;          // holds a copy of the count

int state0Length;
int state1Length;
int state2Length;
int state3Length;

/********** ADC constants **********/
#define fSample 1000 // adc sample rate. maximum 1MHz

#define BUFFLENGTH 1000 // buffer length for data buffers
// #define BUFFLENGTH 10000 // buffer length for data buffers

byte data[2]; // holder array for instantaneous ADC reads
int redData[BUFFLENGTH]; // buffer for reddata
int irData[BUFFLENGTH]; // buffer for ir data
int redMaxAverage = 0; //
// int muRed[BUFFLENGTH];
// int muIR[BUFFLENGTH];
// int cRed[BUFFLENGTH];
// int cIR[BUFFLENGTH];
int O2Sat;
int hr;


//
// uint8_t transmitBuf[BUFFLENGTH+2];
// uint8_t lightFlag[1];

int n = 0;
int n1 = 0;
int envelopeDetectAvg = 0;
#define average_countThreshold 30

// SPI pins
const int CSPin   = 10;
const int MOSIPin = 11;
const int MISOPin0 = 12; // connect PMODAD1.D0 to Teensy.pin12
const int MISOPin1 = 39; // connect PMODAD1.D1 to Teensy.pin39
const int adc2 = 32; // teensy adc pin for envelope detector
const int SCKPin  = 14;

// SPI Settings: speed, mode and endianness
SPISettings settings(fSample, MSBFIRST, SPI_MODE2);  // 1MHz, MSB,

void setup(void)
{

  // set the adc for the peak detector
  analogReadResolution(12);
  pinMode(adc2,INPUT);

  pinMode(25,OUTPUT);

  // set up leds
  pinMode(IR, OUTPUT);  // designate pin 13 an output pin
  pinMode(RED, OUTPUT); // designate pin 14 an output pin
  Timer3.initialize(25);
  Timer3.attachInterrupt(ISR); // call ISR every 25 us

  // set up adc
  pinMode(CSPin, OUTPUT);
  Serial.begin(baudRate);

  // Configure SPI Pins
  SPI.begin();
  SPI.setMISO(MISOPin0);
  SPI.setMOSI(MOSIPin);
  SPI.setSCK(SCKPin);

  // set initial state lengths to maximum
  state0Length=16;               // state0 lasts 14*25 = 350 us. Do not exceed 400 us (i.e. keep state0Length <= 16)
  state1Length=20-state0Length;  // state1 lasts (500 us minus the state0 duration)
  state2Length=16;               // state2 lasts 14*25 = 350 us. Do not exceed 400 us (i.e. keep state2Length <= 16)
  state3Length=20-state2Length;  // state3 lasts (500 us minus the state2 duration)
}

void loop(void) {

  // Serial.println("made it into main loop" );

  // if we've taken enough data samples
  if (n>= BUFFLENGTH) {

    // read the envelope detector adc
    int envelopeDetect = analogRead(adc2);

    // add to the running sum for avg
    envelopeDetectAvg += envelopeDetect;

    // increment n1
    n1++;


    // if we've taken enough peak detector samples
    if (n1>average_countThreshold) {

      // delay(25000);

      // calculate heartrate
      Serial.print("heartrate: ");
      Serial.println(getHeartRate(irData, lowThresh, midThresh, highThresh));

      digitalWrite(25,HIGH);
      // HeartRate(irData);
      digitalWrite(25,LOW);

      // get the average peak detector value
      envelopeDetectAvg = envelopeDetectAvg/n1;

      // Serial.print("Peak detect avg: ");
      // Serial.println(envelopeDetectAvg*voltageFactor);
      updatePwm(envelopeDetectAvg);

      // reset count and avg count
      n1 = 0;
      envelopeDetectAvg = 0;
    }

    // reset everything
    memset(redData, 0, sizeof(redData));
    memset(irData, 0, sizeof(irData));
    n = 0;
  }

  // // read red adc
  // getADC(data,MISOPin0);
  // redData[n] = ((data[0] << 8) + data[1]);
  //
  // // read ir adc
  // getADC(data,MISOPin1);
  // irData[n] = ((data[0] << 8) + data[1]);


  noInterrupts();
  countCopy = count;
  interrupts();

  // finite state machine
  switch (state) {
    case 0:
    if (countCopy >= state0Length){
      noInterrupts();
      count = 0;
      state = 1;
      interrupts();
    }
    else{
      // red ON
      digitalWrite(RED, HIGH);
    }
    break;
    case 1:
    if (countCopy >= state1Length){
      noInterrupts();
      count = 0;
      state = 2;
      interrupts();
    }
    else{
      // red OFF
      digitalWrite(RED, LOW);
    }
    break;
    case 2:
    if (countCopy >= state2Length){
      noInterrupts();
      count = 0;
      state = 3;
      interrupts();
    }
    else{

      // IR ON
      digitalWrite(IR, HIGH);
    }
    break;
    case 3:
    if (countCopy >= state3Length){
      noInterrupts();
      count = 0;
      state = 0;
      interrupts();
    }
    else{
      // getADC(data,MISOPin1);
      // IR OFF
      digitalWrite(IR, LOW);
    }
    break;
  }

  // sample
  // read red adc

  getADC(data,MISOPin0);
  redData[n] = ((data[0] << 8) + data[1]);
  getADC(data,MISOPin1);
  irData[n] = ((data[0] << 8) + data[1]);
  n++;
}

/********************** getHeartRate() *********************/
int getHeartRate(int* data, int lowThreshold, int midThreshold, int highThreshold) {

  int index1 = -1;
  int index2 = -1;
  int index3 = -1;
  int findNextPulse = 0;
  int heartRate = -1;

  // for every value in the array
  for (int i=0;i <sizeof(data);i++) {

    // if the value is higher than the mid threshold and we haven't found a pulse yet
    if ((data[i] >= midThreshold) && (index1 == -1)) {
      // save it as index1
      index1 = i;
      continue;
    }

    // if the value is higher than the high threshold and we've already found index1
    if ((data[i] >= highThreshold) && (index1 != -1) && (!findNextPulse)) {
      // we've found a true pulse. start looking for the decrease.
      findNextPulse = 1;
      continue;
    }

    // if the data is below the low threshold & we're finding the next pulse
    // if ((data[i] <= lowThreshold) && (findNextPulse==1) && (index2 == -1)) {
    if ((data[i] <= lowThreshold)) {
      Serial.println("made i3");
      // save this index as the low swing
      index2 = i;
      continue;
    }
    else if ((data[i]>midThreshold) && (index2 != -1)) {
      index3 = i;
      Serial.print("made it4");
    }
    else if ((data[i]>highThreshold) && (index3 != -1)) {

      Serial.print("index1: ");
      Serial.println(index1);

      Serial.print("index2: ");
      Serial.println(index1);

      Serial.print("index3: ");
      Serial.println(index1);

      // calculate heart rate from index 1 and 3
      heartRate = 1/((index3 - index1)/SAMPLING_FREQ_EFF);

      break;

      // find maximum and minimum values
    }
  }

  return heartRate;
}

/********************** findMinMax() *********************/
// int findMinMax(int* data, int heartRate) {
//   // look through data vector for a maximum heartrate
//
//   // look within (heartrate/sampling rate) indices to find minimum
//
// }


/********************** max() *********************/
int max(int *data) {

  // set max val as first val in array
  int maxVal = data[0];

  // for every value in the array
  for (int i = 0; i < sizeof(data); i++) {
    // if this value is more than previous max
    if (data[i] > maxVal) {

      // update max value
      maxVal = data[i];
    }
  }

  // return max value
  return maxVal;
}

// float getAverage(int *data) {
//   int sum=0;
//   int i;
//   for (i=0;i<sizeof(data);i++){
//     sum += data[i];
//   }
//
//   int average = (float) sum/(float) i;
//   return average;
// }
//

// update PWM
int updatePwm(int peakVoltage) {

  // if received voltage is too high
  if ((peakVoltage > redMaxThreshold) && (state0Length > 2)) {

    // decrease pulse with
    state0Length=state0Length-1;   // decrement state0
    state1Length=20-state0Length;  // state1 lasts (500 us minus the state0 duration)
    state2Length=state0Length;     // match state 0 length
    state3Length=20-state2Length;  // state3 lasts (500 us minus the state2 duration)

    if (DEBUG) {
      Serial.print("Threshold exceeded: ");
      Serial.println(peakVoltage*voltageFactor);
      Serial.print("Decreasing pulse width to: ");
      Serial.println(state0Length);
    }
  }
  // if received voltage is too low
  else if ((peakVoltage < redMinThreshold) && (state0Length <= 15)) {

    // increase the pulse width
    state0Length=state0Length+1;   // increment state0
    state1Length=20-state0Length;  // state1 lasts (500 us minus the state0 duration)
    state2Length=state0Length;     // match state 0 length
    state3Length=20-state2Length;  // state3 lasts (500 us minus the state2 duration)

    if (DEBUG) {
      Serial.print("Threshold exceeded: ");
      Serial.println(peakVoltage*voltageFactor);
      Serial.print("Increasing pulse width to: ");
      Serial.println(state0Length);
    }
  }
  return state0Length;
}

// double HeartRate(int* data) {
//
//   // sampling_period_us = round((1.0/SAMPLING_FREQ));
//   double vImag[sizeof(data)];
//   memset(vImag,0,sizeof(data));
//
//
//   /*FFT*/
//     FFT.Windowing((double*)data, sizeof(data), FFT_WIN_TYP_HAMMING, FFT_FORWARD);
//     FFT.Compute((double*) data, vImag, sizeof(data), FFT_FORWARD);
//     FFT.ComplexToMagnitude((double*) data, vImag, sizeof(data));
//     double peak = FFT.MajorPeak((double*) data, sizeof(data), SAMPLING_FREQ);
//
//     /*PRINT RESULTS*/
//     Serial.println(peak);     //Print out what frequency is the most dominant.
//     return peak;
// }

// keep count, in 25 us divisions
void ISR(void)
{
  count = count + 1;
}

void getADC(byte* data, int whichMISO) {

  SPI.setMISO(whichMISO);    // set MISO pin

  SPI.beginTransaction(settings);
  digitalWrite(CSPin,LOW);   // pull CSPin low to start SPI communication
  data[0] = SPI.transfer(0); // grab upper byte
  data[1] = SPI.transfer(0); // grab lower byte
  digitalWrite(CSPin,HIGH);  // set CSPin high to end SPI communication
  SPI.endTransaction();
}
