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
int redpeak[BUFFLENGTH]; // peak locations for red
// int irpeak[BUFFLENGTH]; // peak indexes for ir
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
int prevTime = 0;
int avgHeartRate = 0;
int minHeartRate = 20;
int nHeartRates = 0;

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

  pinMode(25,INPUT);


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

  digitalWrite(25,HIGH);

  // if we've taken enough data samples
  if (n>= BUFFLENGTH) {

    // increment n1
    n1++;

    // if we've taken enough peak detector samples
    if (n1>average_countThreshold) {

      // calculate average heartRate
      avgHeartRate = avgHeartRate/nHeartRates;

      Serial.print("heartrate: ");
      Serial.println(avgHeartRate);

      // get the average peak detector value
      envelopeDetectAvg = envelopeDetectAvg/n;

      // Serial.print("Peak detect avg: ");
      // Serial.println(envelopeDetectAvg*voltageFactor);
      updatePwm(envelopeDetectAvg);

      // reset counts and avg sums
      n1 = 0;
      envelopeDetectAvg = 0;

      avgHeartRate = 0;
      nHeartRates = 0;
    }

    // reset everything
    memset(redData, 0, sizeof(redData));
    memset(irData, 0, sizeof(irData));
    // memset(irpeak, 0,sizeof(irpeak));
    n = 0;
  }

  // read red adc
  getADC(data,MISOPin0);
  redData[n] = ((data[0] << 8) + data[1]);

  Serial.println("made it here1");

  // read ir adc
  getADC(data,MISOPin1);
  irData[n] = ((data[0] << 8) + data[1]);
  Serial.println("made it here2");

  // read the envelope detector adc
  int currEnvelopeDetect = analogRead(adc2);
  Serial.println("made it here3");

  // add envelope detect value to the running sum for avg
  envelopeDetectAvg += currEnvelopeDetect;
  Serial.println("made it here4");


  matchPeakDetect(currEnvelopeDetect,irData[n]);

  Serial.println("made it here5");

  // if our IR value is a peak
  if (matchPeakDetect(currEnvelopeDetect,irData[n])) {

    // get the curr time
    int currTime = millis();

    // if this is not the first loop
    if (n!=0) {

      // get the millis difference from the last peak
      int diffTime = currTime - prevTime;

      // get the heartrate
      int currHeartRate = (1000/diffTime) * 60;

      // if it's greater than a threshold
      if (currHeartRate >= minHeartRate){

        Serial.print("heartRate: ");
        Serial.println(currHeartRate);

        // add it to the running sum
        avgHeartRate += currHeartRate;
        nHeartRates++;

        // save this time
        prevTime = currTime;
      }
    } else { // otherwise this is the first loop

      // save this time
      prevTime = currTime;
    }
  }
  digitalWrite(25,LOW);


  // // how to get n1 of irpeak to line up with envelope read
  // Serial.print("IR peak: ");
  // Serial.println(irpeak[n1]);

  // increment n
  n++;

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


/********************** max() *********************/
int max(int *data) {

  // set first val in array as current maximum
  int maxVal = data[0];

  // for every value in the array
  for (int i = 1; i < sizeof(data); i++) {
    // if this value is more than previous max
    if (data[i] > maxVal) {

      // update max value
      maxVal = data[i];
    }
  }
}

/********************** matchPeakDetect() *********************/
int matchPeakDetect(int peak, int data) {
  int maxpk = 0;

  // want to check if data = peak from peak detector
  if (data == peak){
    maxpk = 1;
  }

  return maxpk;
}

/********************** findMinMax() *********************/
int findMinMax(int* data, int heartRate) {
  // look through data vector for a maximum heartrate
  int heartfreq = heartRate/60;
  int numSamps = 1/heartfreq;

  // We start off with the first entry as the nearest.
  byte nearindex = 0;
  // abs returns the positive portion of a value, so -40 becomes 40.
  int localMin = 2.2/voltageFactor;

  // Now scan the rest of the array.
  for (byte i = 1; i < numSamps; i++) {
    // Is the difference from the target value for this entry smaller than the currently recorded one?
    if (abs(data[i]) < localMin) {
      // Yes, so save the index number along with the current difference.
      localMin = data[i];
      nearindex = i;
    }
  }
  Serial.print("The localMin is: ");
  Serial.println(data[nearindex]);
  return localMin;
}

float BloodSat(int RedMin, int RedMax, int IRMin, int IRMax) {

  // take in min and max from each LED color and determine absorption ratio
  float R = (RedMax/RedMin)/(IRMax/IRMin);
  int k = 10; // calibration term that needs to be tuned empiracally
  float O2sat = 110-k*R;
  Serial.print("Oxygen saturation: ");
  Serial.println(O2sat);
  return O2sat;
}

// float findHeartRate(int* irpeaks) {
//   int peakdist;
//   // check that maxes are far enough apart but will need to add a moving variable to index
//   if (irpeak[2] - irpeak[1] > 10)
//   peakdist = irpeak[2] - irpeak[1];
//   float heartfreq = 1/(peakdist*fSample);
//   float heartrate = heartfreq*60;
//   Serial.print("Heartrate: ");
//   Serial.println(heartrate);
//   return heartrate;
// }

/********************** max() *********************/
//int max(int *data) {
//
//  // set max val as first val in array
//  int maxVal = data[0];
//
//  // for every value in the array
//  int size = sizeof(data) / sizeof(int)
//  for (int i = 0; i < size; i++) {
//    // if this value is more than previous max
//    if (data[i] > maxVal) {
//
//      // update max value
//      maxVal = data[i];
//    }
//  }
//
//  // return max value
//  return maxVal;
//}

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
