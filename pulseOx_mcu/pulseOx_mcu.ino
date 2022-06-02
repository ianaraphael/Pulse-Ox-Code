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

#include <TimerThree.h>
#include <SPI.h>

#define baudRate 9600

/****************** biomed constants ******************/
#define KVALUE 2.5
#define maxHeartRate 150 // maximum acceptable heart rate, otherwise discarded as noise
#define minHeartRatePeriod 400 // minimum acceptable heart rate period 1/(400/1000)*60 = 150 bpm
#define minHeartRateSamples 5 // minimum samples for heartrate average
#define redMaxThreshold 3720
#define redMinThreshold 3350
#define voltageFactor 0.0008058608059 // = 3.3/(2^12-1) for scaling binary voltage to decimal

#define pwmUpdateTime 1000 // time between PWM update calls in [ms]

/****************** LED driver constants ******************/
const int IR = 24;
const int RED = 15;

/****************** fsm constants ******************/
int state = 0;
volatile int count = 0; // use volatile for shared variables
int countCopy;          // holds a copy of the count

int state0Length;
int state1Length;
int state2Length;
int state3Length;

/****************** ADC ******************/
#define fSample 1000000 // adc sample rate. maximum 1MHz
byte data[2]; // holder array for instantaneous ADC reads

/****************** init values ******************/
double prevTime = 0;
double avgHeartRate = 0;
int nHeartRateSamples = 0;
int currMin_ir = 0;
int currMax_ir = 2000;
int currMin_red = 2000;
int currMax_red = 0;
int pwmStartTime = 0;
double currHeartRate = 0.0;
double currO2 = 100;
double avgO2 = 0;

/****************** SPI settings ******************/
// SPI pins
const int CSPin   = 10;
const int MOSIPin = 11;
const int MISOPin0 = 12; // connect PMODAD1.D0 to Teensy.pin12
const int MISOPin1 = 39; // connect PMODAD1.D1 to Teensy.pin39
const int adc2 = 32; // teensy adc pin for envelope detector
const int SCKPin  = 14;

// SPI Settings: speed, mode and endianness
SPISettings settings(fSample, MSBFIRST, SPI_MODE2);  // 1MHz, MSB,

void setup(void) {

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

  /****************** read data ******************/
  // read red adc
  getADC(data,MISOPin0);
  int currRed = ((data[0] << 8) + data[1]);

  // read ir adc
  getADC(data,MISOPin1);
  int currIR = ((data[0] << 8) + data[1]);

  // if ir signal is less than previous min
  if (currIR < currMin_ir) {
    // save as new min
    currMin_ir = currIR;
  }
  // if ir signal is more than previous max
  if (currIR > currMax_ir){
    // save as new max
    currMax_ir = currIR;
  }
  // if red signal is less than previous min
  if (currRed < currMin_red) {
    // save as new min
    currMin_red = currRed;
  }
  // if red signal is more than previous max
  if (currRed > currMax_red){
    // save as new max
    currMax_red = currRed;
  }

  /****************** read envelope detect ******************/
  // read the envelope detector adc
  int currEnvelopeDetect = analogRead(adc2);

  // get the current time
  int pwmCurrTime = millis();

  // get the elapsed time
  int elapsedPWM = pwmCurrTime - pwmStartTime;

  // if it's greater than the threshold
  if (elapsedPWM >= pwmUpdateTime) {

    // update our PWM
    updatePwm(currEnvelopeDetect);

    // save this as new start time
    pwmStartTime = pwmCurrTime;
  }

  /****************** check for peak ******************/
  // check if there's a peak match - detect minimum
  int matchedPeak = matchPeakDetect(currEnvelopeDetect,currIR);
  //   Serial.print("matched peak?: ");
  //   Serial.println(matchedPeak);

  // if our IR value is a peak
  if (matchedPeak == 1) {

    // get the curr time
    double currTime = millis();

    // get the elpased time since last peak in [ms]
    double diffTime = currTime - prevTime;

    // if enough time has elapsed to be a real heartbeat
    if (diffTime > minHeartRatePeriod) {

      // get heartrate in bpm
      currHeartRate = 1/(diffTime/1000)*60;
      //      Serial.print("currHeartRate ");
      //      Serial.println(currHeartRate);

      // add it to the running sum
      avgHeartRate += currHeartRate;
      avgO2 += currO2;

      // increment n samples
      nHeartRateSamples++;

      // update time
      prevTime = currTime;

      // good time to find blood saturation while we're at it
      currO2 = BloodSat((double)currMin_red, (double)currMax_red, (double)currMin_ir,(double)currMax_ir);
      Serial.println(currO2);
    }
  }

  // if we've taken enough heart rate samples
  if (nHeartRateSamples >= minHeartRateSamples) {

    // calculate average heartRate
    avgHeartRate = avgHeartRate/nHeartRateSamples;
    avgO2 = avgO2/nHeartRateSamples;

    Serial.print("heartrate: ");
    Serial.println(avgHeartRate);
    Serial.print("O2 Saturation: ");
    Serial.println(avgO2);

    // reset counts and avg sums
    nHeartRateSamples = 0;
    avgHeartRate = 0;
    avgO2 = 0;
    currO2 = 0;
    currMin_ir = 2000;
    currMin_red = 2000;
    currMax_ir = 0;
    currMax_red = 0;
  }


}
/****************** end loop() ******************/


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
  return maxVal;
}

/********************** matchPeakDetect() *********************/
int matchPeakDetect(int peak, int data) {
  int maxpk = 0;

  // want to check if data = reaches min
  if ((double)data >= (double).99*peak){
    maxpk = 1;
  }

  return maxpk;
}

/****************** BloodSat() ******************/
double BloodSat(double RedMin, double RedMax, double IRMin, double IRMax) {

  // take in min and max from each LED color and determine absorption ratio
  double R = (RedMax/RedMin)/(IRMax/IRMin);
  int k = KVALUE; // calibration term that needs to be tuned empiracally
  double O2sat = 100-k*R;
  //  Serial.print("Oxygen saturation: ");
  //  Serial.println(O2sat);
  return O2sat;
}

/****************** updatePwm() ******************/
int updatePwm(int peakVoltage) {

  // if received voltage is too high
  if ((peakVoltage > redMaxThreshold) && (state0Length > 2)) {

    // decrease pulse with
    state0Length=state0Length-1;   // decrement state0
    state1Length=20-state0Length;  // state1 lasts (500 us minus the state0 duration)
    state2Length=state0Length;     // match state 0 length
    state3Length=20-state2Length;  // state3 lasts (500 us minus the state2 duration)

    Serial.print("Threshold exceeded: ");
    Serial.println(peakVoltage*voltageFactor);
    Serial.print("Decreasing pulse width to: ");
    Serial.println(state0Length);

  }
  // if received voltage is too low
  else if ((peakVoltage < redMinThreshold) && (state0Length <= 15)) {

    // increase the pulse width
    state0Length=state0Length+1;   // increment state0
    state1Length=20-state0Length;  // state1 lasts (500 us minus the state0 duration)
    state2Length=state0Length;     // match state 0 length
    state3Length=20-state2Length;  // state3 lasts (500 us minus the state2 duration)
    //
    Serial.print("Threshold exceeded: ");
    Serial.println(peakVoltage*voltageFactor);
    Serial.print("Increasing pulse width to: ");
    Serial.println(state0Length);

  }
  return state0Length;
}


/****************** ISR ******************/
void ISR(void)
{
  /****************** fsm ******************/
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
      // IR OFF
      digitalWrite(IR, LOW);
    }
    break;
  }
  count = count + 1;
}

/****************** getADC ******************/
void getADC(byte* data, int whichMISO) {

  SPI.setMISO(whichMISO);    // set MISO pin

  SPI.beginTransaction(settings);
  digitalWrite(CSPin,LOW);   // pull CSPin low to start SPI communication
  data[0] = SPI.transfer(0); // grab upper byte
  data[1] = SPI.transfer(0); // grab lower byte
  digitalWrite(CSPin,HIGH);  // set CSPin high to end SPI communication
  SPI.endTransaction();
}
