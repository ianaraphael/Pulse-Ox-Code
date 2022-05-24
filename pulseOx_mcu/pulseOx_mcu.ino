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
* 2022.05.18
*/

#include <TimerThree.h>
#include <SPI.h>
#include <math.h>

#define baudRate 9600

#define redThreshold 3.0


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
#define fSample 1000000

#define BUFFLENGTH 300

byte data[2];
int redData[BUFFLENGTH];
int irData[BUFFLENGTH];
int muRed[BUFFLENGTH];
int muIR[BUFFLENGTH];
int cRed[BUFFLENGTH];
int cIR[BUFFLENGTH];
int O2Sat;
int hr;

// uint8_t transmitBuf[BUFFLENGTH+2];
// uint8_t lightFlag[1];
int numLoops=0;
int n = 0;

// SPI pins
const int CSPin   = 10;
const int MOSIPin = 11;
const int MISOPin0 = 12; // connect PMODAD1.D0 to Teensy.pin12
const int MISOPin1 = 39; // connect PMODAD1.D1 to Teensy.pin39
const int SCKPin  = 14;

// int ADCdata0;
// int ADCdata1;

// SPI Settings: speed, mode and endianness
SPISettings settings(fSample, MSBFIRST, SPI_MODE2);  // 1MHz, MSB,


void setup(void)
{

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
}

void loop(void)
{
  // adjust state0Length and state2Length according to desired duty cycle
  state0Length=14;               // state0 lasts 14*25 = 350 us. Do not exceed 400 us (i.e. keep state0Length <= 16)
  state1Length=20-state0Length;  // state1 lasts (500 us minus the state0 duration)
  state2Length=14;               // state2 lasts 14*25 = 350 us. Do not exceed 400 us (i.e. keep state2Length <= 16)
  state3Length=20-state2Length;  // state3 lasts (500 us minus the state2 duration)

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

  if (n>= BUFFLENGTH) {

    // get red max
    redMax = getMax(redData);

    // get ir max
    irData = getMax(irData);

    // if red received signal is greater than threshold
    if(redMax > redThreshold) {
      // update our pwm

      // DEBUG
      Serial.println("exceeded red threshold");
    }

    // int redAverage = getAverage(redData);
    // int irAverage = getAverage(irData);

    /**** do our signal processing ****/
    // // calculate mu for red
    // for (int i=0; n<(BUFFLENGTH); n++) {
    //
    //   // muRed[i] = ln(redData)/ln();
    // }
    //
    // // calculate mu for IR

    // transmit the data out
    // Serial.write(redData);
    // Serial.flush();

    // reset everything
    memset(redData, 0, sizeof(redData));
    memset(irData, 0, sizeof(irData));
    n = 0;
  }

  // read red adc
  getADC(data,MISOPin0);
  redData[n] = ((data[0] << 8) + data[1]);

  // read ir adc
  getADC(data,MISOPin1);
  irData[n] = ((data[0] << 8) + data[1]);

  n++;
}

/********************** getMax() *********************/
float getMax(int *data) {

  // set max val as first val in array
  float maxVal = data[0];

  // for every value in the array
  for (int i = 0; i < sizeof(data); i++) {
    // if this value is more than previous max
    if (myArray[i] > maxVal) {

      // update max value
      maxVal = myArray[i];
    }
  }

  // return max value
  return (float) maxVal;
}

float getAverage(int *data) {
  int sum;
  for (int i=0;i<sizeof(data);i++){
    sum += data[i];
  }

  int average = (float) sum/(float) i;
  return average;
}

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
