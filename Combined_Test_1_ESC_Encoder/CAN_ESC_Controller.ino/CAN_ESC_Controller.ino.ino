/* 
|--------------< CONTENTS >---------------|
|_[ Global immutible Variables ] 
  |_ ESC related settings
  |_ Encoder & pwm settings
  |_ [ Global Functions, Initilizations of mutable variables ]
    |_ ESC Dependencies 
    |_ Encoder Dependencies
|_ [ void setup ]

*/

// GLOBALS
const long HW_TIMER_INTERVAL_US = 20;
const uint32_t TIME_INTERVAL = 100000;   //Unit: Microseconds
const uint32_t TICKS_PER_ROTATION = 1024;  //how many encoder ticks per roation of wheel
const float WHEEL_RADIUS = 0.13;      //Unit: Metres

// Pin Definitions
const uint32_t PIN_ESC_A_PWM = 14; // PWM Output Pin for ESC A
const uint32_t PIN_ESC_B_PWM = 15; // PWM Output Pin for ESC B
const uint32_t PIN_RENC_A = 32;  // Pin for Rotary Encoder A
const uint32_t PIN_RENC_B = 33;  // Pin for Rotary Encoder B

#define PI 3.14159265

/////////////////////////////////
// DEFINITIONS NEEDED FOR ESC: //
/////////////////////////////////

#define _PWM_LOGLEVEL_          3
#define USING_MICROS_RESOLUTION true
// Default is true, uncomment to false
// Unsure what this does, from old code
//#define CHANGING_PWM_END_OF_CYCLE     false

#include "ESP32_PWM.h" //Only include this in the main .ino file

volatile uint32_t startTimeMicroseconds

////////////////
// TIMER INIT //
////////////////

// Initialise ESP32 Timer 1
ESP32Timer ITimer(1);

//////////////////////////////
// INITIALISE ESP32_ISR_PWM //
//////////////////////////////
ESP32_PWM ISR_PWM;

bool IRAM_ATTR TimerHandler(void * timerNo)
{
  ISR_PWM.run();

  return true;
}

#define USING_PWM_FREQUENCY     false

//TODO: WHAT IS THIS
int channelNum;

float throttleToDuty(float throttle)
{
  // Assume the vehicles ESC which maps 7.5% to neutural, 5% reverse, 10% forward (full)
  // function: duty(throttle) = throttle * multipier + constant
  //float mult = 0.025;
  //float cons = 7.5;

  float mult = 0.01;
  float cons = 7.5;

  // adding 25 because of deadzone
  return (throttle + 25) * mult + cons;
}

// Frequency used in tandem with duty cycle to control the ESC
const float PWM_FREQ = 50;

void setPWMThrottle(int pin, int channel, float throttle)
{
  // You can use this with PWM_Freq in Hz
  if (!ISR_PWM.modifyPWMChannel(channel, pin, PWM_FREQ, throttleToDuty(throttle)))
  {
    Serial.print(F("modifyPWMChannel error for PWM_Period"));
  }
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
