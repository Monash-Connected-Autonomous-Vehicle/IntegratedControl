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

// You can assign pins here. Be carefull to select good pin to use or crash
uint32_t PWM_Pin    = 14;
#define HW_TIMER_INTERVAL_US      20L

// TODO, Merge goal into pipeline from ROS
    float goal_velocity = 0.9; 


#define time_interval 100000  // us
#define ticks_per_roation 1024  //how many encoder ticks per roation of wheel
#define wheel_radius 0.13       //m
#define ENCODER_A 32  // Pin for Encoder A
#define ENCODER_B 33  // Pin for Encoder B
#define PI 3.14159265









// ESC Dependencies //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// These define's must be placed at the beginning before #include "ESP32_PWM.h"
// _PWM_LOGLEVEL_ from 0 to 4
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_                3

#define USING_MICROS_RESOLUTION       true    //false

// Default is true, uncomment to false
//#define CHANGING_PWM_END_OF_CYCLE     false

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "ESP32_PWM.h"


volatile uint32_t startMicros = 0;

// Init ESP32 timer 1 
ESP32Timer ITimer(1);

//TODO: Additonal timers


// Init ESP32_ISR_PWM
ESP32_PWM ISR_PWM;

bool IRAM_ATTR TimerHandler(void * timerNo)
{
  ISR_PWM.run();

  return true;
}

#define USING_PWM_FREQUENCY     false

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

void changePWM(int pin, int channel, float throttle)
{
  // You can use this with PWM_Freq in Hz
  if (!ISR_PWM.modifyPWMChannel(channel, pin, 50, throttleToDuty(throttle)))
  {
    Serial.print(F("modifyPWMChannel error for PWM_Period"));
  }
}


// Encoder Dependencies //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PID
#include <PID_v2.h>
//double Kp = 2, Ki = 5, Kd = 1; 
double Kp = 1, Ki = 0.1, Kd = 0; 
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);

// Kinematics 
float err = 0.0;
float vel = 0.0;
float old_vel = 0.0;
float accel = 0.0;
float PID_out_magnitude = 0.0;
int PID_out_pwm = 0.0;

volatile int elapsed_ticks = 0;
volatile int displacement_ticks = 0; // Global variable for storing the encoder position


// Interrupt callback function
void encoder_isr() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    displacement_ticks--;
    elapsed_ticks--;
  } else {
    displacement_ticks++;
    elapsed_ticks++;
  }
}

// Math function for ang disp -> lin vel
float measure_velocity() {
  float rotations_per_interval = float(elapsed_ticks) / float(ticks_per_roation); 
  float meters_per_interval = rotations_per_interval * float(wheel_radius) * 2 * PI;
  float time_interval_floatified = float(time_interval)/float(1000000); 
  elapsed_ticks = 0; //reset accumilator 
  return (meters_per_interval / time_interval_floatified);
}

float measure_accel(float vel, float vel_old){
  return((vel - vel_old)/(float(time_interval)/float(1000000)));
}

float err_amount(float velocity, float goal){
  return((velocity-goal)/goal);
}

float PID() {
  float PID_raw = myPID.Run(vel);
  if (PID_raw > 100) {PID_raw = 100;}
  PID_out_pwm = (PID_raw*5) + 1500;
  PID_out_magnitude = PID_raw / 100.0f;
  return PID_raw;
}



// MISC Setup ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while (!Serial);

  delay(2000);
 
// ESC & PWM Section //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.print(F("\nStarting ISR_Modify_PWM on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP32_PWM_VERSION);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

  // Interval in microsecs
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler))
  {
    startMicros = micros();
    Serial.print(F("Starting ITimer OK, micros() = ")); Serial.println(startMicros);
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

channelNum = ISR_PWM.setPWM_Period(PWM_Pin, 50, 7.5f); // Begins at 1500us 
Serial.println("Arming ESCS");
delay(5000); // Allow ESC to arm

// Encoder Section //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  // Attaching the ISR to encoder A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);

  myPID.Start(vel,                // input
              0,                  // current output
              goal_velocity);     // setpoint


}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:

  //TODO: Convert global sleep to ISR Delay
          delayMicroseconds(time_interval);

  // Cache old versions of each order term
    old_vel = vel;
    vel = measure_velocity();
    accel = measure_accel(vel, old_vel);

  // TODO: Make pid function a callback (remove globals for better flow)

  changePWM(PWM_Pin,channelNum,PID());

    // use this print format so you can disable individual plots on the plotter window
    Serial.print("zeroline...Div's_displacement:"+ String(0,10)+",");
    Serial.print("cur:"+ String(displacement_ticks,10)+",");
    Serial.print("cur':"+ String(vel,10)+",");
    Serial.print("tar':"+ String(goal_velocity,10)+",");
    Serial.print("cur'':"+ String(accel,10)+",");
    Serial.print("PID:"+ String(PID_out_magnitude,10)+",");
    Serial.print("PWM:"+ String(PID_out_pwm,10)+",");

    //Serial.print("error:"+ String(err_amount(vel,goal_velocity),10)+",");
    //Serial.print("zero:"+ String(0,10)+",");

    Serial.println();


    




}
