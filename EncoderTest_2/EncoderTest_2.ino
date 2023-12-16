#define time_interval 100000  // us
#define ticks_per_roation 1024  //how many encoder ticks per roation of wheel
#define wheel_radius 0.13       //m
#define ENCODER_A 32  // Pin for Encoder A
#define ENCODER_B 33  // Pin for Encoder B
#define PI 3.14159265

float goal_velocity = 0.9;
float err = 0.0;
float vel = 0.0;
float old_vel = 0.0;
float accel = 0.0;

volatile int elapsed_ticks = 0;
volatile int displacement_ticks = 0; // Global variable for storing the encoder position

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

float measure_velocity() {
  float rotations_per_interval = float(elapsed_ticks) / float(ticks_per_roation); 
  float meters_per_interval = rotations_per_interval * float(wheel_radius) * 2 * PI;
  float time_interval_floatified = float(time_interval)/float(1000000); //fucking wanted to be an int every step of the way and was flooring my decimals to 0
  elapsed_ticks = 0; //reset accumilator 
  return (meters_per_interval / time_interval_floatified);
}

float measure_accel(float vel, float vel_old){
  return((vel - vel_old)/(float(time_interval)/float(1000000)));
}

float err_amount(float velocity, float goal){
  return((velocity-goal)/goal);
}



void setup() {
  Serial.begin(115200); // Initialize serial communication

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  // Attaching the ISR to encoder A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);
}



void loop() {
delayMicroseconds(time_interval);
old_vel = vel;
vel = measure_velocity();
accel = measure_accel(vel, old_vel);


// use this print format so you can disable individual plots on the plotter window
Serial.print("velocity:"+ String(vel,10)+",");
Serial.print("goal:"+ String(goal_velocity,10)+",");
Serial.print("accel:"+ String(accel,10)+",");

//Serial.print("error:"+ String(err_amount(vel,goal_velocity),10)+",");
//Serial.print("zero:"+ String(0,10)+",");

Serial.println();

}
