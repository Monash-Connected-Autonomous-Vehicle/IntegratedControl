// TEST ENCODER CODE 

#define delta_t 100000 //sleep duration for velcoty readings (us)
#define ticks_per_roation 1024 //how many encoder ticks per roation of wheel
#define wheel_radius 0.13 //m

//Velocity bias from raw displacement readings 
#define instant_bias 0.25
#define rolling_bias 0.75
#define rolling_length 100


// velocity 
float ticks_per_second = 0;
float rot_per_second = 0;
float m_per_second = 0;



#define ENCODER_A 32 // Pin for Encoder A
#define ENCODER_B 33 // Pin for Encoder B
volatile int encoder_value = 0; // Global variable for storing the encoder position
volatile unsigned long time_of_poll = 1000000; // time of latest reading (microseconds)
volatile unsigned long time_of_prev_poll = 0; // time of second latest reading (microseconds - us )

float time_passed_rolling =1; // Holds tick duration over a rolling peroid
float time_passed_instant =1; // Holds tick duration over instant change in time

//times of last x readings, rolling average
volatile unsigned long poll_times[rolling_length];  // BOTH VALUES MUST BE IDENTICAL, One indexes the other. 
volatile int poll_times_index = 0;


void encoder_isr() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    encoder_value--;
  } else {
    encoder_value++;
  }

  // timestamp polls 
  time_of_prev_poll = time_of_poll;
  time_of_poll = micros();

  //Rolling average 
  if (poll_times_index >= rolling_length+1) { poll_times_index = 1; } // Overflow 
  poll_times[poll_times_index] = micros();
  poll_times_index++;
}
void setup() {
  Serial.begin(115200); // Initialize serial communication
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  // Attaching the ISR to encoder A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);
}
void loop() {
  delayMicroseconds(delta_t);


/*ticks per second 
  Calculated based on the velocity given by the difference bewteen two ticks (in us)
  However if a tick is not detected within the time period (velocity) of the previous, 
  a delcelrating velocity is interpelated based on current time and the last tick using min(normal velocity, interpolated)

     normal: 1/((time_of_poll - time_of_prev_poll)*0.000001);
     interpolated: 1/((micros() - time_of_poll)*0.000001)

*/

  poll_times[0] = poll_times[100]; // allows wrap-around indexing ([i-1])
  time_passed_rolling = (time_of_poll - time_of_prev_poll)*0.000001;
  time_passed_instant = (time_of_poll - time_of_prev_poll)*0.000001;


  ticks_per_second = min(1/((micros() - poll_times[poll_times_index])*0.000001) , 1/((poll_times[poll_times_index] - poll_times[poll_times_index-1])*0.000001));

  //rotations per second
  rot_per_second = ticks_per_second/ticks_per_roation;








  //debug
  //Serial.println("Encoder value = " + String(encoder_value));
  Serial.println("Ang Velocity = " + String(rot_per_second,20));
  


  





}