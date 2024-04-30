
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

// ENUM DEFINITIONS
enum ESDACanMessageID {
    SetTargetVelLeft = 1,
    SetTargetVelRight = 2,
    CurrentVelLeft = 3,
    CurrentVelRight = 4,
    CurrentDSPLeft = 5,
    CurrentDSPRight = 6,
    SteerAmount = 7,
    MCUState = 16,
    MCUErrorState = 17,
    ESTOP = 8,
    SetAutonomousMode = 9,
};

// GLOBALS

const long HW_TIMER_INTERVAL_US = 20;
const uint32_t TIME_INTERVAL = 100000;   //Unit: Microseconds
const uint32_t TICKS_PER_ROTATION = 1024;  //how many encoder ticks per roation of wheel
const float WHEEL_RADIUS = 0.13;      //Unit: Metres

// // Pin Definitions
const uint32_t PIN_ESC_A_PWM = 26; // PWM Output Pin for ESC A
const uint32_t PIN_ESC_B_PWM = 16; // PWM Output Pin for ESC B

const uint32_t PIN_RENC_L1 = 18;  // Pin 1 for Left Rotary Encoder
const uint32_t PIN_RENC_L2 = 19;  // Pin 2 for Left Rotary Encoder
const uint32_t PIN_RENC_L3 = 21;  // Pin 3 for Left Rotary Encoder
const uint32_t PIN_RENC_R1 = 34;   // Pin 1 for Right Rotary Encoder
const uint32_t PIN_RENC_R2 = 33;   // Pin 2 for Right Rotary Encoder
const uint32_t PIN_RENC_R3 = 32;   // Pin 3 for Right Rotary Encoder

// #define HW_TIMER_INTERVAL_US 20L
// #define TIME_INTERVAL 100000   //Unit: Microseconds
// #define TICKS_PER_ROTATION 1024  //how many encoder ticks per roation of wheel
// #define WHEEL_RADIUS 0.13     //Unit: Metres

// // Pin Definitions
// #define PIN_ESC_A_PWM 5 // PWM Output Pin for ESC A
// #define PIN_ESC_B_PWM 18 // PWM Output Pin for ESC B
// #define PIN_RENC_A1   19 // Pin for Rotary Encoder A
// #define PIN_RENC_A2   21 // Pin for Rotary Encoder B
//const uint32_t PIN_RENC_B1 = ;  // Pin for Rotary Encoder A
//const uint32_t PIN_RENC_B2 = ;  // Pin for Rotary Encoder B

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

volatile uint32_t startTimeMicroseconds;

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

    // adding 25 because of deadzoneX
    return (throttle + 25) * mult + cons;
}

// Frequency of PWM used to control the ESC
const float PWM_FREQ = 50;

void setPWMThrottle(int pin, int channel, float throttle)
{
    // You can use this with PWM_Freq in Hz
    if (!ISR_PWM.modifyPWMChannel(channel, pin, PWM_FREQ, throttleToDuty(throttle)))
    {
        Serial.print(F("modifyPWMChannel error for PWM_Period"));
    }
}

///////////////////////////////
// PID THROTTLE CONTROL LOOP //
///////////////////////////////
#include <PID_v2.h>

volatile float DRAM_ATTR targetVelocities[2] = {0,0};
//double Kp = 2, Ki = 5, Kd = 1; 
double pidGainP = 1, pidGainI = 0.1, pidGainD = 0; 
PID_v2 leftPID(pidGainP, pidGainI, pidGainD, PID::Direct);
PID_v2 rightPID(pidGainP, pidGainI, pidGainD, PID::Direct);

// Kinematics 
float pidErr[2] = {0.0,0.0};
float velocity[2] = {0.0,0.0};
float oldVelocity[2] = {0.0,0.0};
float acceleration[2] = {0.0,0.0};
float pidOutputMagnitude[2] = {0.0,0.0};
int pidOutputPWM[2] = {0.0,0.0};

volatile int DRAM_ATTR pidElapsedTicks[2] = {0.0,0.0};;
volatile int DRAM_ATTR pidDisplacementTicks[2] = {0.0,0.0}; // Global variable for storing the encoder position

// Interrupt callback function for rotary encoders
void encoder_isr() {
    // Process each side
    for (int side = 0; side >= 1; side++) {
        // Reading the current state of encoder A
        int encPin1Val = digitalRead(PIN_RENC_L1);
        int encPin2Val = digitalRead(PIN_RENC_L2);
        // If the state of A changed, it means the encoder has been rotated
        if ((encPin1Val == HIGH) != (encPin2Val == LOW)) {
        pidDisplacementTicks[side]--;
        pidElapsedTicks[side]--;
        } else {
            pidDisplacementTicks[side]++;
            pidElapsedTicks[side]++;
        }
    }
}

// Math function for ang disp -> lin vel
float measureVelocity(int side) {
    float rotationsPerInterval = float(pidElapsedTicks[side]) / float(TICKS_PER_ROTATION); 
    float metersPerInterval = rotationsPerInterval * float(WHEEL_RADIUS) * 2 * PI;
    float timeIntervalFloatified = float(TIME_INTERVAL)/float(1000000); 
    pidElapsedTicks[side] = 0; //reset accumilator 
    return (metersPerInterval / timeIntervalFloatified);
}

float calculateAcceleration(float velocity, float velocityOld){
    return((velocity - velocityOld)/(float(TIME_INTERVAL)/float(1000000)));
}

float errorAmount(float velocity, float velocityGoal){
    return((velocity-velocityGoal)/velocityGoal);
}

float PIDTickLeft() {
    float pidRAW = leftPID.Run(velocity[0]);
    if (pidRAW > 100) {pidRAW = 100;}
    pidOutputPWM[0] = (pidRAW*5) + 1500;
    pidOutputMagnitude[0] = pidRAW / 100.0f;
    return pidRAW;
}
float PIDTickRight() {
    float pidRAW = rightPID.Run(velocity[1]);
    if (pidRAW > 100) {pidRAW = 100;}
    pidOutputPWM[1] = (pidRAW*5) + 1500;
    pidOutputMagnitude[1] = pidRAW / 100.0f;
    return pidRAW;
}

//////////
// CAN2 //
//////////
#include <CAN.h>

// Pins for CAN Transciever
const uint16_t PIN_CAN_TX = 23;
const uint16_t PIN_CAN_RX = 25;

// Link Speed for CANBUS in bps
const long CAN_BAUDRATE = 500E3;

// Function to setup canbus
void setupCAN() {
    Serial.println("Initializing CAN Transciever...");
    // Set CAN Pin States
    CAN.setPins(PIN_CAN_RX,PIN_CAN_TX);
    // Start the CAN Bus and wait for it to come online
    if (!CAN.begin(CAN_BAUDRATE)) {
        Serial.println("Failed to start CAN!");
        // Pause indefinitely
        while (1);
    } else {
        Serial.println("CAN bus Initialized Successfully");
    }
    Serial.println("Adding CAN Recieve Hook");
    CAN.onReceive(onRecieveCAN);
}

////////////////
// MISC SETUP //
////////////////

void setup() {

    // Initialise UART
    Serial.begin(115200);
    // Wait for UART to come online
    while (!Serial);

    // Sleep for 2 Seconds
    delay(2000);
    
    // ESC AND PWM INIT //
    Serial.print(F("\nStarting ISR_Modify_PWM on ")); Serial.println(ARDUINO_BOARD);
    Serial.println(ESP32_PWM_VERSION);
    Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

    // Setup timer with interval //
    if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler))
    {
        startTimeMicroseconds = micros();
        Serial.print(F("Starting ITimer OK, micros() = ")); Serial.println(startTimeMicroseconds);
    }
    else
        Serial.println(F("Can't set ITimer. Select another freq. or timer"));
    // TODO: Handle ESC B
    channelNum = ISR_PWM.setPWM_Period(PIN_ESC_A_PWM, PWM_FREQ, 7.5f); // Begins at 1500us 
    Serial.println("Arming ESCS");
    delay(10000); // Allow ESC time to arm

    /////////////////////
    // Encoder Section //
    /////////////////////

    // Set Encoder A PinModes
    pinMode(PIN_RENC_L1, INPUT_PULLUP);
    pinMode(PIN_RENC_L2, INPUT_PULLUP);

    // Attaching the ISR to encoder A
    attachInterrupt(digitalPinToInterrupt(PIN_RENC_L1), encoder_isr, CHANGE);

    leftPID.Start(velocity[0],          // input
                0,                      // current output
                targetVelocities[0]); 

    rightPID.Start(velocity[1],          // input
                0,                      // current output
                targetVelocities[1]); 

    // Setup the can bus
    // setupCAN();

    }

    void loop() {

    // TODO: Convert global sleep to ISR Delay
    delayMicroseconds(TIME_INTERVAL);

    for (int side = 0; side <= 1; side++) {
        // Cache old versions of each order term
        oldVelocity[side] = velocity[side];
        velocity[side] = measureVelocity(side);
        acceleration[side] = calculateAcceleration(velocity[side], oldVelocity[side]);
    }


    // TODO: Make pid function a callback (remove globals for better flow)

    setPWMThrottle(PIN_ESC_A_PWM,channelNum,PIDTickLeft());
    setPWMThrottle(PIN_ESC_B_PWM,channelNum,PIDTickRight());


    // use this print format so you can disable individual plots on the plotter window
    //Serial.print("zeroline...Div's_displacement:"+ String(0,10)+",");
    //Serial.print("cur:"+ String(pidDisplacementTicks,10)+",");
    //Serial.print("cur':"+ String(velocity,10)+",");
    //Serial.print("tar':"+ String(10,10)+",");
    //Serial.print("cur'':"+ String(acceleration,10)+",");
    //Serial.print("PID:"+ String(pidOutputMagnitude,10)+",");
    //Serial.print("PWM:"+ String(pidOutputPWM,10)+",");

    // Serial.print("error:"+ String(errorAmount(velocity,targetVelocity),10)+",");
    // Serial.print("zero:"+ String(0,10)+",");

    Serial.println();
}

union FloatByteUnion
{
    byte bytes[4];
    float real;
};

void onRecieveCAN(int packetSize) {
    // Store packetID
    ESDACanMessageID packetID = (ESDACanMessageID)CAN.packetId();

    // Serial.print("Recieved ");

    // Handle extended CAN Packet
    if (CAN.packetExtended()) {
        Serial.print("Extended ");
    }

    if (CAN.packetRtr()) {
        //Serial.print("RTR ");
        //TODO: HANDLE RTR
    }

    // Report Packet ID in Hex
    //Serial.print("packet with id ");
    //Serial.print(CAN.packetId(), HEX);

    // Print the requested length if the packet is a Remote Transmit Request
    if (CAN.packetRtr()) {
        //Serial.print(" and requested length ");
        //Serial.print(CAN.packetDlc());
    }
    // Otherwise it is a normal packet
    else {
        // Print its length
        //Serial.print(" and length ");
        //Serial.print(packetSize);

        // Initialise a buffer for the packet data
        // TODO: MAYBE HARDCODE THIS TO 8?
        // packetBuffer[packetSize];

        // Read the can packet into the buffer
        //Serial.print(": Reading CAN packet...");

        FloatByteUnion packetInnerValue;

        // Read the Velocity from the BUS
        packetInnerValue.bytes[0] = CAN.read();
        packetInnerValue.bytes[1] = CAN.read();
        packetInnerValue.bytes[2] = CAN.read();
        packetInnerValue.bytes[3] = CAN.read();


        // Update corresponding target velocity
        switch (packetID)
        {
            case ESDACanMessageID::SetTargetVelLeft:
            targetVelocities[0] = packetInnerValue.real;
            break;
            case ESDACanMessageID::SetTargetVelRight:
            targetVelocities[1] = packetInnerValue.real;
            break;
        
        default:
            break;
        }
        Serial.println();
    }
}
