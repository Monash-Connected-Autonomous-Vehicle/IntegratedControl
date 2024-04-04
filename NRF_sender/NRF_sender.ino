// code for handheld controller
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <CAN.h>

#define CE_PIN 22
#define CSN_PIN 21

#define VRX_PIN_LEFT_JOYSTICK 35   // Controls Left Joystick. X position (Horizontal)
#define VRY_PIN_RIGHT_JOYSTICK 34  // Controls Right Joystick. Y position (Vertical)
#define BUTTON 12
#define TOGGLE_SWITCH 14

const byte thisSlaveAddress[5] = { 'R', 'x', 'A', 'A', 'A' };

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 200;

// ISR flag
volatile bool dataChange = false;

// first send
bool first = true;

struct data_pack {
  u_int8_t xValuePack;
  u_int8_t yValuePack;
  u_int8_t buttonState;
  u_int8_t togSwitchVal;
};
data_pack outData;

RF24 radio(CE_PIN, CSN_PIN);

void updateInput() {
  outData.xValuePack = analogRead(VRX_PIN_LEFT_JOYSTICK);
  outData.yValuePack = analogRead(VRY_PIN_RIGHT_JOYSTICK);
  outData.buttonState = digitalRead(BUTTON);
  outData.togSwitchVal = digitalRead(TOGGLE_SWITCH);
}
void send() {
  currentMillis = millis();
  if (currentMillis - prevMillis) {
    Serial.print("sending\n");
    bool rslt_data_bool, rslt_button_bool;
    rslt_data_bool = radio.write(&outData, sizeof(outData));
  }
  prevMillis = millis();
}

void IRAM_ATTR dataISR() {
  dataChange = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(VRX_PIN_LEFT_JOYSTICK, INPUT);
  pinMode(VRY_PIN_RIGHT_JOYSTICK, INPUT);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);  // delay, count
  radio.openWritingPipe(thisSlaveAddress);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(TOGGLE_SWITCH, INPUT_PULLUP);

  // GPIO interrupts
  attachInterrupt(VRX_PIN_LEFT_JOYSTICK, dataISR, CHANGE);
  attachInterrupt(VRY_PIN_RIGHT_JOYSTICK, dataISR, CHANGE);
  attachInterrupt(BUTTON, dataISR, FALLING);
  attachInterrupt(TOGGLE_SWITCH, dataISR, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (dataChange){
    updateInput();
    send();
    dataChange = false;
  }
}
