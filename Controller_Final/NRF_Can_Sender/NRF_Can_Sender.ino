// sender code
// SimpleTx - the master or the transmitter

// ADD A JOYSTICK CODE

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 22
#define CSN_PIN 21

#define VRX_PIN_LEFT_JOYSTICK 35   // Controls Left Joystick. X position (Horizontal)
#define VRY_PIN_RIGHT_JOYSTICK 34  // Controls Right Joystick. Y position (Vertical)
#define button 12
#define toggle_switch 14

int xValue = 0;  // Initial position at (0,0)
int yValue = 0;  // Initial position at (0,0)

// int prevState = HIGH;
// int currentState;

const byte thisSlaveAddress[5] = { 'R', 'x', 'A', 'A', 'A' };
RF24 radio(CE_PIN, CSN_PIN);
char dataToSend[10] = "Message 0";
char txNum = '0';
unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 200;  // send once per second

struct data_pack {
  float xValuePack;
  float yValuePack;
  int buttonState;
  int togSwitchVal;
};

float xToSend = 0;
float yToSend = 0;

data_pack data_var;

void setup() {
  Serial.begin(115200);
  pinMode(VRX_PIN_LEFT_JOYSTICK, INPUT);
  pinMode(VRY_PIN_RIGHT_JOYSTICK, INPUT);
  Serial.println("SimpleTx Starting");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);  // delay, count
  radio.openWritingPipe(thisSlaveAddress);
  pinMode(button, INPUT_PULLUP);
  pinMode(toggle_switch, INPUT_PULLUP);
}

void loop() {

  data_var.xValuePack = analogRead(VRX_PIN_LEFT_JOYSTICK) >> 4;
  data_var.yValuePack = analogRead(VRY_PIN_RIGHT_JOYSTICK) >> 4;
  data_var.buttonState = digitalRead(button);
  data_var.togSwitchVal = digitalRead(toggle_switch);

  
  yToSend = (float) data_var.yValuePack;
  xToSend = (float) data_var.xValuePack;

  data_var.yValuePack = mapFunc(yToSend, 0, 255, -2.22, 2.22);
  data_var.xValuePack = mapFunc(xToSend, 0, 255, -2.22, 2.22);
  if(data_var.yValuePack > -0.3 && data_var.yValuePack < -0.2)
    data_var.yValuePack = 0;
  

  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    send(data_var.xValuePack, data_var.yValuePack, data_var.buttonState, data_var.togSwitchVal);
    prevMillis = millis();
  }
}

void send(float X_VAL_HORIZONTAL, float Y_VAL_VERTICAL, int BUTTON_VAL, int SWITCH_VAL) {
  bool rslt_data_bool, rslt_button_bool;

  //  rslt_message = radio.write(&dataToSend, sizeof(dataToSend));
  rslt_data_bool = radio.write(&data_var, sizeof(data_var));


  // Always use sizeof() as it gives the size as the number of bytes.
  // For example if dataToSend was an int sizeof() would correctly return 2
  Serial.print("Data Sent ");
  Serial.print(dataToSend);
  if (rslt_data_bool) {
    Serial.print(" Acknowledge received:");
    Serial.print("x-value = ");
    Serial.print(X_VAL_HORIZONTAL);
    Serial.print(", y-value = ");
    Serial.print(Y_VAL_VERTICAL);
    Serial.print(", Button-value = ");
    Serial.print(BUTTON_VAL);
    Serial.print(", Switch-value = ");
    Serial.println(SWITCH_VAL);
    updateMessage();
  } else {
    Serial.println(" Tx failed");
  }
}
void updateMessage() {
  // can see that new data is being sent
  txNum += 1;
  if (txNum > '9') {
    txNum = '0';
  }
  dataToSend[8] = txNum;
}

float mapFunc(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}