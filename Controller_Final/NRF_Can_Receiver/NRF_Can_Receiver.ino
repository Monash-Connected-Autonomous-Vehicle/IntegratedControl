

// SimpleRx - the slave or the receiver
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <CAN.h>

#define SERVO1 1
#define SERVO2 2
#define LED_BUTTON 14
#define LED_SWITCH 12

#define CE_PIN  22
#define CSN_PIN 21

#define TX_GPIO_PIN 15
#define RX_GPIO_PIN 2

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

const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};
RF24 radio(CE_PIN, CSN_PIN);
char dataReceived[10]; // this must match dataToSend in the TX
int xValData;
int yValData;
bool newData = false;

struct data_pack{
  float xValuePack;
  float yValuePack;
  float buttonState;
  float togSwitchVal;
};

data_pack data_var;

//===========
void setup()
{
  Serial.begin(115200);
  // while(!Serial);
  delay(3000);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
  pinMode(LED_BUTTON, OUTPUT);
  pinMode(LED_SWITCH, OUTPUT);
  Serial.println("SimpleRx Starting");
  CAN.setPins(RX_GPIO_PIN, TX_GPIO_PIN);
  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }

}
//=============
void loop()
{
  getData();
  delay(100);
//  canSender500();

  // Float 1: X-Value (This is going to be changed to the steering angle or equivalent value)
  canSenderFinal(ESDACanMessageID::SteerAmount, data_var.xValuePack);

  // // // Float 2: Y-Value (This is the target velocity)
  canSenderFinal(ESDACanMessageID::SetTargetVelLeft, data_var.yValuePack);
  canSenderFinal(ESDACanMessageID::SetTargetVelRight, data_var.yValuePack);

  // // // Float 3: Button value (On/Off value maybe?)
  canSenderFinal(ESTOP, data_var.buttonState);

  // // // Float 4: 
  canSenderFinal(4, data_var.togSwitchVal);

  showData();
  //ledLightUp();
}
//==============
void getData()
{
  if (radio.available()){
    radio.read(&data_var, sizeof(data_var));
    newData = true;
  }
}

void showData()
{
   if (newData == true)
   {
    Serial.print("Data received: x-value =  ");
    Serial.print(data_var.xValuePack);
    Serial.print(", y-value = ");
    Serial.print(data_var.yValuePack);
    Serial.print(", Button-value = ");
    Serial.print(data_var.buttonState);
    Serial.print(", Switch-value = ");
    Serial.println(data_var.togSwitchVal);

    //Serial.write(data_var.yValuePack);

    newData = false;
   
  }
}

void ledLightUp(){
  if(!data_var.buttonState){
    digitalWrite(LED_BUTTON, HIGH);
   } else{
    digitalWrite(LED_BUTTON, LOW);
   }
  if(!data_var.togSwitchVal){
    digitalWrite(LED_SWITCH, HIGH);
  } else{
    digitalWrite(LED_SWITCH, LOW);
  }
}

void canSenderFinal(uint32_t messageID, float floatValue){
  byte floatBytes[sizeof(float)];
  memcpy(floatBytes, &floatValue, sizeof(float));
  CAN.beginPacket(messageID);
  CAN.write(floatBytes, sizeof(float));
  CAN.endPacket();
}

// void canSender(){
//   Serial.print("Sending packet ...");
//   CAN.beginPacket(0x1);
//   int ID = 1; 
//   int xDataToCan = data_var.xValuePack;
//   int yDataToCan = data_var.yValuePack;

//   float yDataSendCan = mapFunc(yDataToCan, 0, 255, -2.22, 2.22);
//   if(yDataSendCan > -0.26 && yDataSendCan < 0.2){
//     yDataSendCan = 0;
//   }

//   int buttonDataToCan = data_var.buttonState;
//   int toggleSwitchDataToCan = data_var.togSwitchVal;
//   byte buffer[5];
//   memcpy(buffer, &ID, sizeof(byte));
//   memcpy(buffer + 1, &xDataToCan, sizeof(int)); 
//   memcpy(buffer + 2, &yDataToCan, sizeof(int));
//   memcpy(buffer + 3, &buttonDataToCan, sizeof(int));
//   memcpy(buffer + 4, &toggleSwitchDataToCan, sizeof(int));
//   CAN.write(buffer, sizeof(buffer));

//   CAN.endPacket();

//   Serial.println ("done");
//   Serial.println(sizeof(buffer));
//   Serial.print("Buffer content: ");
//   for (int i = 0; i < sizeof(buffer); i++) {
//     Serial.print(buffer[i], DEC);
//     Serial.print(" ");
//   }
//   Serial.println();
//   Serial.println();
// }

// void canSender500(){
//   Serial.print("Sending packet ...");
//   CAN.beginPacket(0x1);
//   int xDataToCan = data_var.xValuePack;
//   int yDataToCan = data_var.yValuePack;

  

//   int buttonDataToCan = data_var.buttonState;
//   int toggleSwitchDataToCan = data_var.togSwitchVal;

//   float xDataSendCan = mapFunc(xDataToCan, 0, 255, -2.22, -1.95);
//   if(xDataSendCan > -0.26 && xDataSendCan < 0.2){
//     xDataSendCan = 0;
//   }
//   //float xDataSendCan = (float) xDataToCan;
//   float buttonSendCan = (float) buttonDataToCan;

//   CAN.write(xDataSendCan);
  
//   Serial.print(xDataSendCan);
//   Serial.println("");
//   CAN.endPacket();

  
//   // Serial.print("Buffer content: ");
//   // for (int i = 0; i < sizeof(buffer); i++) {
//   //   Serial.print(buffer[i], DEC);
//   //   Serial.print(" ");
//   // }
//   Serial.println();
//   Serial.println();
// }


