

// SimpleRx - the slave or the receiver
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

#define SERVO1 1
#define SERVO2 2
#define LED_BUTTON 14
#define LED_SWITCH 12

#define CE_PIN  22
#define CSN_PIN 21
const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};
RF24 radio(CE_PIN, CSN_PIN);
char dataReceived[10]; // this must match dataToSend in the TX
int xValData;
int yValData;
bool newData = false;

struct data_pack{
  int xValuePack;
  int yValuePack;
  int buttonState;
  int togSwitchVal;
};

data_pack data_var;

//===========
void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("SimpleRx Starting");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
  pinMode(LED_BUTTON, OUTPUT);
  pinMode(LED_SWITCH, OUTPUT);
}
//=============
void loop()
{
   getData();
   showData();
   ledLightUp();
}
//==============
void getData()
{
  if (radio.available()){
    radio.read(&data_var, sizeof(data_var));
      
    newData = true;
  }
}

int getControllerPosData(){
  
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
