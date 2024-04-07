#include <CAN.h>

// canSender() function is directly copied and pasted from can2 which is the designated can sender
// need to copy and paste the canReceiver so that this designated esp can receive messages that are on the can
// got rid of canForwardSender as the canForwardSender function is only present in the canBridge as the canForwardSender 
// is meant to transmit messages FROM THE PC TO THE CAN BUS
// got rid of canSender1()
// got rid of canReceiver()

#define TX_GPIO_NUM   21  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX

//==================================================================================//

void setup() {
  Serial.begin (115200);
  while (!Serial);
  delay (1000);

  Serial.println ("CAN Receiver/Receiver");

  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);
            
  // start the CAN bus at 500 kbps
  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }
  CAN.onReceive(onReceive); // onReceive 
}

//==================================================================================//

void loop() {
  canSender();
  delay(1000);
}

//==================================================================================//

void canSender() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print ("Sending packet ... ");

  CAN.beginPacket(0x11); // begining to send
  int ID = 24;
  float data = 69;

  byte buffer[1+ sizeof(float)]; // making the buffer the size 

  memcpy(buffer, &ID, sizeof(byte)); // originally: memcpy(buffer, &ID, sizeof(byte))
  memcpy(buffer + 1, &data, sizeof(float)); 

  CAN.write(buffer, sizeof(buffer));

  CAN.endPacket();

  Serial.println ("done");
  Serial.println(sizeof(buffer));
  Serial.print("Buffer content: ");
  for (int i = 0; i < sizeof(buffer); i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();
}

//==================================================================================//

void onReceive(){
  
}