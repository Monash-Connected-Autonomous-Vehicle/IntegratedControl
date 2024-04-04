                                                                                                                                                                                                                                                                                                                                                                              //==================================================================================//

#include <CAN.h>

#define TX_GPIO_NUM   21  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX


//==================================================================================//

void setup() {

  
  //pinMode(15, LOW);
  Serial.begin (115200);
  while (!Serial);
  delay (10);

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
}

//==================================================================================//

void loop() {
  // canForwarderSender();
  canSender();
  delay(1000);
  // canReceiver();
}

//==================================================================================//
void canForwarderSender() {
  if(Serial.available() > 0){
    // a = Serial.read()
    CAN.beginPacket(Serial.read());
    
    
    while(Serial.available() > 0) {
      CAN.write (Serial.read());
    }
    CAN.endPacket();
  }
}

void canSender1() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print ("Sending packet ... ");

  CAN.beginPacket (0x12);  //sets the ID and clears the transmit buffer
  // CAN.beginExtendedPacket(0xabcdef);
  CAN.write ('1'); //write data to buffer. data is not sent until endPacket() is called.
  CAN.write ('2');
  CAN.write ('3');
  CAN.write ('4');
  CAN.write ('5');
  CAN.write ('6');
  CAN.write ('7');
  CAN.write ('8');
  CAN.endPacket();

  //RTR packet with a requested data length
  // CAN.beginPacket (0x12, 3, true);
  // CAN.endPacket();

  Serial.println ("done");

}

void canSender() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print ("Sending packet ... ");

  // This identifier can't been found on receiving python side 
  CAN.beginPacket(0x11);
  byte preambleBuffer = 0;
  // byte ID = 69;
  int ID = 24;
  float data = 69;

  byte buffer[1+ sizeof(float)]; // originally: byte buffer[sizeof(byte) + sizeof(double)];

  memcpy(buffer, &ID, sizeof(byte)); // originally: memcpy(buffer, &ID, sizeof(byte))
  memcpy(buffer + 1, &data, sizeof(float)); 

  // memcpy(buffer + sizeof(int), &data, sizeof(float)); 

  CAN.write(buffer, sizeof(buffer));

  CAN.endPacket();

  //RTR packet with a requested data length
  // CAN.beginPacket (0x12, 3, true);
  // CAN.endPacket();

  Serial.println ("done");
  Serial.println(sizeof(buffer));
  Serial.print("Buffer content: ");
  for (int i = 0; i < sizeof(buffer); i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();

  // delay (1000);
}

//==================================================================================//

void canReceiver() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // received a packet
    Serial.print ("Received ");

    if (CAN.packetExtended()) {
      Serial.print ("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print ("RTR ");
    }

    Serial.print ("packet with id 0x");
    Serial.print (CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print (" and requested length ");
      Serial.println (CAN.packetDlc());
    } else {
      Serial.print (" and length ");
      Serial.println (packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        Serial.print ((char) CAN.read());
      }
      Serial.println();
    }

    Serial.println();
  }
}

//==================================================================================//



