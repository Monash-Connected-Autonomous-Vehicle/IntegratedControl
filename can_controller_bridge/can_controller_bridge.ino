//==================================================================================//

#include <CAN.h>

#define TX_GPIO_NUM   21  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX

//==================================================================================//

struct data_pack{
  int xValuePack;
  int yValuePack;
  int buttonState;
  int togSwitchVal;
};

data_pack data_var;

void setup() {
  Serial.begin (115200);
  while (!Serial);
  delay (1000);

  Serial.println ("CAN Bridge Receiver/Receiver");

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
  // register the receive callback
  // CAN.onReceive(onReceive);
}

//==================================================================================//

void loop() {

  canReceiver(); // sends data to PC
  // delay(500);
  RxRadioTxCan(); // reads data from PC
}


//==================================================================================//

void RxRadioTxCan() {

  if(radio.available){
    radio.read(&data_var, sizeof(data_var));
    CAN.beginPacket(0x12);
    CAN.write(data_var.xValuePack);
    CAN.beginPacket(0x13);
    CAN.write(data_var.yValuePack);
    CAN.beginPacket(0x14);
    CAN.write(data_var.buttonState);
    CAN.beginPacket(0x15);
    CAN.write(data_var.togSwitchVal);
    CAN.endPacket();
  }

}

//==================================================================================//

void canSender() {
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

  delay (1000);
}

//==================================================================================//


void canReceiver2() { // Jiawei's code
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
      //sex?
      Serial.print ("RTR ");
      rtrSender(CAN.packetId(), CAN.packetDlc());
    }

    Serial.print ("packet with id ");
    Serial.print (CAN.packetId(), DEC);

    if (CAN.packetRtr()) {
      Serial.print (" and requested length ");
      Serial.println (CAN.packetDlc());
    } else {
      Serial.print (" and length ");
      Serial.println (packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        byte buffer[8];
        for (int i = 0; i < 8; i++) {
          buffer[i] = CAN.read();
        }

        double data;
        memcpy(&data, buffer, sizeof(data));
        Serial.println(data, 12);
      }
      Serial.println();
    }

    Serial.println();
  }
}

//==================================================================================//

void canReceiver() { // FORWARDS MESSAGES TO THE PC
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // received a packet
    // Serial.print ("Received ");

    if (CAN.packetExtended()) {
      // Serial.print ("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      // Serial.print ("RTR ");
      rtrSender(CAN.packetId(), CAN.packetDlc());
    }

    // Serial.print ("packet with id 0x");
    // Serial.print (CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      // Serial.print (" and requested length ");
      // Serial.println (CAN.packetDlc());
    } else {
      // Serial.print (" and length ");
      // Serial.println (packetSize);

      // only print packet data for non-RTR packets
      Serial.write(CAN.packetId());
      while (CAN.available()) {
        //Serial.print ((char) CAN.read());
        Serial.write( CAN.read());

      
      }
      // Serial.println();
    }

    // Serial.println();
  }
}

//==================================================================================//
void onReceive(int packetSize) {
  // received a packet
  Serial.print("Received ");

  if (CAN.packetExtended()) {
    Serial.print("extended ");
  }

  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    Serial.print("RTR ");
    rtrSender(CAN.packetId(), CAN.packetDlc());
  }

  Serial.print("packet with id 0x");
  Serial.print(CAN.packetId(), HEX);

  if (CAN.packetRtr()) {
    Serial.print(" and requested length ");
    Serial.println(CAN.packetDlc());
  } else {
    Serial.print(" and length ");
    Serial.println(packetSize);

    // only print packet data for non-RTR packets
    while (CAN.available()) {
      Serial.print((char)CAN.read());
    }
    Serial.println();
  }

  Serial.println();
}

void rtrSender(long id, int DLC) {
  if (id == 0x13) {
    CAN.beginPacket (0x13);
    for (int i = 0; i < DLC; i++) {
      CAN.write ('e');
    }
    CAN.endPacket();
  }
  delay (1000);
}