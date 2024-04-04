// #include <CAN.h>

// #define TX_GPIO_NUM 21
// #define RX_GPIO_NUM 22

// void setup() {
//   // put your setup code here, to run once:

//   Serial.begin (115200);
//   while (!Serial);
//   delay (1000);
//   Serial.println ("Can bridge");

//   if(!CAN.begin(500E3)){
//     Serial.println("Starting CAN failed");
//     while(1);
//   } else{
//     Serial.println("CAN Bridge Initialized");
//   }

//   // have to write a can sender and can receiver code 
// }



// void loop() {
//   // put your main code here, to run repeatedly:
//   // data from esp32 #1 gets sent to esp32 #2
//   // data from esp32 #2 gets sent from here to the python script
//   canReceiver();
//   // Serial.write(255);
//   // delay(100);
// }



// void canReceiver() {
//   // try to parse packet
//   int packetSize = CAN.parsePacket();

//   if (packetSize) {
//     // received a packet
//     Serial.print ("Received ");

//     if (CAN.packetExtended()) {
//       Serial.print ("extended ");
//     }

//     if (CAN.packetRtr()) {
//       // Remote transmission request, packet contains no data
//       Serial.print ("RTR ");
//       rtrSender(CAN.packetId(), CAN.packetDlc());
//     }

//     Serial.print ("packet with id 0x");
//     Serial.print (CAN.packetId(), HEX);

//     if (CAN.packetRtr()) {
//       Serial.print (" and requested length ");
//       Serial.println (CAN.packetDlc());
//     } else {
//       Serial.print (" and length ");
//       Serial.println (packetSize);

//       // only print packet data for non-RTR packets
//       while (CAN.available()) {
//         Serial.print ((char) CAN.read());
//       }
//       Serial.println();
//     }

//     Serial.println();
//   }
// }

// //==================================================================================//
// void onReceive(int packetSize) {
//   // received a packet
//   Serial.print("Received ");

//   if (CAN.packetExtended()) {
//     Serial.print("extended ");
//   }

//   if (CAN.packetRtr()) {
//     // Remote transmission request, packet contains no data
//     Serial.print("RTR ");
//     rtrSender(CAN.packetId(), CAN.packetDlc());
//   }

//   Serial.print("packet with id 0x");
//   Serial.print(CAN.packetId(), HEX);

//   if (CAN.packetRtr()) {
//     Serial.print(" and requested length ");
//     Serial.println(CAN.packetDlc());
//   } else {
//     Serial.print(" and length ");
//     Serial.println(packetSize);

//     // only print packet data for non-RTR packets
//     while (CAN.available()) {
//       Serial.print((char)CAN.read());
//     }
//     Serial.println();
//   }

//   Serial.println();
// }

// void rtrSender(long id, int DLC) {
//   if (id == 0x13) {
//     CAN.beginPacket (0x13);
//     for (int i = 0; i < DLC; i++) {
//       CAN.write ('e');
//     }
//     CAN.endPacket();
//   }
//   delay (1000);
// }

//==================================================================================//

#include <CAN.h>

#define TX_GPIO_NUM   21  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX

//==================================================================================//


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
  canForwarderSender(); // reads data from PC
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

void canReceiver() {
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

  Serial.println(); // comment
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