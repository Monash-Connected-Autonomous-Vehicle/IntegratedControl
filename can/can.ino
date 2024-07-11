// may not work
//==================================================================================//

#include <CAN.h>

#define TX_GPIO_NUM   2  // Connects to CTX Originally 21
#define RX_GPIO_NUM   15  // Connects to CRX Originally 22

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


  // delay(1000);
  CAN.onReceive(onReceiveTesting);


}

//==================================================================================//

void loop() {
  delay(100000000000000000);
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

}

//==================================================================================//

void onReceiveTesting(int packetSize) {
  int ID;
  if (CAN.packetExtended()) {
    // Serial.print("extended ");
  }

  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    // Serial.print("RTR ");
  }

  // Serial.print("packet with id 0x");
  // Serial.print(CAN.packetId(), HEX);

  if (CAN.packetRtr()) {
    // Serial.print(" and requested length ");
    // Serial.println(CAN.packetDlc());
  } else {
    Serial.print("Length of packet size: ");
    Serial.println(packetSize);

    // only print packet data for non-RTR packets

    while (CAN.available()) { // DELETE THIS AFTER DONE
      if(packetSize == 8){ // data coming from the PC. PC sends decoded data already, i.e. the actual values
        byte idBuffer[4]; 
        byte buffer[8];
        byte buffer1[8];
        byte buffer2[8];
        for (int i = 0; i < 16; i++) {
          if(i < 8){
            buffer1[i] = CAN.read();
          }
          
          else{
            buffer2[i - 8] = CAN.read();
          }
        }
        byte idData;
        byte data;
        memcpy(&idData, buffer1, sizeof(idData));
        memcpy(&data, buffer2, sizeof(data));
        Serial.print("ID: ");
        Serial.println(buffer1[1], 1);
        Serial.print("Data: ");
        Serial.println(buffer1[0], 1);
        Serial.println("");
      }
      else if(packetSize == 5){ // data coming from the CAN
        byte idBuffer[4];
        byte buffer[5];
        byte buffer1[1];
        byte buffer2[4];
        // byte buffer3[4];

        for (int i = 0; i < 5; i++) {
          if(i < 1){
            buffer1[i] = CAN.read();
          }
          else{
            buffer2[i - 1] = CAN.read();
          }
        
        }
        
        int idData = buffer1[0];
        float data, data2;
        memcpy(&data, buffer2, sizeof(data));
        // memcpy(&data2, buffer3, sizeof(data2));
        Serial.print("ID: ");
        Serial.println(idData);
        Serial.print("Data: ");
        Serial.println(data, 5);
        Serial.println("");
      }
      else if(packetSize == 3){
        int id;
        int data1;
        int data2;

        for(int i = 0; i < packetSize + 1; i++){
          if(i == 1){
            id = CAN.read();
          } else if(i == 2){
            data1 = CAN.read();
          } else if(i == 3){
            data2 = CAN.read();
          }
        }
        Serial.print("ID: ");
        Serial.println(id);
        Serial.print("Data 1: ");
        Serial.println(data1);
        Serial.print("Data 2: ");
        Serial.println(data2);
        Serial.println("");
      }
    } 
  }
}

//==================================================================================//

void onReceive(int packetSize) {
  // delay(1000);
  // received a packet
  // Serial.print("Received ");
  int ID;
  if (CAN.packetExtended()) {
    // Serial.print("extended ");
  }

  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    // Serial.print("RTR ");
  }

  // Serial.print("packet with id 0x");
  // Serial.print(CAN.packetId(), HEX);

  if (CAN.packetRtr()) {
    // Serial.print(" and requested length ");
    // Serial.println(CAN.packetDlc());
  } else {
    Serial.print("Length of packet size: ");
    Serial.println(packetSize);

    // only print packet data for non-RTR packets

    while (CAN.available()) { // DELETE THIS AFTER DONE
      if(packetSize == 8){ // data coming from the PC. PC sends decoded data already, i.e. the actual values
        byte idBuffer[4]; 
        byte buffer[8];
        byte buffer1[8];
        byte buffer2[8];
        for (int i = 0; i < 16; i++) {
          if(i < 8){
            buffer1[i] = CAN.read();
          }
          
          else{
            buffer2[i - 8] = CAN.read();
          }
        }
        byte idData;
        byte data;
        memcpy(&idData, buffer1, sizeof(idData));
        memcpy(&data, buffer2, sizeof(data));
        Serial.print("ID: ");
        Serial.println(buffer1[1], 1);
        Serial.print("Data: ");
        Serial.println(buffer1[0], 1);
      }
      else if(packetSize == 5){ // data coming from the CAN
        byte idBuffer[4];
        byte buffer[5];
        byte buffer1[1];
        byte buffer2[4];
        // byte buffer3[4];

        for (int i = 0; i < 5; i++) {
          if(i < 1){
            buffer1[i] = CAN.read();
          }
          else{
            buffer2[i - 1] = CAN.read();
          }
        
        }
        
        int idData = buffer1[0];
        float data, data2;
        memcpy(&data, buffer2, sizeof(data));
        // memcpy(&data2, buffer3, sizeof(data2));
        Serial.print("ID: ");
        Serial.println(idData);
        Serial.print("Data: ");
        Serial.println(data, 5);
        // Serial.print("Data 2: ");
        // Serial.println(data2, 5);
      }
    } 
  }
}

// void onReceive(int packetSize) { UNCOMMENT AFTER DONE
//   // delay(1000);
//   // received a packet
//   // Serial.print("Received ");
//   int ID;
//   if (CAN.packetExtended()) {
//     // Serial.print("extended ");
//   }

//   if (CAN.packetRtr()) {
//     // Remote transmission request, packet contains no data
//     // Serial.print("RTR ");
//   }

//   // Serial.print("packet with id 0x");
//   // Serial.print(CAN.packetId(), HEX);

//   if (CAN.packetRtr()) {
//     // Serial.print(" and requested length ");
//     // Serial.println(CAN.packetDlc());
//   } else {
//     Serial.print("Length of packet size: ");
//     Serial.println(packetSize);

//     // only print packet data for non-RTR packets

//     while (CAN.available()) { // DELETE THIS AFTER DONE
//       if(packetSize == 8){ // data coming from the PC. PC sends decoded data already, i.e. the actual values
//         byte idBuffer[4]; 
//         byte buffer[8];
//         byte buffer1[8];
//         byte buffer2[8];
//         for (int i = 0; i < 16; i++) {
//           if(i < 8){
//             buffer1[i] = CAN.read();
//           }
          
//           else{
//             buffer2[i - 8] = CAN.read();
//           }
//         }
//         byte idData;
//         byte data;
//         memcpy(&idData, buffer1, sizeof(idData));
//         memcpy(&data, buffer2, sizeof(data));
//         Serial.print("ID: ");
//         Serial.println(buffer1[1], 1);
//         Serial.print("Data: ");
//         Serial.println(buffer1[0], 1);
//       }
//       else if(packetSize == 5){ // data coming from the CAN
//         byte idBuffer[4];
//         byte buffer[5];
//         byte buffer1[1];
//         byte buffer2[4];
//         // for(int i = 0; i < 4; i ++){
//         //   buffer2[i] = CAN.read();
//         // }
//         for (int i = 0; i < 5; i++) {
//           if(i < 1){
//             buffer1[i] = CAN.read();
//           }
//           else{
//             buffer2[i - 1] = CAN.read();
//           }
//         }
        
//         int idData = buffer1[0];
//         float data;
//         memcpy(&data, buffer2, sizeof(data));
//         Serial.print("ID: ");
//         Serial.println(idData);
//         Serial.print("Data: ");
//         Serial.println(data, 5);
//       }
//     } 
//   }
// }