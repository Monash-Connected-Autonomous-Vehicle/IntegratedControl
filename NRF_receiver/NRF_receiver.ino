#include <SPI.h>
#include <RF24.h> 
#include <nRF24L01.h>
#include <CAN.h>

#define CE_PIN  22
#define CSN_PIN 21

#define IRQ_PIN 5

// CAN
#define TX_GPIO_NUM   2  // Connects to CTX
#define RX_GPIO_NUM   15  // Connects to CRX

// button debounce
#define debounce  10

const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};
RF24 radio(CE_PIN, CSN_PIN);

struct data_pack{
  u_int8_t xValuePack;
  u_int8_t yValuePack;
  u_int8_t buttonState;
  u_int8_t togSwitchVal;
};

data_pack inData;

volatile bool dataAvailable = false;

void IRAM_ATTR radio_ISR(){
  dataAvailable = true;
}

void getData(){
  Serial.print("e\n");
  radio.read(&inData, sizeof(inData));
  Serial.print(inData.xValuePack);
}

void sendCAN() {
  Serial.print("b");
  CAN.beginPacket(0x27);
  // CAN.write(inData.xValuePack);
  // CAN.write(inData.yValuePack);
  // CAN.write(inData.buttonState);
  // CAN.write(inData.togSwitchVal);
  CAN.write('1');
  CAN.endPacket();
  Serial.print("e\n");
}

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(3000);
  Serial.println("SimpleRx Starting");

  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);
  // start the CAN bus at 500 kbps
  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (radio.available()) {
    getData();
    // sendCAN();
  }
}
