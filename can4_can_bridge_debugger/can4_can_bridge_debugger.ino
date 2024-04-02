void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  float data = 123.456;
  byte buffer[sizeof(float)]; // originally: byte buffer[sizeof(byte) + sizeof(double)];


  // memcpy(buffer, &ID, sizeof(byte)); // originally: memcpy(buffer, &ID, sizeof(byte))
  memcpy(buffer, &data, sizeof(float)); 

  Serial.write(buffer, sizeof(buffer));
}
