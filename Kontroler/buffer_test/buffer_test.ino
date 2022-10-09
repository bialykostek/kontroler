void* bufferxd[50];

void setup() {
  // put your setup code here, to run once:
Serial3.begin(38400);
Serial.begin(38400);
delay(2000);
Serial.println("Start xd");
Serial.println(Serial3.availableForWrite());
Serial3.addMemoryForWrite(bufferxd, sizeof(bufferxd));
Serial.println(Serial3.availableForWrite());
}

void loop() {
  // put your main code here, to run repeatedly:

}
