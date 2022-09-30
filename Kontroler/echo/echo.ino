#define COM Serial3

void setup() {
  COM.begin(38400);
  Serial.begin(38400);
}

void loop() {
  if(COM.available()){
    COM.write(COM.read());
    //Serial.print(COM.read());
  }
}
