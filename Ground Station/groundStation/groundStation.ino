void setup()
{
  // Otwarcie portu do komunikacji
  Serial.begin(38400);
  Serial2.begin(38400);
  delay(1000);
}

void loop()
{
  if(Serial2.available()){
    Serial.write(Serial2.read());
  }
  if(Serial.available()){
    char in = Serial.read();
    if(in == 35){
      Serial.println("%");
    }else{
      Serial2.write(in);
    }
  }
}
