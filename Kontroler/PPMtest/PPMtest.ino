#include <PulsePosition.h>

PulsePositionInput myInput(RISING);

void setup() {
  // put your setup code here, to run once:
  myInput.begin(6);
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
   for(int i =0; i < myInput.available(); i++){
  Serial.print(myInput.read(i));
  Serial.print(",");
  }
  Serial.println();
  delay(50);
  
}
