#include <EEPROM.h>
#include "PWM.hpp"
#include <Servo.h>

#define COM Serial3


int receiverInterval = 20;
int heartbeatInterval = 7;
int sendDataInterval = 50;

float heartbeatValue = 0;

int mode = 0;
bool armed = false;

bool emergency = false;

int heartbeatTimer = 0;
int receiverTimer = 0;
int sendDataTimer = 0;

PWM ch1(2);
PWM ch2(3);
PWM ch3(4);
PWM ch4(5);
PWM ch5(6);
PWM ch6(7);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo esc;

int inp1, inp2, inp3, inp4, inp5, inp6;

int requestStatus = 0;
int requestTarget = 0;
int requestType = 0;
float requestValue = 0;
bool requestComma = false;
int requestCommaPlace = 0;


void queryValue(int query){
  switch(query) {
    case 0:
      //server echo
      COM.println();
      COM.println("#0|1");
      break;
    case 1:
      //server echo
      COM.println();
      if(armed){
        COM.println("#2|1");
      }else{
        COM.println("#2|0");
      }
      break;
    default:
      COM.print("Query ");
      COM.print(query);
      COM.println(" is not defined.");
  }
}

void changeValue(int target, float value){
  COM.print("Zmiana o wartosci ");
  COM.print(target);
  COM.print(" na wartosc ");
  COM.print(value);
  COM.println(".");
}

void executeOrder(int order){
  switch(order) {
    case 0:
      //arm
      armed = true;
      EEPROM.write(0, 1);
      break;
    case 1:
      //disarm
      armed = false;
      EEPROM.write(0, 0);
      break;
    default:
      COM.print("Order ");
      COM.print(order);
      COM.println(" is not defined.");
  }
}

void checkForMessage(){
  if(COM.available()){
    int in = COM.read();
    
    if(in == 46){
      requestComma = true;
    }

    if(in-'0' >= 0 && in-'0' <= 9){
      if(requestStatus == 1){
        requestTarget *= 10;
        requestTarget += in-'0';
      }
      if(requestStatus == 2){
        if(!requestComma){
          requestValue *= 10;
          requestValue += in-'0';
        }else{
          requestValue += (float)(in-'0')/requestCommaPlace;
          requestCommaPlace *= 10;
        }
      }
    }

    if(in == 59){
      requestStatus++;
      if(requestStatus == 2 && requestType == 0){
        queryValue(requestTarget);
        requestStatus = 0;
      }
      if(requestStatus == 2 && requestType == 1){
        executeOrder(requestTarget);
        requestStatus = 0;
      }
      if(requestStatus == 3 && requestType == 2){
        changeValue(requestTarget, requestValue);
        requestStatus = 0;
      }
    }
    
    if(in == 63 || in == 64 || in == 33){
      //default
      requestStatus = 0;
      requestTarget = 0;
      requestType = 0;
      requestValue = 0;
      requestComma = false;
      requestCommaPlace = 10;
    }
    if(in == 63){
      requestStatus = 1;
      requestType = 0;
    }else if(in == 64){
      requestStatus = 1;
      requestType = 1;
    }else if(in == 33){
      requestStatus = 1;
      requestType = 2;
    }
  }
}


void readReceiver(){
  inp1 = map(ch4.getValue(), 1100, 1900, 0, 180);
  inp2 = map(ch2.getValue(), 1100, 1900, 0, 180);
  inp3 = map(ch5.getValue(), 1100, 1900, 0, 180);
  inp4 = map(ch3.getValue(), 1100, 1900, 0, 180);
  inp5 = map(ch1.getValue(), 1100, 1900, 0, 180);
  inp6 = map(ch6.getValue(), 1100, 1900, 0, 180);
}

void setup() {
  //heart
  pinMode(13, OUTPUT);
  
  //communication
  COM.begin(38400);
  Serial.begin(38400);

  //start mode
  mode = EEPROM.read(0);
  if(mode == 0){
    //normal start
    COM.println();
    COM.println("#1|0");
  }else{
    //emergency
    armed = true;
    emergency = true;
    COM.println();
    COM.println("#1|1");
  }

  //receiver
  //PWM
  ch1.begin(true);
  ch2.begin(true);
  ch3.begin(true);
  ch4.begin(true);
  ch5.begin(true);
  ch6.begin(true);

  //SERVO
  servo1.attach(23);
  servo2.attach(22);
  servo3.attach(11);
  servo4.attach(10);
  esc.attach(9, 1000, 2000);
  
}

void loop() {
  checkForMessage();
  
  if(millis() - receiverTimer > receiverInterval){
    readReceiver();

    servo1.write(inp1);
    servo2.write(inp2);
    servo3.write(inp3);
    servo4.write(inp4);
    if(armed){
      esc.write(inp5);
    }else{
      esc.write(0);  
    }
    
    receiverTimer = millis();
  }
  
  if(millis() - heartbeatTimer > heartbeatInterval){
    int val = (int)(sin(heartbeatValue)*255);
    if(val < 0){
      val = 0;
    }
    analogWrite(13, val);
    if(heartbeatValue >= 3.14){
      heartbeatValue = 0;
    }
    heartbeatValue += 0.03;
    heartbeatTimer = millis();
  }

  if(!emergency && millis() - sendDataTimer > sendDataInterval){
    COM.print("$");
    COM.print(inp1);
    COM.print(",");
    COM.print(inp2);
    COM.print(",");
    COM.print(inp3);
    COM.print(",");
    COM.print(inp4);
    COM.print(",");
    COM.print(inp5);
    COM.print(",");
    COM.print(inp6);
    COM.println();
    sendDataTimer=millis();
  }

}
