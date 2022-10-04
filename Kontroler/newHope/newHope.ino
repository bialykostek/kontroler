#include <EEPROM.h>
#include "PWM.hpp"
#include <Servo.h>
#include "ICM_20948.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <LPS.h>

#define COM Serial3
#define WIRE_PORT Wire
#define AD0_VAL 1

SFE_UBLOX_GNSS myGNSS;
ICM_20948_I2C myICM;
LPS ps;

int receiverInterval = 20;
int heartbeatInterval = 7;
int sendDataInterval = 50;
int gpsInterval = 500;
int sensorInterval = 50;

float heartbeatValue = 0;

double yaw, pitch, roll;

int mode = 0;
bool armed = false;

bool emergency = false;

float altiPress;

float vPitot;

long heartbeatTimer = 0;
long receiverTimer = 0;
long sendDataTimer = 0;
long gpsTimer = 0;
long sensorTimer = 0;

long emergencyTimer = 0;
int emergencyCounter = 0;
int emergencySafety = 200;

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

long latitude;
long longitude;
long GPSaltitude;
long gspeed;
byte SIV;
float heading;

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

float getPitot() {
  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  unsigned int T_dat;
  byte Temp_L;
  byte Temp_H;
  address = 0x28;
  Wire.requestFrom((int) address, (int) 4);
  uint16_t millis_start = millis();
  bool ok = true;
  while (Wire.available() < 4) {
    if (((uint16_t) millis() - millis_start) > 1) {
      ok = false;
      Wire.endTransmission();
      return 0;
    }
  }
  if (ok) {
    Press_H = Wire.read();
    Press_L = Wire.read();
    Temp_H = Wire.read();
    Temp_L = Wire.read();
  }
  Wire.endTransmission();
  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int) Press_H) << 8) | Press_L;
  Temp_L = (Temp_L >> 5);
  T_dat = (((unsigned int) Temp_H) << 3) | Temp_L;
  double PR = (double)((P_dat - 819.15) / (14744.7));
  PR = (PR - 0.49060678);
  if(PR < 0){
    PR *= -1;
  } 
  double V = ((PR * 13789.5144) / 1.225);
  float VV = (sqrt((V)));
  return VV;
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
  digitalWrite(13, HIGH);
  
  //communication
  COM.begin(38400);
  Serial.begin(38400);
  
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

  //start mode
  mode = EEPROM.read(0);
  if(mode == 0){
    //normal start
    COM.println();
    COM.println("#1|0");

    WIRE_PORT.begin();

    while(true){
      myICM.begin(WIRE_PORT, AD0_VAL);
      if (myICM.status != ICM_20948_Stat_Ok){
        COM.println();
        COM.println("#3|0");
        delay(500);
      }else{
        COM.println();
        COM.println("#3|1");
        break;
      }
    }
    
    bool success = true;
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
    if (success){
      COM.println();
      COM.println("#4|1");
    }
    else
    {
      while(true){
        COM.println();
        COM.println("#4|0");
        delay(500);
      }
    }

    while(!myGNSS.begin()){
      COM.println();
      COM.println("#5|0");
      delay(500);
    }
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.setNavigationFrequency(2);
    myGNSS.setAutoPVT(true);

    COM.println("#5|1");

    while(!ps.init()) {
      COM.println("#6|0");
      delay(500);
    }
    ps.enableDefault();

    COM.println("#6|1");
    
  }else{
    //emergency
    armed = true;
    emergency = true;
    COM.println();
    COM.println("#1|1");
  }
}

void loop() {
  checkForMessage();

  if(emergency && millis() - emergencyTimer > 1000){
    COM.println();
    COM.println("#7|1");
    emergencyTimer = millis();
  }
  
  if(millis() - receiverTimer > receiverInterval){
    readReceiver();

    if(!emergency && armed){
      if(millis() - emergencyTimer > emergencySafety){
        emergencyCounter++;
        if(emergencyCounter >= 3){
          emergency = true;         
        }
      }else{
        emergencyCounter = 0;
      }
      emergencyTimer = millis();
    }
    
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
    COM.print(",");
    COM.print(roll);
    COM.print(",");
    COM.print(pitch);
    COM.print(",");
    COM.print(yaw);
    COM.print(",");
    COM.print(latitude);
    COM.print(",");
    COM.print(longitude);
    COM.print(",");
    COM.print(GPSaltitude);
    COM.print(",");
    COM.print(SIV);
    COM.print(",");
    COM.print(gspeed);
    COM.print(",");
    COM.print(heading);
    COM.print(",");
    COM.print(altiPress);
    COM.print(",");
    COM.print(vPitot);
    COM.println();
    
    sendDataTimer = millis();
  }

  if(!emergency){
    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);
  
    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)){
      if ((data.header & DMP_header_bitmap_Quat6) > 0){
        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        double q2sqr = q2 * q2;
        double t0 = +2.0 * (q0 * q1 + q2 * q3);
        double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
        roll = atan2(t0, t1) * 180.0 / PI;
        double t2 = +2.0 * (q0 * q2 - q3 * q1);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        pitch = asin(t2) * 180.0 / PI;
        double t3 = +2.0 * (q0 * q3 + q1 * q2);
        double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
        yaw = atan2(t3, t4) * 180.0 / PI;
        yaw *= -1;
        yaw += 180;
       }
    }
  }
  
  if(!emergency && millis() - gpsTimer > gpsInterval){
    if (myGNSS.getPVT() && (myGNSS.getInvalidLlh() == false)){
      latitude = myGNSS.getLatitude();
      longitude = myGNSS.getLongitude();
      GPSaltitude = myGNSS.getAltitude();
      SIV = myGNSS.getSIV();
      gspeed = myGNSS.getGroundSpeed();
      heading = heading = (float)((float)myGNSS.getHeading())/100000.0;
      gpsTimer = millis();
    }
  }

  if(!emergency && millis() - sensorTimer > sensorInterval){
    float pressure = ps.readPressureMillibars();
    altiPress = ps.pressureToAltitudeMeters(pressure);
    
    vPitot = getPitot();
    
    sensorTimer = millis();
  }
}
