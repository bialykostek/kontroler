#include <EEPROMex.h>
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

void* comBuffer[100];

bool sendingData = false;

int receiverInterval = 20;
int heartbeatInterval = 7;
int sendDataInterval = 50;
int gpsInterval = 500;
int sensorInterval = 50;

float heartbeatValue = 0;

double yaw, pitch, roll;

int planeX, planeY = 0;
int previousSide = 0;

int mode = 0;
bool armed = false;

bool emergency = false;

float altiPress;

float vPitot;

const int waypointsNumber = 28;
int waypoints[waypointsNumber][2];
int sendingWaypointsIndex = 0;

int uploadingWaypointsIndex = 0;

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

float lineA, lineB, lineC = 1;
int currentWaypoint = 0;

int inp1, inp2, inp3, inp4, inp5, inp6;

int requestStatus = 0;
int requestTarget = 0;
int requestType = 0;
float requestValue = 0;
bool requestComma = false;
int requestCommaPlace = 0;

long leftColumnLon = 0;
double leftColumnLat = 0;

int visLeftColumnLon = 550;
int visLeftColumnLat = 700;

double scaleEW = 1;
double scaleNS = 1;

double angle;

int visualizationScale = 10;

int globalToLocalX(long lon, long lat){
  double point[2];
  point[0] = (double)lon/10000000 - leftColumnLon;
  point[1] = (double)lat/10000000 - leftColumnLat;

  point[0] *= scaleEW * visualizationScale;
  point[1] *= scaleNS * visualizationScale;

  point[0] = point[0] * cos(angle) - point[1] * sin(angle);
  point[1] = point[0] * sin(angle) + point[1] * sin(angle);

  point[0] += visLeftColumnLon;
  point[1] += visLeftColumnLat;
  
  return (int) point[0];
}

int globalToLocalY(long lon, long lat){
  double point[2];
  point[0] = (double)lon/10000000 - leftColumnLon;
  point[1] = (double)lat/10000000 - leftColumnLat;

  point[0] *= scaleEW * visualizationScale;
  point[1] *= scaleNS * visualizationScale;

  point[0] = point[0] * cos(angle) - point[1] * sin(angle);
  point[1] = point[0] * sin(angle) + point[1] * sin(angle);

  point[0] += visLeftColumnLon;
  point[1] += visLeftColumnLat;
  
  return (int) point[1];
}

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
    case 2:
      //send waypoints
      COM.println();
      for(int i = 0; i < waypointsNumber; i++){
        COM.print(waypoints[i][0]);
        COM.print(" ");
        COM.println(waypoints[i][1]);
      }
      break;
    default:
      COM.print("Query ");
      COM.print(query);
      COM.println(" is not defined.");
  }
}

void changeValue(int target, float value){
  bool okMessage = true;
  switch(target) {
    case 0:
      if((int)value == 0){
        sendingData = false;
      }else{
        sendingData = true;
      }
      break;
    case 1:
      sendingWaypointsIndex = (int) value;
      break;
    case 2:
      waypoints[sendingWaypointsIndex/2][sendingWaypointsIndex%2] = (int) value;
      EEPROM.writeInt(sendingWaypointsIndex*2+1, (int) value);
      break;
    case 3:
      if((int)value == 0){
        leftColumnLon = longitude;
        EEPROM.writeLong(114, leftColumnLon);
        COM.println();
        COM.println("Left column longitude updated");
      }
      break;
    case 4:
      if((int)value == 0){
        leftColumnLat = latitude;
        EEPROM.writeLong(118, leftColumnLat);
        COM.println();
        COM.println("Left column latitude updated");
      }
      break;
    default:
      okMessage = false;
      COM.print("Value to change ");
      COM.print(target);
      COM.println(" is not defined.");
  }
  if(okMessage){
    COM.println();
    COM.println("#8|1");
  }
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

float getAngle(float p1x,float  p1y, float p2x, float p2y, float p3x, float p3y){
    float x1 = p2x-p1x;
    float y1 = p2y-p1y;
    float x2 = p3x-p2x;
    float y2 = p3y-p2y;
    float angle = acos((x1*x2 + y1*y2)/(sqrt(x1*x1+y1*y1)*sqrt(x2*x2+y2*y2)))*180/3.14;
    if(x1*y2-x2*y1 > 0){
        angle *= -1;
    }
    return angle;
}

int distance(int p1x, int p1y, int p2x, int p2y){
    float x1 = p2x-p1x;
    float y1 = p2y-p1y;
    return sqrt(x1*x1+y1*y1); 
}

int mathAbs(int x){
  if(x >= 0){
    return x;
  }
  return x*-1;
}

float getPitot() {
  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  unsigned int T_dat;
  byte Temp_L;
  byte Temp_H;
  address = 0x28;
  Serial.print("x ");
  Serial.println(millis());
  Wire.requestFrom((int) address, (int) 4);
  Serial.print("d ");
  Serial.println(millis());
  long millis_start = millis();
  bool ok = true;
  while (Wire.available() < 4) {
    if (millis() - millis_start > 2) {
      ok = false;
      break;
    }
  }
  if (ok) {
    Press_H = Wire.read();
    Press_L = Wire.read();
    Temp_H = Wire.read();
    Temp_L = Wire.read();
  }
  Wire.endTransmission(false);
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

  COM.addMemoryForWrite(comBuffer, sizeof(comBuffer));
  
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
    WIRE_PORT.setClock(400000);
    
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
    /*success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
    */
    if (success){
      COM.println();
      COM.println("#4|1");
    }else{
      COM.println();
      COM.println("#4|0");
      //SCB_AIRCR = 0x05FA0004;
      while(true);
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

    for(int i=0; i < waypointsNumber; i++){
      waypoints[i][0] = EEPROM.readInt(i*4+1);
      waypoints[i][1] = EEPROM.readInt(i*4+3);
    }
    leftColumnLon = EEPROM.readLong(114);
    leftColumnLat = EEPROM.readLong(118);
    
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
    Serial.print("receiver ");
    Serial.println(millis());
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
    Serial.print("heartbeat ");
    Serial.println(millis());
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

  if(!emergency && sendingData && millis() - sendDataTimer > sendDataInterval){
    Serial.print("sending ");
    Serial.print(millis());
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
    Serial.print("-");
    Serial.println(millis());
    sendDataTimer = millis();
  }

  if(!emergency){
    Serial.print("icm ");
    Serial.println(millis());
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
    Serial.print("gps ");
    Serial.println(millis());
    if (myGNSS.getPVT() && (myGNSS.getInvalidLlh() == false)){
      latitude = myGNSS.getLatitude();
      longitude = myGNSS.getLongitude();
      GPSaltitude = myGNSS.getAltitude();
      SIV = myGNSS.getSIV();
      gspeed = myGNSS.getGroundSpeed();
      heading = heading = (float)((float)myGNSS.getHeading())/100000.0;

      planeX = globalToLocalX(longitude, latitude);
      planeY = globalToLocalY(longitude, latitude);

      int currentSide = 0;
      if(lineA*planeX + lineB*planeY + lineC > 0){
        currentSide = 1;
      }
      
      if(previousSide != currentSide){
        currentWaypoint++;
        int previousWaypoint = currentWaypoint-1;
        if(previousWaypoint < 0){
          previousWaypoint = waypointsNumber - 1;
        }
        int nextWaypoint = currentWaypoint++;
        if(nextWaypoint >= waypointsNumber){
          nextWaypoint = 0;
        }
        //A - prev, B - next, C - current, D - tmp
        float AcBcRatio = distance(waypoints[previousWaypoint][0], waypoints[previousWaypoint][1], waypoints[currentWaypoint][0], waypoints[currentWaypoint][1])/distance(waypoints[nextWaypoint][0], waypoints[nextWaypoint][1], waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);
        int ADx = (waypoints[nextWaypoint][0] - waypoints[previousWaypoint][0]) * AcBcRatio;
        int ADy = (waypoints[nextWaypoint][1] - waypoints[previousWaypoint][1]) * AcBcRatio;

        int Dx = waypoints[previousWaypoint][0] + ADx;
        int Dy = waypoints[previousWaypoint][1] + ADy;

        lineA = (waypoints[currentWaypoint][1] - Dy)/(waypoints[currentWaypoint][0] - Dx);
        lineB = 1;
        lineC = Dy - lineA*Dx;

        previousSide = 0;
        if(lineA*planeX + lineB*planeY + lineC > 0){
          previousSide = 1;
       }
      }

      
      gpsTimer = millis();      
    }
  }

  if(!emergency && millis() - sensorTimer > sensorInterval){
    Serial.print("pressure ");
    Serial.println(millis());
    float pressure = ps.readPressureMillibars();
    altiPress = ps.pressureToAltitudeMeters(pressure);
    Serial.print("pitot ");
    Serial.println(millis());
    vPitot = getPitot();
    sensorTimer = millis();
  }
}
