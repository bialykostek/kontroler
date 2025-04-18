/*
 * od strony krawedzi:
 * ESC
 * wysokosc
 * lewe
 * kierunek
 * prawe
*/

#include <EEPROMex.h>
#include "PWM.hpp"
#include <Servo.h>
#include "ICM_20948.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <LPS.h>
#include <TensorFlowLite.h>

#define COM Serial3
#define WIRE_PORT Wire
#define AD0_VAL 1
#define pi 3.14159265

#include "second_model.h"
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/c/c_api_internal.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"


SFE_UBLOX_GNSS myGNSS;
ICM_20948_I2C myICM;
LPS ps;

namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  TfLiteTensor* output = nullptr;
  constexpr int kTensorArenaSize = 20 * 1024;
  uint8_t tensor_arena[kTensorArenaSize];
}

void* comBufferTX[700];
void* comBufferRX[700];

bool sendingData = false;

int receiverInterval = 20;
int heartbeatInterval = 7;
int sendDataInterval = 50;
int sensorInterval = 50;
int packageInterval = 10;
float heartbeatValue = 0;

int readSensorNumber = 0;

int packageNumber = 0;

float altiPressOffset = 0;

bool sendOne = false;
double yaw, pitch, roll;
long frameNumber = 0;
float angles[8];
float distanceToCurrent;

int planeX, planeY = 0;
int previousSide = 0;

int mode = 0;
bool armed = false;

bool sendingModeNormal = true;

bool emergency = false;

const int historicLength = 5;

float historicvPitot[historicLength] = {0, 0, 0, 0, 0};
float historicAltiPress[historicLength] = {0, 0, 0, 0, 0};


float altiPress;

bool resetFirstStep = false;

float vPitot;

float yawOffset = 0;

int NNinpLen = 27;

float NNlearnOut[5] = {0, 0, 0, 0, 0};

const int waypointsNumber = 28;
int waypoints[waypointsNumber][2];
int sendingWaypointsIndex = 0;

int uploadingWaypointsIndex = 0;

long heartbeatTimer = 0;
long receiverTimer = 0;
long sendDataTimer = 0;
long gpsTimer = 0;
long sensorTimer = 0;
long packageTimer = 0;
long NNtimer = 0;

long emergencyTimer = 0;
int emergencyCounter = 0;
int emergencySafety = 50;


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
long leftColumnLat = 0;

float NNinputs[29];

long rightColumnLon = 0;
long rightColumnLat = 0;

int visLeftColumnLon = 550;
int visLeftColumnLat = 700;

float scaleEW = 1;
float scaleNS = 1;

float angle = 0;

int visualizationScale = 10;

int globalToLocal(long lon, long lat, int ind){
  double point[2];

  point[0] = (double)lon/10000000 - (double)leftColumnLon/10000000;
  point[1] = (double)lat/10000000 - (double)leftColumnLat/10000000;

  point[0] *= scaleEW * visualizationScale;
  point[1] *= scaleNS * visualizationScale;
  double tmpPoint0 = point[0];
  
  point[0] = point[0] * cos(-angle) - point[1] * sin(-angle);
  point[1] = tmpPoint0 * sin(-angle) + point[1] * cos(-angle);


  point[0] += visLeftColumnLon;
  point[1] += visLeftColumnLat;

  return (int) point[ind];
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
    case 3:
      COM.println();
      COM.print("#9|");
      COM.print(leftColumnLon);
      COM.print(";");
      COM.println(leftColumnLat);
      break;
    case 4:
      COM.println();
      COM.print("#10|");
      COM.print(rightColumnLon);
      COM.print(";");
      COM.println(rightColumnLat);
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
        leftColumnLat = latitude;
        EEPROM.writeLong(114, leftColumnLon);
        EEPROM.writeLong(118, leftColumnLat);
        COM.println();
        COM.println("Left column updated");
      }
      break;
    case 4:
      if((int)value == 0){
        rightColumnLon = longitude;
        rightColumnLat = latitude;
        EEPROM.writeLong(122, rightColumnLon);
        EEPROM.writeLong(126, rightColumnLat);
        COM.println();
        COM.println("Right column updated");
      }
      break;
    case 5:
      scaleNS = value;
      EEPROM.writeFloat(130, scaleNS);
      COM.println();
      COM.println("Scale NS updated");
      break;
    case 6:
      scaleEW = value;
      EEPROM.writeFloat(134, scaleEW);
      COM.println();
      COM.println("Scale EW updated");
      break;
    case 7:
      angle = value;
      EEPROM.writeFloat(138, angle);
      COM.println();
      COM.println("Angle updated");
      break;
    case 8:
      currentWaypoint = (int) value;
      calculateLine();
      break;
    case 9:
      yawOffset = yaw;
      COM.println("Compass calibrated");
      break;
    case 10:
      if((int)value == 0){
        sendingModeNormal = true;
      }
      if((int)value == 1){
        sendingModeNormal = false;
      }
      COM.println("Sending data mode changed");
      break;
    case 11:
      myGNSS.setNavigationFrequency((int)value);
      break;
    default:
      okMessage = false;
      COM.print("Value to change ");
      COM.print(target);
      COM.println(" is not defined.");
      COM.clear();
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
      altiPressOffset = (float)GPSaltitude/1000;
      EEPROM.write(0, 1);
      break;
    case 1:
      //disarm
      armed = false;
      EEPROM.write(0, 0);
      break;
    case 2:
      sendOne = true;
      break;
    case 3:
      resetFirstStep = true;
      break;
    case 4:
      if(resetFirstStep){
        SCB_AIRCR = 0x05FA0004;
      }
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

float getAngleVectors(float p1x, float p1y, float p2x, float p2y, float p3x, float p3y){
  float vec1x = p1x-p2x;
  float vec1y = p1y-p2y;

  float vec2x = p3x-p2x;
  float vec2y = p3y-p2y;

  float result = acos((vec1x*vec2x+vec1y*vec2y)/sqrt(vec1x*vec1x+vec1y*vec1y)/sqrt(vec2x*vec2x+vec2y*vec2y));
  if(vec1x*vec2y - vec1y*vec2x < 0){
    result *= -1;
  }
  return result;
}

float getAngle(int p1, int p2, int p3){
  return getAngleVectors(waypoints[p1][0], waypoints[p1][1], waypoints[p2][0], waypoints[p2][1], waypoints[p3][0], waypoints[p3][1]);
}

float getAnglePoints(float p1x, float p1y, float p2x, float p2y, float p3x, float p3y){
  float a = distance(p1x, p1y, p2x, p2y);
  float b = distance(p2x, p2y, p3x, p3y);
  float c = distance(p1x, p1y, p3x, p3y);

  float angleOut = acos((a*a + b*b - c*c)/(2*a*b));
  
  angleOut = pi - angleOut;

  if((p2x - p1x)*(p3y - p2y) - (p2y - p2y)*(p3x - p2x) > 0){
    angleOut *= -1;
  }
  
  return angleOut;
}

float distance(int p1x, int p1y, int p2x, int p2y){
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

void calculateLine(){
  int previousWaypoint = currentWaypoint-1;
  if(previousWaypoint < 0){
    previousWaypoint = waypointsNumber - 1;
  }
  int nextWaypoint = currentWaypoint+1;
  if(nextWaypoint >= waypointsNumber){
    nextWaypoint = 0;
  }
  //A - prev, B - next, C - current, D - tmp
  float AcBcRatio = distance(waypoints[previousWaypoint][0], waypoints[previousWaypoint][1], waypoints[currentWaypoint][0], waypoints[currentWaypoint][1])/distance(waypoints[nextWaypoint][0], waypoints[nextWaypoint][1], waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);
  float AB = distance(waypoints[previousWaypoint][0], waypoints[previousWaypoint][1], waypoints[nextWaypoint][0], waypoints[nextWaypoint][1]);
  float AD = AB/(1+1/AcBcRatio);
  float AdAbRatio = AD/AB;
  float ADx = (waypoints[nextWaypoint][0] - waypoints[previousWaypoint][0]) * AdAbRatio;
  float ADy = (waypoints[nextWaypoint][1] - waypoints[previousWaypoint][1]) * AdAbRatio;

  float Dx = waypoints[previousWaypoint][0] + ADx;
  float Dy = waypoints[previousWaypoint][1] + ADy;

  lineA = -(waypoints[currentWaypoint][1] - Dy)/(waypoints[currentWaypoint][0] - Dx);
  lineB = 1;
  lineC = -Dy - lineA*Dx;

  previousSide = 0;
  if(lineA*waypoints[previousWaypoint][0] + lineB*waypoints[previousWaypoint][1] + lineC > 0){
    previousSide = 1; 
  }
  int p1 = currentWaypoint;
  int p2 = currentWaypoint+1;
  if(p2 >= waypointsNumber){
    p2 -= waypointsNumber;
  }
  int p3 = currentWaypoint+2;
  if(p3 >= waypointsNumber){
    p3 -= waypointsNumber;
  }
  for(int i=3; i<8; i++){
    if(p1 >= waypointsNumber){
      p1 = 0;
    }
    if(p2 >= waypointsNumber){
      p2 = 0;
    }
    if(p3 >= waypointsNumber){
      p3 = 0;
    }
    angles[i] = -pi + getAngle(p1, p2, p3);
    p1++;
    p2++;
    p3++;
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
  inp1 = map(ch4.getValue(), 1100, 1900, 30, 150);
  inp2 = map(ch2.getValue(), 1100, 1900, 40, 160);
  inp3 = map(ch5.getValue(), 1100, 1900, 30, 150);
  inp4 = map(ch3.getValue(), 1100, 1900, 30, 130);
  inp5 = map(ch1.getValue(), 1100, 1900, 0, 180);
  inp6 = map(ch6.getValue(), 1100, 1900, 0, 180);
}

void setup() {
  //heart
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  //communication
  Serial.begin(115200);
  COM.begin(38400);

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

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  esc.write(0);

   for(int i=0; i<NNinpLen; i++){
      NNinputs[i] = 0;
    }

    static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  
  model = tflite::GetModel(g_second_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model version error");
    return;
  }
  
  static tflite::ops::micro::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("Allocation tensors error");
    return;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  //start mode
  mode = EEPROM.read(0);
  if(mode == 0){
    //normal start
  
   COM.println("#1|0");

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
    
    delay(2000);

    bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  

    
    bool success = true;
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
    
    if (success){
      COM.println();
      COM.println("#3|1");
      COM.println("#4|1");
    }else{
      COM.println();
      COM.println("#4|0");
      delay(100);
      SCB_AIRCR = 0x05FA0004;
      while(true);
    }
    delay(1000);

    COM.addMemoryForWrite(comBufferTX, sizeof(comBufferTX));
    COM.addMemoryForRead(comBufferRX, sizeof(comBufferRX));
    
    while(!myGNSS.begin()){
      COM.println();
      COM.println("#5|0");
      delay(500);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.setNavigationFrequency(12);
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
    rightColumnLon = EEPROM.readLong(122);
    rightColumnLat = EEPROM.readLong(126);
    scaleNS = EEPROM.readFloat(130);
    scaleEW = EEPROM.readFloat(134);
    angle = EEPROM.readFloat(138);

 
  
     
  }else{
    //emergency
    armed = true;
    emergency = true;
    COM.println();
    COM.println("#1|1");
    
  }


  
  

  COM.println("#14|1");
}



void loop(){
  long debugTimer = millis();
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
    if(inp6 < 75 || inp6 > 105){
      servo1.write(inp1);
      servo2.write(inp2);
      servo3.write(inp3);
      servo4.write(inp4);
      if(armed){
        esc.write(inp5);
      }else{
        esc.write(0);  
      }
    }
    receiverTimer = millis();
  }

  if(inp6 > 75 && inp6 < 105 && millis()-NNtimer > 50){

      float test1[27] = {0.58, 0.51, 0.57, 0.51, 0.59, 0.48, 0.23, 0.59, 0.48, 0.22, 0.18 -0.3, 0.53, 0.18 -0.31, 0.53, 0.43, 0.43, 0.37, 0.43, 0.43, 0.33, 0.44, 0.42, 0.64, 0.49, 0.48};  
    for(int l=0; l <27; l++){
      NNinputs[l] = test1[l];
    }
  
  Serial.print(sizeof(input->data.f));
  Serial.print(" ");
  Serial.print(input->params.scale);
  Serial.print(" ");
  Serial.println(input->params.zero_point);

  
  
  for(int i=0; i <27; i++){
    input->data.f[i] = NNinputs[i];
  }

  for(int l=0; l <27; l++){
      Serial.print(input->data.f[l]);
      Serial.print(" ");
    }
    Serial.println();
  
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Model invoke error");
    return;
  }
  
  NNlearnOut[0] = output->data.f[0];
  NNlearnOut[1] = output->data.f[1];
  NNlearnOut[2] = output->data.f[2];
  NNlearnOut[3] = output->data.f[3];
  NNlearnOut[4] = output->data.f[4];
  

/*
    servo1.write(NNlearnOut[0]*60+60);
      servo2.write(NNlearnOut[1]*60+60);
      servo3.write(NNlearnOut[2]*60+60);
      servo4.write(NNlearnOut[3]*60+60);
      if(armed){
        esc.write(NNlearnOut[4]*180);
      }else{
        esc.write(0);  
      }

    */
    Serial.print(NNlearnOut[0]);
    Serial.print(", ");
    Serial.print(NNlearnOut[1]);
    Serial.print(", ");
    Serial.print(NNlearnOut[2]);
    Serial.print(", ");
    Serial.print(NNlearnOut[3]);
    Serial.print(", ");
    Serial.println(NNlearnOut[4]);
    NNtimer = millis();
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

  
  
  if(!emergency && (inp6 < 15 || inp6 > 30) && ((sendingData && millis() - sendDataTimer > sendDataInterval) || sendOne)){
   // if(sendingModeNormal == true){

NNinputs[2] = NNinputs[0]; //roll
      NNinputs[3] = NNinputs[1]; //pitch

      NNinputs[7] = NNinputs[4]; //ground speed
      NNinputs[8] = NNinputs[5]; //alti press
      NNinputs[9] = NNinputs[6]; //vpitot

      NNinputs[13] = NNinputs[10]; //angles 0
      NNinputs[14] = NNinputs[11]; //angles 1
      NNinputs[15] = NNinputs[12]; //angles 2

      NNinputs[22] = NNlearnOut[0]; //pwm
      NNinputs[23] = NNlearnOut[1];
      NNinputs[24] = NNlearnOut[2];
      NNinputs[25] = NNlearnOut[3];
      NNinputs[26] = NNlearnOut[4];

      NNinputs[0] = (roll+180)/360;
      NNinputs[1] = (pitch+180)/360;
      
      NNinputs[4] = (float)gspeed/25000;
      NNinputs[5] = ((float)GPSaltitude/1000-altiPressOffset)/40;
      NNinputs[6] = vPitot/20;

      float tmp = angles[0];
      tmp *= -1;
      tmp += pi;
      if(tmp > 2*pi){
        tmp -= 2*pi;
      }
      NNinputs[10] = tmp;

      tmp = angles[1];
      tmp *= -1;
      tmp += pi;
      if(tmp > 2*pi){
        tmp -= 2*pi;
      }
      NNinputs[11] = tmp;

      tmp = angles[3];
      tmp *= -1;
      tmp += pi;
      if(tmp > 2*pi){
        tmp -= 2*pi;
      }
      NNinputs[16] = tmp;

      tmp = angles[4];
      tmp *= -1;
      tmp += pi;
      if(tmp > 2*pi){
        tmp -= 2*pi;
      }
      NNinputs[17] = tmp;

      tmp = angles[5];
      tmp *= -1;
      tmp += pi;
      if(tmp > 2*pi){
        tmp -= 2*pi;
      }
      NNinputs[18] = tmp;

      tmp = angles[6];
      tmp *= -1;
      tmp += pi;
      if(tmp > 2*pi){
        tmp -= 2*pi;
      }
      NNinputs[19] = tmp;
    
      tmp = angles[7];
      tmp *= -1;
      tmp += pi;
      if(tmp > 2*pi){
        tmp -= 2*pi;
      }
      NNinputs[20] = tmp;

      NNinputs[12] = (angles[2]+pi)/2/pi;
      
      NNinputs[21] = distanceToCurrent/500;

      if(inp6 < 75 || inp6 > 105){
        NNlearnOut[0] = ((float)inp1-60)/60;
        NNlearnOut[1] = ((float)inp2-60)/60;
        NNlearnOut[2] = ((float)inp3-60)/60;
        NNlearnOut[3] = ((float)inp4-60)/60;
        NNlearnOut[4] = (float)inp5/180;
      }
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
          COM.print((float)GPSaltitude/1000-altiPressOffset);

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
         
          COM.print(",");
          COM.print(lineA);
          COM.print(",");
          COM.print(lineB);
          COM.print(",");
          COM.print(lineC);
          COM.print(",");
          COM.print(planeX);
          COM.print(",");
          COM.print(planeY);
          COM.print(",");
          COM.print(currentWaypoint);
          COM.print(",");
         
         COM.print(angles[0]);
          COM.print(",");
          COM.print(angles[1]);
          COM.print(",");
          COM.print(angles[2]);
          COM.print(",");
          COM.print(angles[3]);
          COM.print(",");
          COM.print(angles[4]);
          COM.print(",");
          COM.print(angles[5]);
          COM.print(",");
          COM.print(angles[6]);
          COM.print(",");
          COM.print(angles[7]);
          COM.print(",");
          COM.print(distanceToCurrent);
          COM.print(",");
          COM.print(frameNumber);
          
          
     
  
COM.print(",");
      for(int i=0; i<NNinpLen; i++){
        COM.print(NNinputs[i]);
        COM.print(">");
      }

      COM.print(NNlearnOut[0]);
      COM.print(">");
      COM.print(NNlearnOut[1]);
      COM.print(">");
      COM.print(NNlearnOut[2]);
      COM.print(">");
      COM.print(NNlearnOut[3]);
      COM.print(">");
      COM.print(NNlearnOut[4]);
 
      
          COM.println();
          sendDataTimer = millis();
          frameNumber++;
          sendOne = false;

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
        yaw = atan2(t3, t4);
        yaw += pi;
        yaw -= angle;
        
        if(yaw < 0){
          yaw += 2*pi;
        }

        yaw -= yawOffset;
        
        if(yaw < 0){
          yaw += 2*pi;
        }
         float tmpPointX;
      if(yaw < pi/2 || yaw > pi*3/2){
        tmpPointX = planeX + 100;
      }else{
        tmpPointX = planeX - 100;
      }
        float tmpPointY = tmpPointX*tan(yaw) + planeY - tan(yaw)*planeX;
      int nextWaypoint = currentWaypoint + 1;
      if(nextWaypoint > waypointsNumber){
        nextWaypoint = 0;
      }
        angles[0] = getAngleVectors(tmpPointX, tmpPointY, planeX, planeY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);


        angles[2] = heading - yaw;
        if(angles[2] > pi){
          angles[2] = angles[2] - 2*pi;
        }else if(angles[2] < -pi){
          angles[2] = angles[2] + 2*pi;
        }
      }
    }
  }
 debugTimer = millis();
    if (millis()-gpsTimer>100 && myGNSS.getPVT(5)){
      latitude = myGNSS.getLatitude(5);
      longitude = myGNSS.getLongitude(5);
      GPSaltitude = myGNSS.getAltitude(5);
      SIV = myGNSS.getSIV(5);
      gspeed = myGNSS.getGroundSpeed(5);
      heading = heading = (float)((float)myGNSS.getHeading(5))/100000.0;

      heading = 360 - heading;
      heading = heading*pi/180;

      heading -= angle;
      if(heading < 0){
        heading += 2*pi;
      }
      heading -= 3*pi/2;
      if(heading < 0){
        heading += 2*pi;
      }

      planeX = globalToLocal(longitude, latitude, 0);
      planeY = globalToLocal(longitude, latitude, 1);

      int currentSide = 0;
      if(lineA*planeX +lineB*planeY + lineC > 0){
        currentSide = 1;
      }
      
      if(previousSide != currentSide){
        currentWaypoint++;
        if(currentWaypoint >= waypointsNumber){
          currentWaypoint = 0;
        }
        calculateLine();
      }
     

     int nextWaypoint = currentWaypoint + 1;
      if(nextWaypoint > waypointsNumber){
        nextWaypoint = 0;
      }
      
      angles[1] = -pi + getAngleVectors(planeX, planeY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1], waypoints[nextWaypoint][0], waypoints[nextWaypoint][1]);
      distanceToCurrent = distance(planeX, planeY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);

  float tmpPointX;
      if(yaw < pi/2 || yaw > pi*3/2){
        tmpPointX = planeX + 100;
      }else{
        tmpPointX = planeX - 100;
      }
        float tmpPointY = tmpPointX*tan(yaw) + planeY - tan(yaw)*planeX;

      angles[0] = getAngleVectors(tmpPointX, tmpPointY, planeX, planeY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);


        angles[2] = heading - yaw;
        if(angles[2] > pi){
          angles[2] = angles[2] - 2*pi;
        }else if(angles[2] < -pi){
          angles[2] = angles[2] + 2*pi;
        }
      
      gpsTimer = millis();      
    }

  if(!emergency && millis() - sensorTimer > sensorInterval ){
    if(readSensorNumber == 0){
      
      float pressure = ps.readPressureMillibars();
      if(pressure != 0){
        for(int i=1; i < historicLength; i++){
          historicAltiPress[i-1] = historicAltiPress[i];
        }
        altiPress = ps.pressureToAltitudeMeters(pressure);
        historicAltiPress[historicLength-1] = altiPress;

        float tmpSum = 0;
        for(int i=0; i < historicLength; i++){
          tmpSum += historicAltiPress[i];
        }
        altiPress = tmpSum/historicLength;
        
      }
      
      readSensorNumber++;
    }else if(readSensorNumber == 1){
      vPitot = getPitot();


      for(int i=1; i < historicLength; i++){
          historicvPitot[i-1] = historicvPitot[i];
        }
       vPitot = getPitot();
        historicvPitot[historicLength-1] = vPitot;

        float tmpSum = 0;
        for(int i=0; i < historicLength; i++){
          tmpSum += historicvPitot[i];
        }
        vPitot = tmpSum/historicLength;
        
      
      sensorTimer = millis();
      readSensorNumber = 0;  


      
    }
    
  }
 
}
