#include "PWM.hpp"
#include <Servo.h>
#include <Wire.h>
#include "ICM_20948.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <LPS.h>
#include <TensorFlowLite.h>

#include "map.h"
#include "model.h"
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#define SER Serial3
#define WIRE_PORT Wire
#define AD0_VAL 1

namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  TfLiteTensor* output = nullptr;
  int inference_count = 0;
  constexpr int kTensorArenaSize = 10 * 1024;
  uint8_t tensor_arena[kTensorArenaSize];
}

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

ICM_20948_I2C myICM;
LPS ps;
SFE_UBLOX_GNSS myGNSS;

int os1 = 0;
int os2 = 0;
int os3 = 0;
int os4 = 0;
int ose = 0;

int rs1 = 0;
int rs2 = 0;
int rs3 = 0;
int rs4 = 0;
int rse = 0;

int ns1 = 0;
int ns2 = 0;
int ns3 = 0;
int ns4 = 0;
int nse = 0;

float gX, gY, gZ = 0;
float roll, pitch, yaw = 0;

long sendT = 0;
byte sendP = 0;
long sensorT = 0;
long nnT = 0;

long latitude = 0;
long longitude = 0;
long alti = 0;
long gspeed = 0;
float heading = 0;
float altiPress = 0;
float vPitot = 0;

float angHY;
float angPN;
float angNN1;
float angNN2;
float angNN3;
float angNN4;
float angNN5;
float angHN;
float lenN;

bool ficek = true;
bool prev = 0;
float lineA, lineB, lineC;
int curr = 0;
bool calibration = false;
bool calibrationT = 0;

int absxd(int x);
float getDist(float p1x,float  p1y, float p2x, float p2y);
float getPitot();
float getAngle(float p1x,float  p1y, float p2x, float p2y, float p3x, float p3y);

void setup() {
  //SER
  SER.begin(38400);

  //RECEIVER AND SERVOS
  
  ch1.begin(true);
  ch2.begin(true);
  ch3.begin(true);
  ch4.begin(true);
  ch5.begin(true);
  ch6.begin(true);
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
  
  //WIRE
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  //IMU INIT
  bool initialized = false;
  while (!initialized){
    SER.println("IMU init");
    myICM.begin(Wire, 1);

    if (myICM.status != ICM_20948_Stat_Ok){
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
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

 if (success){
    SER.println(F("DMP enabled!"));
  }
  else{
    SER.println(F("Enable DMP failed!"));
    SER.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
  }
  
  //GPS 
  
  while (myGNSS.begin() == false) {
    SER.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    delay(100);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setI2CpollingWait(25);
myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(10); //Produce two solutions per second
  myGNSS.setAutoPVT(true);
  
  //MAP
  for(int i = 0; i<POINTSNUM; i++){
    points[i][0] = (pointsD[i][0] - zeroLat)*LatMet;
    points[i][1] = (pointsD[i][1] - zeroLong)*LongMet;
  }
  
  //LPS
  if (!ps.init()) {
    SER.println("Failed to autodetect pressure sensor!");
  }
  ps.enableDefault();

  //AI
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  model = tflite::GetModel(g_sine_model_data);
  static tflite::ops::micro::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    error_reporter->Report("AllocateTensors() failed");
    return;
  }
  input = interpreter->input(0);
  output = interpreter->output(0);
  
  //DEBUG
  SER.println("INIT DONE");
}

int settings = 0;
void loop() {
  //RECURSIVE
  if(settings < 150){
    rs1 = os1;
    rs2 = os2;
    rs3 = os3;
    rs4 = os4;
    rse = ose;
  }else{
    rs1 = ns1;
    rs2 = ns2;
    rs3 = ns3;
    rs4 = ns4;
    rse = nse;
  }
  
  //RECEIVER DATA
  
  int s1 = map(ch4.getValue(), 1100, 1900, 40, 140);
  int s2 = map(ch2.getValue(), 1100, 1900, 40, 140);
  int s3 = map(ch5.getValue(), 1100, 1900, 40, 140);
  int s4 = map(ch3.getValue(), 1100, 1900, 40, 140);
  int se = map(ch6.getValue(), 1100, 1900, 0, 180);
  settings = map(ch1.getValue(), 1100, 1900, 0, 180);
  
  
  if (s1 < 40) {
    s1 = 40;
  }
  if (s1 > 140) {
    s1 = 140;
  }
  if (s2 < 40) {
    s2 = 40;
  }
  if (s2 > 140) {
    s2 = 140;
  }
  if (s3 < 40) {
    s3 = 40;
  }
  if (s3 > 140) {
    s3 = 140;
  }
  if (s4 < 40) {
    s4 = 40;
  }
  if (s4 > 140) {
    s4 = 140;
  }
  
  //MANUAL CONTROLL
  
  if(settings < 150){
    if (absxd(os1 - s1) > 2) {
      servo1.write(s1);
      os1 = s1;
    }
    if (absxd(os2 - s2) > 2) {
      servo2.write(s2);
      os2 = s2;
    }
    if (absxd(os3 - s3) > 2) {
      servo3.write(s3);
      os3 = s3;
    }
    if (absxd(os4 - s4) > 2) {
      servo4.write(s4);
      os4 = s4;
    }
    if (absxd(ose - se) > 2) {
      esc.write(se);
      ose = se;
    }
  }
  

  //SENSORS
  if (millis() - sensorT > 50){

    //IMU DATA
  
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO( & data);
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      double q1 = ((double) data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double) data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double) data.Quat6.Data.Q3) / 1073741824.0;
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
    }
    if (myICM.dataReady())
    {
      myICM.getAGMT();         
      gX = myICM.gyrX();
      gY = myICM.gyrY();
      gZ = myICM.gyrZ();
    }
  }
  

  
    //GPS
    if(false && myGNSS.getPVT()){
      latitude = myGNSS.getLatitude();
      longitude = myGNSS.getLongitude();
      gspeed = myGNSS.getGroundSpeed();
      heading = (float)((float)myGNSS.getHeading())/100000.0;
//      heading -= 180;
//      heading *= -1;
//      heading -= 90;
//      if(heading < -180){
//        heading += 360;
//      }

      //ANGLES
      double posx, posy;
      bool pos = 0;
      posx = (((float)latitude)/10000000.0 - zeroLat)*LatMet;
      posy = (((float)longitude)/10000000.0 - zeroLong)*LongMet;
      
      if(lineA*posx + lineB*posy + lineC > 0){
          pos = 1;
      }
    
      if(pos != prev || ficek){
        curr++;
        ficek = false;
        double tpx = points[curr][0] + points[curr][1] - points[curr - 1][1];
        double tpy = points[curr][1] + points[curr - 1][0] - points[curr][0];
        
        //NEW LINE
        lineA = points[curr][1] - tpy;
        lineB = tpx - points[curr][0];
        lineC = tpy*points[curr][0] - points[curr][1]*tpx;
        
        if(lineA*points[curr - 1][0] + lineB*points[curr - 1][1] + lineC > 0){
          prev = 1;
        }
      }
      if(curr >= POINTSNUM-6){
        curr = 0;
        ficek = true;
      }
      
      float angTN = getAngle(posx+10, posy, posx, posy, points[curr][0], points[curr][1]);
      if(angTN < 0){
        angTN = 360 - angTN;
      }
  
      angPN = getAngle(posx, posy, points[curr][0], points[curr][1], points[curr+1][0], points[curr+1][1]);
      lenN = getDist(posx, posy, points[curr][0], points[curr][1]);
      angNN1 = getAngle(points[curr][0], points[curr][1], points[curr+1][0], points[curr+1][1], points[curr+2][0], points[curr+2][1]);
      angNN2 = getAngle(points[curr+1][0], points[curr+1][1], points[curr+2][0], points[curr+2][1], points[curr+3][0], points[curr+3][1]);
      angNN3 = getAngle(points[curr+2][0], points[curr+2][1], points[curr+3][0], points[curr+3][1], points[curr+4][0], points[curr+4][1]);
      angNN4 = getAngle(points[curr+3][0], points[curr+3][1], points[curr+4][0], points[curr+4][1], points[curr+5][0], points[curr+5][1]);
      angNN5 = getAngle(points[curr+4][0], points[curr+4][1], points[curr+5][0], points[curr+5][1], points[curr+6][0], points[curr+6][1]);
      
      float xhead = (heading-180)*-1;
      xhead -= 180;
      if(xhead < -180){
         xhead += 360;
      }
      angHN = angTN - xhead;
    
      if(angHN > 180){
        angHN = 360 - angHN;
      }
      if(angHN < -180){
        angHN += 360;
      }
      angHY = yaw - heading;
      if(angHY > 180){
        angHY = 360 - angHY;
      }
      if(angHY < -180){
        angHY += 360;
      }
    }
    
    
    //LPS
    Wire.endTransmission();
    float pressure = ps.readPressureMillibars();
    altiPress = ps.pressureToAltitudeMeters(pressure);

    //PITOT
    vPitot = getPitot();

    //TIMER
    sensorT = millis();
  }

  //AI
  if(settings > 60 && millis() - nnT > 30){
    input->data.f[0] = gX/255;
    input->data.f[1] = gY/255;
    input->data.f[2] = gZ/255;
    input->data.f[3] = (pitch +180)/360;
    input->data.f[4] = (roll +180)/360;
    input->data.f[5] = (float)((float)gspeed)/25000;
    input->data.f[6] = (altiPress-184)/15;
    input->data.f[7] = vPitot/20;
    input->data.f[8] = (float)(rs1-40)/100;
    input->data.f[9] = (float)(rs2-40)/100;
    input->data.f[10] = (float)(rs3-40)/100;
    input->data.f[11] = (float)(rs4-40)/100;
    input->data.f[12] = (float)rse/180;
    input->data.f[13] = (float)(angHY +180)/360;
    input->data.f[14] = (float)(angNN1 +180)/360;
    input->data.f[15] = (float)(angNN2 +180)/360;
    input->data.f[16] = (float)(angNN3 +180)/360;
    input->data.f[17] = (float)(angNN4 +180)/360;
    input->data.f[18] = (float)(angNN5 +180)/360;
    input->data.f[19] = (float)(angHN +180)/360;
    input->data.f[20] = (float)lenN/50;

    if(settings > 150){
      //AI CONTROL
      TfLiteStatus invoke_status = interpreter->Invoke();
      
      ns1 = output->data.f[0];
      ns2 = output->data.f[1];
      ns3 = output->data.f[2];
      ns4 = output->data.f[3];
      nse = output->data.f[4];
      
      SER.print(ns1, 4);
      SER.print(F(","));
      SER.print(ns2, 4);
      SER.print(F(","));
      SER.print(ns3, 4);
      SER.print(F(","));
      SER.print(ns4, 4);
      SER.print(F(","));
      SER.println(nse, 4);
    }
        
    nnT = millis();
  }
  
  //PRINT DATA
  if(settings < 60){
    switch(sendP){
      case 0:
        SER.print(latitude);
        SER.print(F(","));
        SER.print(longitude);
        SER.print(F(","));
        SER.print(os1);
        SER.print(F(","));
        SER.print(os2);
        SER.print(F(","));
        SER.print(os3);
        SER.print(F(","));
        sendP++;
        break;
      case 1:
        SER.print(os4);
        SER.print(F(","));
        SER.print(ose);
        SER.print(F(","));
        SER.print(roll);
        SER.print(F(","));
        SER.print(pitch);
        SER.print(F(", "));
        SER.print(yaw);
        SER.print(F(" ,"));
        sendP++;
        break;
      case 2:
        SER.print(gX);
        SER.print(F(","));
        SER.print(gY);
        SER.print(F(","));
        SER.print(gZ);
        SER.print(F(","));
        SER.print(gspeed);
        SER.print(F(", "));
        SER.print(heading);
        SER.print(F(" ,"));
        sendP++;
        break;
      case 3:
        SER.print(altiPress);
        SER.print(F(","));
        SER.print(vPitot);
        SER.print(F(","));
        sendP++;
        break;    
      default:
        SER.println();
        sendP = 0;
        SER.flush();
    }
  }
  if(settings > 60 && settings < 150){
    switch(sendP){
      case 0:
        SER.print(latitude);
        SER.print(F(","));
        SER.print(longitude);
        SER.print(F(","));
        SER.print(curr);
        SER.print(F(","));
        SER.print(input->data.f[0], 4);
        SER.print(F(","));
        SER.print(input->data.f[1], 4);
        SER.print(F(","));
        SER.print(input->data.f[2], 4);
        SER.print(F(","));
        SER.print(input->data.f[3], 4);
        SER.print(F(","));
        sendP++;
        break;
      case 1:
        SER.print(input->data.f[4], 4);
        SER.print(F(","));
        SER.print(input->data.f[5], 4);
        SER.print(F(","));
        SER.print(input->data.f[6], 4);
        SER.print(F(","));
        SER.print(input->data.f[7], 4);
        SER.print(F(","));
        SER.print(input->data.f[8], 4);
        SER.print(F(","));
        sendP++;
        break;
      case 2:
        SER.print(input->data.f[9], 4);
        SER.print(F(","));
        SER.print(input->data.f[10], 4);
        SER.print(F(","));
        SER.print(input->data.f[11], 4);
        SER.print(F(","));
        SER.print(input->data.f[12], 4);
        SER.print(F(","));
        SER.print(input->data.f[13], 4);
        SER.print(F(","));
        SER.print(input->data.f[14], 4);
        SER.print(F(","));
        sendP++;
        break;
      case 3:
        SER.print(input->data.f[15], 4);
        SER.print(F(","));
        SER.print(input->data.f[16], 4);
        SER.print(F(","));
        SER.print(input->data.f[17], 4);
        SER.print(F(","));
        SER.print(input->data.f[18], 4);
        SER.print(F(","));
        SER.print(input->data.f[19], 4);
        SER.print(F(","));
        SER.print(input->data.f[20], 4);
        SER.print(F(","));
        sendP++;
        break;
      default:
        SER.print((float)(os1-40)/100.0, 4);
        SER.print(F(","));
        SER.print((float)(os2-40)/100.0, 4);
        SER.print(F(","));
        SER.print((float)(os3-40)/100.0, 4);
        SER.print(F(","));
        SER.print((float)(os4-40)/100.0, 4);
        SER.print(F(","));
        SER.print((float)(ose)/180.0, 4);
        SER.print(F(","));
        SER.println();
        sendP = 0;
        SER.flush();
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

float getDist(float p1x,float  p1y, float p2x, float p2y){
    float x1 = p2x-p1x;
    float y1 = p2y-p1y;
    return sqrt(x1*x1+y1*y1); 
}

int absxd(int x){
  if(x >= 0){
    return x;
  }
  return x*-1;
}
