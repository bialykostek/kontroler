/*
#include "EEPROMex.h"
#include "Servo.h"
#include "ICM_20948.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "PulsePosition.h"
#include "Wire.h"
#include "TensorFlowLite.h"
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/c/c_api_internal.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "flight_model_18.h"

#define COM Serial3
#define WIRE_PORT Wire
#define AD0_VAL 1
#define pi 3.14159265


void* comBufferTX[700];
void* comBufferRX[700];

bool sendingData = true;

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

float historicvPitot[historicLength] = {
    0,
    0,
    0,
    0,
    0
};

float altiPress = 0;

bool resetFirstStep = false;

float vPitot;

float yawOffset = 0;

const int NNinpLen = 21;

float NNinput[NNinpLen];
float NNoutput[5];
float NNlearnOut[5] = {
    0,
    0,
    0,
    0,
    0
};

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
long NNdataTimer = 0;

long emergencyTimer = 0;
int emergencyCounter = 0;
int emergencySafety = 50;

PulsePositionInput ppm(RISING);

Servo servo_elevator;
Servo servo_rudder;
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

int inp1_elevator, inp2_leftail, inp3_rightail, inp4_rudder, inp5_esc, inp6_settings;

int requestStatus = 0;
int requestTarget = 0;
int requestType = 0;
float requestValue = 0;
bool requestComma = false;
int requestCommaPlace = 0;

long leftColumnLon = 0;
long leftColumnLat = 0;

long rightColumnLon = 0;
long rightColumnLat = 0;

int visLeftColumnLon = 550;
int visLeftColumnLat = 700;

float scaleEW = 1;
float scaleNS = 1;

float angle = 0;

int visualizationScale = 10;


void setup() {
    // heart beat
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // communication
    Serial.begin(115200);
    COM.begin(38400);

    // receiver
    ppm.begin(6);

    // Servos
    servo_elevator.attach(9);
    servo_rudder.attach(11);
    servo3.attach(23);
    servo4.attach(10);
    esc.attach(22, 1000, 2000);














    // //   servo_elevator.write(90);
    // //   servo_rudder.write(90);
    // //   servo3.write(90);
    // //   servo4.write(90);
    // //   esc.write(0);

    //   //start mode
    //   mode = EEPROM.read(0);

    //   if (mode == 0) {
    //     //normal start

    //     COM.println("#1|0");

    //     WIRE_PORT.begin();
    //     //WIRE_PORT.setClock(400000);

    //     //delay(2000);


    //     while (!myGNSS.begin()) {
    //       COM.println();
    //       COM.println("#5|0");
    //       delay(500);
    //     }

    //     myGNSS.setI2COutput(COM_TYPE_UBX);
    //     myGNSS.setNavigationFrequency(10);
    //     myGNSS.setAutoPVT(true);

    //     COM.println("#5|1");

    //     bool initialized = false;
    //     while (!initialized) {
    //       myICM.begin(WIRE_PORT, AD0_VAL);
    //       if (myICM.status != ICM_20948_Stat_Ok) {
    //         delay(500);
    //       } else {
    //         initialized = true;
    //       }
    //     }

    //     bool success = true;
    //     success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    //     success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    //     success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);         // Set to the maximum
    //     success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);         // Set to the maximum
    //     success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);          // Set to the maximum
    //     success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok);   // Set to the maximum
    //     success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok);         // Set to the maximum
    //     success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok);  // Set to the maximum
    //     success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    //     success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    //     success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    //     success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    //     if (success) {
    //       COM.println();
    //       COM.println("#3|1");
    //       COM.println("#4|1");
    //     } else {
    //       COM.println();
    //       COM.println("#4|0");
    //       delay(100);
    //       SCB_AIRCR = 0x05FA0004;
    //       while (true)
    //         ;
    //     }
    //     delay(1000);

    //     COM.addMemoryForWrite(comBufferTX, sizeof(comBufferTX));
    //     COM.addMemoryForRead(comBufferRX, sizeof(comBufferRX));



    //     for (int i = 0; i < waypointsNumber; i++) {
    //       waypoints[i][0] = EEPROM.readInt(i * 4 + 1);
    //       waypoints[i][1] = EEPROM.readInt(i * 4 + 3);
    //     }

    //     leftColumnLon = EEPROM.readLong(114);
    //     leftColumnLat = EEPROM.readLong(118);
    //     rightColumnLon = EEPROM.readLong(122);
    //     rightColumnLat = EEPROM.readLong(126);
    //     scaleNS = EEPROM.readFloat(130);
    //     scaleEW = EEPROM.readFloat(134);
    //     angle = EEPROM.readFloat(138);

    //     static tflite::MicroErrorReporter micro_error_reporter;
    //     error_reporter = &micro_error_reporter;

    //     model = tflite::GetModel(flight_model);
    //     if (model->version() != TFLITE_SCHEMA_VERSION) {
    //       Serial.println("Model version error");
    //       return;
    //     }

    //     static tflite::ops::micro::AllOpsResolver resolver;
    //     static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
    //     interpreter = &static_interpreter;

    //     TfLiteStatus allocate_status = interpreter->AllocateTensors();
    //     if (allocate_status != kTfLiteOk) {
    //       Serial.println("Allocation tensors error");
    //       return;
    //     }

    //     input = interpreter->input(0);
    //     output = interpreter->output(0);

    //     for (int l = 0; l < NNinpLen; l++) {
    //       NNinput[l] = 0;
    //     }

    //     for (int l = 0; l < 5; l++) {
    //       NNoutput[l] = 0;
    //     }

    //   } else {
    //     //emergency
    //     armed = true;
    //     emergency = true;
    //     COM.println();
    //     COM.println("#1|1");
    //   }

    //   COM.println("#14|1");
}

void loop() {

      long debugTimer = millis();

      checkForMessage();

      long debugTime = millis();


      if (emergency && millis() - emergencyTimer > 1000) {
        COM.println();
        COM.println("#7|1");
        emergencyTimer = millis();
      }



    //   if (millis() - receiverTimer > receiverInterval) {
    //     readReceiver();

    //     if (!emergency && armed) {
    //       if (millis() - emergencyTimer > emergencySafety) {
    //         emergencyCounter++;
    //         if (emergencyCounter >= 3) {
    //           emergency = true;
    //         }
    //       } else {
    //         emergencyCounter = 0;
    //       }
    //       emergencyTimer = millis();
    //     }
    //     if (emergency || inp6_settings < 100) {
    //       servo_elevator.write(inp1_elevator);
    //       servo_rudder.write(inp4_rudder);
    //       servo3.write(inp3_rightail);
    //       servo4.write(inp2_leftail);
    //       if (armed) {
    //         esc.write(inp5_esc);
    //       } else {
    //         esc.write(0);
    //       }
    //     }
    //     receiverTimer = millis();
    //   }



    //   if (millis() - heartbeatTimer > heartbeatInterval) {

    //     int val = (int)(sin(heartbeatValue) * 255);
    //     if (val < 0) {
    //       val = 0;
    //     }
    //     analogWrite(13, val);
    //     if (heartbeatValue >= 3.14) {
    //       heartbeatValue = 0;
    //     }
    //     heartbeatValue += 0.03;
    //     heartbeatTimer = millis();
    //   }

    //   if (!emergency && millis() - NNdataTimer > 50) {

    //     NNinput[2] = NNinput[0];  //roll
    //     NNinput[3] = NNinput[1];  //pitch

    //     NNinput[16] = NNlearnOut[0];  //pwm
    //     NNinput[17] = NNlearnOut[1];
    //     NNinput[18] = NNlearnOut[2];
    //     NNinput[19] = NNlearnOut[3];
    //     NNinput[20] = NNlearnOut[4];

    //     NNinput[0] = (roll + 180) / 360;
    //     NNinput[1] = (pitch + 180) / 360;

    //     NNinput[4] = (float)gspeed / 25000;
    //     NNinput[5] = ((float)GPSaltitude / 1000 - altiPressOffset) / 40;
    //     NNinput[6] = vPitot / 20;

    //     float tmp = angles[0];
    //     tmp *= -1;
    //     tmp += pi;
    //     if (tmp > 2 * pi) {
    //       tmp -= 2 * pi;
    //     }
    //     tmp /= 2 * pi;
    //     NNinput[7] = tmp;

    //     tmp = angles[1];
    //     tmp *= -1;
    //     tmp += pi;
    //     if (tmp > 2 * pi) {
    //       tmp -= 2 * pi;
    //     }
    //     tmp /= 2 * pi;
    //     NNinput[8] = tmp;

    //     tmp = angles[3];
    //     tmp *= -1;
    //     tmp += pi;
    //     if (tmp > 2 * pi) {
    //       tmp -= 2 * pi;
    //     }
    //     tmp /= 2 * pi;
    //     NNinput[10] = tmp;

    //     tmp = angles[4];
    //     tmp *= -1;
    //     tmp += pi;
    //     if (tmp > 2 * pi) {
    //       tmp -= 2 * pi;
    //     }
    //     tmp /= 2 * pi;
    //     NNinput[11] = tmp;

    //     tmp = angles[5];
    //     tmp *= -1;
    //     tmp += pi;
    //     if (tmp > 2 * pi) {
    //       tmp -= 2 * pi;
    //     }
    //     tmp /= 2 * pi;
    //     NNinput[12] = tmp;

    //     tmp = angles[6];
    //     tmp *= -1;
    //     tmp += pi;
    //     if (tmp > 2 * pi) {
    //       tmp -= 2 * pi;
    //     }
    //     tmp /= 2 * pi;
    //     NNinput[13] = tmp;

    //     tmp = angles[7];
    //     tmp *= -1;
    //     tmp += pi;
    //     if (tmp > 2 * pi) {
    //       tmp -= 2 * pi;
    //     }
    //     tmp /= 2 * pi;

    //     NNinput[14] = tmp;

    //     NNinput[9] = (angles[2] + pi) / 2 / pi;

    //     NNinput[15] = distanceToCurrent / 500;

    //     if (inp6_settings > 105) {

    //       //float x[27] = {0.49,0.51,0.49,0.51,0.00,-0.50,3.92,0.80,0.25,0.24,0.53,0.58,0.57,0.57,0.63,0.50,0.85,0.85,0.85,0.85,0.85};
    //       //!!!-5
    //       for (int k = 0; k < NNinpLen; k++) {
    //         input->data.f[k] = NNinput[k];
    //       }

    //       TfLiteStatus invoke_status = interpreter->Invoke();
    //       if (invoke_status != kTfLiteOk) {
    //         emergency = true;
    //       }


    //       for (int p = 0; p < 5; p++) {
    //         NNoutput[p] = output->data.f[p];
    //       }

    //       servo_elevator.write(NNoutput[0] * 60 + 60);
    //       servo_rudder.write(NNoutput[1] * 60 + 60);
    //       servo3.write(NNoutput[2] * 60 + 60);
    //       servo4.write(NNoutput[3] * 60 + 60);
    //       if (armed) {
    //         esc.write(NNoutput[4] * 180);
    //       } else {
    //         esc.write(0);
    //       }

    //       for (int i = 0; i < NNinpLen; i++) {
    //         Serial.print(NNinput[i]);
    //         Serial.print(">");
    //       }
    //       Serial.println();

    //       Serial.print(NNoutput[0] * 60 + 60);
    //       Serial.print(" ");
    //       Serial.print(NNoutput[1] * 60 + 60);
    //       Serial.print(" ");
    //       Serial.print(NNoutput[2] * 60 + 60);
    //       Serial.print(" ");
    //       Serial.print(NNoutput[3] * 60 + 60);
    //       Serial.print(" ");
    //       Serial.print(NNoutput[4] * 180);
    //       Serial.println(" ");
    //       NNlearnOut[0] = NNoutput[0];
    //       NNlearnOut[1] = NNoutput[1];
    //       NNlearnOut[2] = NNoutput[2];
    //       NNlearnOut[3] = NNoutput[3];
    //       NNlearnOut[4] = NNoutput[4];


    //     } else {

    //       NNlearnOut[0] = ((float)inp1_elevator - 60) / 60;
    //       NNlearnOut[1] = ((float)inp2_leftail - 60) / 60;
    //       NNlearnOut[2] = ((float)inp3_rightail - 60) / 60;
    //       NNlearnOut[3] = ((float)inp4_rudder - 60) / 60;
    //       NNlearnOut[4] = (float)inp5_esc / 180;
    //     }

    //     NNdataTimer = millis();
    //   }

    //   if (!emergency && inp6_settings > 0 && ((sendingData && millis() - sendDataTimer > sendDataInterval) || sendOne)) {
    //     long debug = millis();
    //     COM.print("$");
    //     COM.print(inp1_elevator);
    //     COM.print(",");
    //     COM.print(inp2_leftail);
    //     COM.print(",");
    //     COM.print(inp3_rightail);
    //     COM.print(",");
    //     COM.print(inp4_rudder);
    //     COM.print(",");
    //     COM.print(inp5_esc);
    //     COM.print(",");
    //     COM.print(inp6_settings);
    //     COM.print(",");
    //     COM.print(roll);
    //     COM.print(",");
    //     COM.print(pitch);
    //     COM.print(",");
    //     COM.print(yaw);
    //     COM.print(",");

    //     COM.print(latitude);
    //     COM.print(",");
    //     COM.print(longitude);
    //     COM.print(",");
    //     COM.print((float)GPSaltitude / 1000 - altiPressOffset);

    //     COM.print(",");
    //     COM.print(SIV);
    //     COM.print(",");
    //     COM.print(gspeed);
    //     COM.print(",");
    //     COM.print(heading);
    //     COM.print(",");
    //     COM.print(altiPress);
    //     COM.print(",");
    //     COM.print(vPitot);

    //     COM.print(",");
    //     COM.print(lineA);
    //     COM.print(",");
    //     COM.print(lineB);
    //     COM.print(",");
    //     COM.print(lineC);
    //     COM.print(",");
    //     COM.print(planeX);
    //     COM.print(",");
    //     COM.print(planeY);
    //     COM.print(",");
    //     COM.print(currentWaypoint);
    //     COM.print(",");

    //     COM.print(angles[0]);
    //     COM.print(",");
    //     COM.print(angles[1]);
    //     COM.print(",");
    //     COM.print(angles[2]);
    //     COM.print(",");
    //     COM.print(angles[3]);
    //     COM.print(",");
    //     COM.print(angles[4]);
    //     COM.print(",");
    //     COM.print(angles[5]);
    //     COM.print(",");
    //     COM.print(angles[6]);
    //     COM.print(",");
    //     COM.print(angles[7]);
    //     COM.print(",");
    //     COM.print(distanceToCurrent);
    //     COM.print(",");
    //     COM.print(frameNumber);
    //     COM.print(",");

    //     for (int i = 0; i < NNinpLen; i++) {
    //       COM.print(NNinput[i]);
    //       COM.print(">");
    //     }

    //     COM.print(NNlearnOut[0]);
    //     COM.print(">");
    //     COM.print(NNlearnOut[1]);
    //     COM.print(">");
    //     COM.print(NNlearnOut[2]);
    //     COM.print(">");
    //     COM.print(NNlearnOut[3]);
    //     COM.print(">");
    //     COM.print(NNlearnOut[4]);


    //     COM.println();
    //     sendDataTimer = millis();
    //     frameNumber++;
    //     sendOne = false;
    //   }

    //   if (!emergency) {
    //     icm_20948_DMP_data_t data;
    //     myICM.readDMPdataFromFIFO(&data);

    //     if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    //       if ((data.header & DMP_header_bitmap_Quat6) > 0) {
    //         double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
    //         double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
    //         double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
    //         double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    //         double q2sqr = q2 * q2;
    //         double t0 = +2.0 * (q0 * q1 + q2 * q3);
    //         double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);

    //         float tmproll = atan2(t0, t1) * 180.0 / PI;
    //         if (!isnan(tmproll)) {
    //           roll = tmproll;
    //         }
    //         double t2 = +2.0 * (q0 * q2 - q3 * q1);
    //         t2 = t2 > 1.0 ? 1.0 : t2;
    //         t2 = t2 < -1.0 ? -1.0 : t2;
    //         float tmppitch = asin(t2) * 180.0 / PI;
    //         if (!isnan(tmppitch)) {
    //           pitch = tmppitch;
    //         }
    //         double t3 = +2.0 * (q0 * q3 + q1 * q2);
    //         double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
    //         float tmpyaw = atan2(t3, t4);
    //         if (!isnan(tmpyaw)) {
    //           yaw = tmpyaw;
    //         }
    //         yaw += pi;
    //         yaw -= angle;

    //         if (yaw < 0) {
    //           yaw += 2 * pi;
    //         }

    //         yaw -= yawOffset;

    //         if (yaw < 0) {
    //           yaw += 2 * pi;
    //         }
    //         float tmpPointX;
    //         if (yaw < pi / 2 || yaw > pi * 3 / 2) {
    //           tmpPointX = planeX + 100;
    //         } else {
    //           tmpPointX = planeX - 100;
    //         }
    //         float tmpPointY = tmpPointX * tan(yaw) + planeY - tan(yaw) * planeX;
    //         int nextWaypoint = currentWaypoint + 1;
    //         if (nextWaypoint > waypointsNumber) {
    //           nextWaypoint = 0;
    //         }
    //         angles[0] = getAngleVectors(tmpPointX, tmpPointY, planeX, planeY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);

    //         angles[2] = heading - yaw;
    //         if (angles[2] > pi) {
    //           angles[2] = angles[2] - 2 * pi;
    //         } else if (angles[2] < -pi) {
    //           angles[2] = angles[2] + 2 * pi;
    //         }
    //       }
    //     }
    //   }
    //   debugTimer = millis();



    //   if (!emergency && myGNSS.getPVT(10) && myGNSS.getInvalidLlh() == false) {


    //     latitude = myGNSS.getLatitude();
    //     longitude = myGNSS.getLongitude();
    //     GPSaltitude = myGNSS.getAltitude();
    //     SIV = myGNSS.getSIV();
    //     gspeed = myGNSS.getGroundSpeed();
    //     heading = heading = (float)((float)myGNSS.getHeading()) / 100000.0;


    //     if (inp6_settings > 105) {
    //       GPSaltitude -= 20000;
    //     }

    //     heading = 360 - heading;
    //     heading = heading * pi / 180;

    //     heading -= angle;
    //     if (heading < 0) {
    //       heading += 2 * pi;
    //     }
    //     heading -= 3 * pi / 2;
    //     if (heading < 0) {
    //       heading += 2 * pi;
    //     }

    //     planeX = globalToLocal(longitude, latitude, 0);
    //     planeY = globalToLocal(longitude, latitude, 1);

    //     int currentSide = 0;
    //     if (lineA * planeX + lineB * planeY + lineC > 0) {
    //       currentSide = 1;
    //     }

    //     if (previousSide != currentSide) {
    //       currentWaypoint++;
    //       if (currentWaypoint >= waypointsNumber) {
    //         currentWaypoint = 0;
    //       }
    //       calculateLine();
    //     }

    //     int nextWaypoint = currentWaypoint + 1;
    //     if (nextWaypoint > waypointsNumber) {
    //       nextWaypoint = 0;
    //     }

    //     angles[1] = -pi + getAngleVectors(planeX, planeY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1], waypoints[nextWaypoint][0], waypoints[nextWaypoint][1]);
    //     distanceToCurrent = distance(planeX, planeY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);

    //     float tmpPointX;
    //     if (yaw < pi / 2 || yaw > pi * 3 / 2) {
    //       tmpPointX = planeX + 100;
    //     } else {
    //       tmpPointX = planeX - 100;
    //     }
    //     float tmpPointY = tmpPointX * tan(yaw) + planeY - tan(yaw) * planeX;

    //     angles[0] = getAngleVectors(tmpPointX, tmpPointY, planeX, planeY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);

    //     angles[2] = heading - yaw;
    //     if (angles[2] > pi) {
    //       angles[2] = angles[2] - 2 * pi;
    //     } else if (angles[2] < -pi) {
    //       angles[2] = angles[2] + 2 * pi;
    //     }

    //     //gpsTimer = millis();
    //   }



    //   if (!emergency && millis() - sensorTimer > sensorInterval) {
    //     if (readSensorNumber == 0) {
    //       readSensorNumber++;
    //     } else if (readSensorNumber == 1) {
    //       vPitot = getPitot();

    //       for (int i = 1; i < historicLength; i++) {
    //         historicvPitot[i - 1] = historicvPitot[i];
    //       }
    //       vPitot = getPitot();
    //       historicvPitot[historicLength - 1] = vPitot;

    //       float tmpSum = 0;
    //       for (int i = 0; i < historicLength; i++) {
    //         tmpSum += historicvPitot[i];
    //       }
    //       vPitot = tmpSum / historicLength;

    //       sensorTimer = millis();
    //       readSensorNumber = 0;
    //     }
    //   }
}
*/