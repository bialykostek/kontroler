#include "sensors.h"
#include "control.h"
#include "PulsePosition.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

PulsePositionInput rc_receiver(RISING);
SFE_UBLOX_GNSS gps_module;

// ***** STATUS VARIABLES *****
bool gps_initialized = false;

void rc_init() {
    rc_receiver.begin(6);
}

control_data_t rc_read() {
    control_data_t output;
    output.elevator = map(rc_receiver.read(2), 1100, 1900, 75, 105);
    output.leftail = map(rc_receiver.read(1), 1100, 1900, 80, 110);
    output.rightail = map(rc_receiver.read(5), 1100, 1900, 75, 105);
    output.rudder = map(rc_receiver.read(4), 1100, 1900, 30, 130);
    output.esc = map(rc_receiver.read(3), 1100, 1900, 0, 180);
    output.settings = map(rc_receiver.read(6), 1100, 1900, 0, 180);
    return output;
}

bool gps_init() {
    if (gps_module.begin()) {
        gps_module.setI2COutput(COM_TYPE_UBX);
        gps_module.setNavigationFrequency(10);
        gps_module.setAutoPVT(true);
        
        gps_initialized = true;
        return true;
    }
    return false;
}

void gps_read(){

}

// ICM_20948_I2C myICM;

// float getPitot() {
//     byte address, Press_H, Press_L, _status;
//     unsigned int P_dat;
//     unsigned int T_dat;
//     byte Temp_L;
//     byte Temp_H;
//     address = 0x28;
//     Wire.requestFrom((int) address, (int) 4);
//     long millis_start = millis();
//     bool ok = true;
//     while (Wire.available() < 4) {
//         if (millis() - millis_start > 2) {
//             ok = false;
//             break;
//         }
//     }
//     if (ok) {
//         Press_H = Wire.read();
//         Press_L = Wire.read();
//         Temp_H = Wire.read();
//         Temp_L = Wire.read();
//     }
//     Wire.endTransmission(false);
//     _status = (Press_H >> 6) & 0x03;
//     Press_H = Press_H & 0x3f;
//     P_dat = (((unsigned int) Press_H) << 8) | Press_L;
//     Temp_L = (Temp_L >> 5);
//     T_dat = (((unsigned int) Temp_H) << 3) | Temp_L;
//     double PR = (double)((P_dat - 819.15) / (14744.7));
//     PR = (PR - 0.49060678);
//     if (PR < 0) {
//         PR *= -1;
//     }
//     double V = ((PR * 13789.5144) / 1.225);
//     float VV = (sqrt((V)));
//     return VV;
// }
