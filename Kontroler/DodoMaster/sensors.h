#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "control.h"

void rc_init();
control_data_t rc_read();

bool gps_init();
void gps_read();

#endif // _SENSORS_H_