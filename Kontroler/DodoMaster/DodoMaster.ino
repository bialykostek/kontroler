#include "sensors.h"
#include "control.h"

#define INTERVAL_HEARTBEAT 17  // 60Hz
#define INTERVAL_RC 10         // 100Hz

enum flight_mode_t {
    MANUAL,
    EMERGENCY,
    AUTO
};

// ***** TIMERS *****
long timer_heartbeat = 0;
long timer_rc = 0;

// ***** STATUS *****
flight_mode_t flight_mode = MANUAL;


// ***** VARIABLES *****
control_data_t rc_data;
float heartbeat_position = 0;

void heartbeat() {
    int val = (int)(sin(heartbeat_position) * 255);
    if (val < 0) {
        val = 0;
    }

    analogWrite(13, val);

    if (heartbeat_position >= 3.14) {
        heartbeat_position = 0;
    }
    heartbeat_position += 0.06;
}

void setup() {
    // ***** INITIALIZATION *****

    // heartbeat LED
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // communication
    Serial.begin(115200);
    // COM.begin(38400);

    // RC receiver
    rc_init();

    // Control
    control_init();
    digitalWrite(13, LOW);

}

void loop() {

    if (millis() - timer_heartbeat > INTERVAL_HEARTBEAT) {
        heartbeat();
        timer_heartbeat = millis();
    }

    if (millis() - timer_rc > INTERVAL_RC) {
        rc_data = rc_read();

        if (flight_mode == MANUAL || flight_mode == EMERGENCY){
            control_set(rc_data);
        }

        timer_rc = millis();
    }
}