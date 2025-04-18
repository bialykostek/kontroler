#include "control.h"
#include "Servo.h"

Servo servo_elevator;
Servo servo_rudder;
Servo servo_leftail;
Servo servo_rightail;
Servo esc_speed;

void control_init() {
    servo_elevator.attach(9);
    servo_rudder.attach(11);
    servo_leftail.attach(23);
    servo_rightail.attach(10);
    esc_speed.attach(22, 1000, 2000);
}

void control_set(control_data_t values) {
    servo_elevator.write(values.elevator);
    servo_rudder.write(values.rudder);
    servo_leftail.write(values.leftail);
    servo_rightail.write(values.rightail);
    esc_speed.write(values.esc);
}