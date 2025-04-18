#ifndef _CONTROL_H_
#define _CONTROL_H_

struct control_data_t {
    int elevator;
    int rudder;
    int leftail;
    int rightail;
    int esc;
    int settings;
};

void control_init();
void control_set(control_data_t values);

#endif  // _CONTROL_H_