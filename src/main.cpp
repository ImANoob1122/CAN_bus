#include "mbed.h"
#include <CAN.h>
#include "Motor.hpp"

CAN can1(PB_8, PB_9, 1000000);


int main() {
    printf("start");
    char data[8] = {0};
    Motor Lmotor(can1, 1, data, true);
    Lmotor.set_current(100);
    printf("test");
}