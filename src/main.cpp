#include "mbed.h"
#include "CAN.h"
#include "Motor.hpp"

CAN can1(PB_8, PB_9, 1000000);
Motor aaleft(can1, 1);

int main() {
    aaleft.set_target_speed(10);
    printf("test");
}