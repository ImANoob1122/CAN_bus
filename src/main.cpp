#include "mbed.h"
#include <CAN.h>
#include "Motor.hpp"

CAN can1(PB_8, PB_9, 1000000);
static UnbufferedSerial esp32(PA_0, PA_1, 115200);
DigitalOut led(LED1);
char data[8] = {0};
Motor Lmotor(can1, 1, data, true);
char buf[2];
char str[64];
unsigned int test;

void read_esp() {
    static int i = 0;
    esp32.read(buf, 2);
    if (i >= 64) {
        i = 0;
        memset(str, '\0', sizeof(str));
    }
    if (buf[0] == '\n') {
        i=0;
        sscanf(str, "0x%04x\n", &test);
        memset(str, '\0', sizeof(str));
    }
    else {
        led = !led;
        str[i] = buf[0];
        i++;
    }
}


int main() {
    printf("start");
    esp32.attach(&read_esp, UnbufferedSerial::RxIrq);
    Lmotor.set_current(100);

    }