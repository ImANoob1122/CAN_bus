#include <mbed.h>

static UnbufferedSerial esp32(PA_9, PA_10, 115200);
DigitalOut led(LED1);
PwmOut l_motor(PA_0);
PwmOut r_motor(PA_1);
//PwmOut arm();
DigitalOut chack1(PA_6);
DigitalOut chack2(PA_7);
DigitalOut chack3(PA_8);
int speed;
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

void setup() {
     for(float i=0.16f; i<=0.38; i+=0.01f) {
         r_motor.write(i-0.03);
         l_motor.write(i);
         ThisThread::sleep_for(1000ms);
     }
     for(float i=0.38f; i>=0.16; i-=0.01f) {
         r_motor.write(i);
         l_motor.write(i-0.03);
         ThisThread::sleep_for(1000ms);
     }
    r_motor.write(0.22);
}

int main() {
    speed = 1500;
    esp32.attach(&read_esp, UnbufferedSerial::RxIrq);
    l_motor.period(0.002f);
    r_motor.period(0.002f);
    //arm.period_ms(5);

    setup();
    
    while(true) {
        printf("%x\n", test);
        switch(test){
            case 0:
                l_motor.write(0.23f);
                r_motor.write(0.27f);
                //arm.pulsewidth(0);
                break;
            case 8:
                l_motor.write(0.19f);
                r_motor.write(0.32f);
                break;
            case 1:
                l_motor.write(0.29f);
                r_motor.write(0.22f);
                break;
            case 2:
                l_motor.write(0.19f);
                r_motor.write(0.22f);
                break;
            case 4:
                l_motor.write(0.29f);
                r_motor.write(0.32f); 
                break;               
            case 10: 
                   
                //chack1.write(0);
                //chack2.write(0);
                //chack3.write(0);
                
                break;
            case 20:
                //chack1.write(1);
                //chack2.write(1);
                //chack3.write(1);
                break;
            case 80:
                //arm.pulsewidth(1600);
            case 40:
                //arm.pilsewidth(1400);
                break;
        }
        
    }
    
}