#include <mbed.h>

static UnbufferedSerial esp32(PA_9, PA_10, 115200);
DigitalOut led(LED1);
PwmOut l_motor(PA_0);
PwmOut r_motor(PA_1);
PwmOut Movingarm(PA_2);
DigitalOut chack1(PA_6);
DigitalOut chack2(PA_7);
DigitalOut chack3(PA_8);
char buf[2];
char str[64];
unsigned int test;

//モーターなどの変数
float Lmotor_middle = 0.23f;    //モーターのPWMはこれを中心に+-11のみ受け付ける
float Rmotor_middle = 0.27f;
float arm_middle = 0.0f;
float speed = 0.05f; //モーターを動かす速度




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

//キャリブレーションってやつ
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
}

//モーターの動作を確認する
void check_middle(PwmOut& checking_motor, int pre_middle) {
    while(true) {
        int k = pre_middle - 11;
        for(float i = (pre_middle - 11)/100.0f; i<=pre_middle + 11; i += 0.01f) {
            checking_motor.write(i);
            printf("%d\n", k);
            k++;
            ThisThread::sleep_for(1000ms);
        }
        k = pre_middle + 11;
        for(float i = (pre_middle + 11)/100.0f; i>=pre_middle - 11; i -= 0.01f) {
            checking_motor.write(i);
            printf("%d\n", k);
            k--;
            ThisThread::sleep_for(1000ms);
        }
    }
}

//ロボットの停止
void stop() {
    l_motor.write(Lmotor_middle);
    r_motor.write(Rmotor_middle);
}

//ロボットの直進
void forward() {
    l_motor.write(Lmotor_middle-speed);
    r_motor.write(Rmotor_middle+speed);
}

//ロボットの後退
void back() {
    l_motor.write(Lmotor_middle+speed);
    r_motor.write(Rmotor_middle-speed);
}

//ロボットの右回転
void Rturn() {
    l_motor.write(Lmotor_middle-speed);
    r_motor.write(Rmotor_middle-speed);
}

//ロボットの左回転
void Lturn() {
    l_motor.write(Lmotor_middle+speed);
    r_motor.write(Rmotor_middle+speed);
}

int main() {
    esp32.attach(&read_esp, UnbufferedSerial::RxIrq);
    l_motor.period(0.002f);
    r_motor.period(0.002f);
    Movingarm.period(0.002f);

    check_middle(Movingarm, 27); //アームを動かす前にこれを実行して！　中心が分かったらarm_middleを更新してこの行を消すこと

    setup();

    int phase = 0;
    
    while(true) {
        printf("%x\n", test);
        switch(test){
            case 0:
                stop();
                Movingarm.write(arm_middle);
                break;
            case 8:
                forward();
                break;
            case 1:
                back();
                break;
            case 2:
                Rturn();
                break;
            case 4:
                Lturn();
                break;               
            case 10: //自動化部分　自分好みに調整しよう！
                if (phase == 1) {
                    forward();
                    ThisThread::sleep_for(1000ms);//時間を変更することで上の動作の時間を変えられるよ
                    phase++;
                } else if(phase == 2) {
                    Rturn();
                    ThisThread::sleep_for(1000ms);
                    phase++;
                } else if(phase == 3) {
                    forward();
                    ThisThread::sleep_for(1000ms);
                    phase++;
                } else if(phase == 4) {
                    chack1.write(1);
                    chack2.write(1);
                    chack3.write(1);
                    ThisThread::sleep_for(1000ms);
                    phase++;
                } else if (phase == 5) {
                    back();
                    ThisThread::sleep_for(1000ms);
                    phase++;
                } else if (phase == 6) {
                    Lturn();
                    ThisThread::sleep_for(1000ms);
                    phase++;
                } else if (phase == 7) {
                    forward();
                    ThisThread::sleep_for(1000ms);
                    phase++;
                }
                break;
            case 20:
                //chack1.write(1);
                //chack2.write(1);
                //chack3.write(1);
                break;
            case 80:
                Movingarm.write(arm_middle + speed);
                break;
            case 40:
                Movingarm.write(arm_middle - speed);
                break;
        }
        
    }
    
}