#ifndef CANMESSENGER_HPP
#define CANMESSENGER_HPP

#include "mbed.h"
#include <CAN.h>

class CANMessenger {
public:
    CANMessenger(CAN &can);
private:

};

#endif // CANMESSENGER_HPP