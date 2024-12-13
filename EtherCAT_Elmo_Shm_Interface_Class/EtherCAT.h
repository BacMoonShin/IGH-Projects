#ifndef _ETHERCAT_H_
#define _ETHERCAT_H_

#include "EL2889.h"
#include "HC_SV660N.h"
#include "Elmo.h"
#include "SharedMemory.hpp"
#include "SharedMemoryDataType.hpp"


extern bool EtherCAT_Running_Flag;

class EtherCAT_Thread
{
    public:
        EtherCAT_Thread();
        ~EtherCAT_Thread();
        void EtherCAT_Start();
    private:
        EtherCAT_Master Master;
        ELMO_SDO_Data SDO_Data = {1, 1, 3, 2, 131072, 10000, 1, 1, 0, CSP};
        ELMO Elmo1;
        ELMO Elmo2;

        SharedMemory LinerMotorCmdMemory;
        SharedMemory LinerMotorStateMemory;
        AllLineMotorsCmdData allLineMotorsCmdData;
        AllLineMotorsStateData allLineMotorsStateData;

        uint16_t printCount = PRINT_COUNT;
        // uint16_t UpdateCount = UPDATE_COUNT;
        struct timespec cycleTime = {0, TASK_PERIOD_NS};
        struct timespec wakeupTime;
        struct timespec endTime;
        struct timespec actualTime;
        struct timespec actualPeriodTime;
};

#endif