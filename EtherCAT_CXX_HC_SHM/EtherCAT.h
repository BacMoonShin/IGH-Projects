#ifndef _ETHERCAT_H_
#define _ETHERCAT_H_

#include <signal.h>
#include "EL2889.h"
#include "HC_SV660N.h"
#include "sharedMemory.h"

extern bool EtherCAT_Running_Flag;

class EtherCAT_Thread
{
    public:
        EtherCAT_Thread(SharedMemory& SharedMemory);
        void EtherCAT_Start();
    private:
        EtherCAT_Master Master;
        HC_SV660N_Motor Motor;
        HC_SV660N_Sdo_Data SV660N_1_SDO_Data;
        HC_SV660N SV660N;

        SharedMemory sharedMemory;

        uint16_t PrintCount = PRINT_COUNT;
        uint16_t UpdateCount = UPDATE_COUNT;
        struct timespec cycleTime = {0, TASK_PERIOD_NS};
        struct timespec wakeupTime;
        struct timespec endTime;
        struct timespec actualTime;
        struct timespec actualPeriodTime;
};

#endif