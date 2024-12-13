#ifndef _ETHERCAT_H_
#define _ETHERCAT_H_

#include "EL2889.h"
#include "HC_SV660N.h"
#include "Elmo.h"
#include "SharedMemory.hpp"
#include "SharedMemoryDataType.hpp"

extern bool EtherCAT_Running_Flag;

void EtherCAT_Start();

#endif