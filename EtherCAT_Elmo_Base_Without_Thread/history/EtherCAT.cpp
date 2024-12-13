#include "EtherCAT.h"

bool EtherCAT_Running_Flag = true;
static void EtherCAT_Stop(int signal) { EtherCAT_Running_Flag = false; }


EtherCAT_Thread::EtherCAT_Thread(SharedMemory& SharedMemory){
    HC_SV660N_Motor Motor = {MOTOR_ENCODER_RESOLUTION, MOTOR_RATED_TORQUE};
    HC_SV660N_SDO_Data SDO_Data = {2097152, 25};
    HC_SV660N SV660N(Master, 0, 0, 0x1702, 0x1B01, Motor, SDO_Data);
    sharedMemory = &SharedMemory;

    Master.EtherCAT_Master_Config();
    signal(SIGINT, EtherCAT_Stop);
    ETHERCAT_MESSAGE("*EtherCAT Master and Slaves config success*");
}

void EtherCAT_Thread::EtherCAT_Start() noexcept{
    clock_gettime(CLOCK_REALTIME, &wakeupTime);
    while (EtherCAT_Running_Flag)
    {
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeupTime, NULL);
        ecrt_master_application_time(Master.master, TIMESPEC2NS(wakeupTime));

        // 接收过程数据
        Master.EtherCAT_Master_Receive();

#ifdef DEBUG
        Master.EtherCAT_Check_States();
        SV660N.Check_Motor_State();
#endif

        if (printCount == 0)
        {
            if (SV660N.data.drive_state == dsOperationEnabled)
            {
                ETHERCAT_INFO("SV660N: act position = %d, act torque = %lf N/m, status = 0x%x, drive_state = %d, opmode = 0x%x",
                              SV660N.domain_data.TxPDO.actual_position, SV660N.data.actual_torque, SV660N.domain_data.TxPDO.status_word, SV660N.data.drive_state, SV660N.domain_data.RxPDO.operation_mode);
            }
            printCount = PRINT_COUNT;
        }
        else
        {
            printCount--;
        }

        // 轮询检测是否在进行复位、急停或者使能
        // 复位或急停
        if (SV660N.data.reset_busy == true)
        {
            // 控制驱动器的状态转化到Switch On Disabled
            SV660N.Reset_Motor_State();

            //该处急停状态调用Reset_Motor_State()实际上不符合标准，是一种简略的写法
            //实际急停状态应该将控制字6040H写入0x0002进入快速停机状态dsQuickStopActive，快速停机状态之后根据快速停机方式605AH自动切换状态，605AH为0~3自动进入伺服无故障状态dsSwitchOnDisabled，605AH为5~7自动进入伺服运行状态dsOperationEnabled，默认状态下快速停机方式605AH值为2
        }
        // 急停
        else if(SV660N.data.quickstop_busy == true)
        {
            SV660N.Motor_Quickstop();
        }
        // 使能
        else if (SV660N.data.power_busy == true)
        {
            SV660N.Enable_Motor(CSP);
        }

        // 检测没有处于复位、急停或者使能状态，则进行运动逻辑
        if (SV660N.data.drive_state == dsOperationEnabled && SV660N.data.reset_busy == 0 && SV660N.data.power_busy == 0 && SV660N.data.quickstop_busy == 0)
        {
            if (SV660N.domain_data.RxPDO.operation_mode == CSP)
            { // 位置模式
                if (SV660N.data.home_busy == true)
                {
                    SV660N.Move_Motor_Home();
                }
                else
                {
                    if (SV660N.data.position_move_enable == true)
                    {// 开始运动
                        if(updateCount == 0)
                        {
                            if(MOVE_DIRECTION)
                            {
                                // SV660N.domain_data.RxPDO.target_position += (int32_t)((ENCODER_RESOLUTION * MOVE_VELOCITY) / TASK_FREQUENCY);
                                SV660N.domain_data.RxPDO.target_position += (int32_t)(UPDATE_COUNT * SCREW_MOVE_VELOCITY / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY);
                            }
                            else
                            {
                                // SV660N.domain_data.RxPDO.target_position -= (int32_t)((ENCODER_RESOLUTION * MOVE_VELOCITY) / TASK_FREQUENCY);
                                SV660N.domain_data.RxPDO.target_position -= (int32_t)(UPDATE_COUNT * SCREW_MOVE_VELOCITY / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY);
                            }
                            updateCount = UPDATE_COUNT;
                        }
                        else
                        {
                            updateCount--;
                        }
                    }
                    else
                    {
                        SV660N.domain_data.RxPDO.target_position = SV660N.domain_data.TxPDO.actual_position;
                    }
                    EC_WRITE_S32(SV660N.domain_pd + SV660N.domain_offset.RxPDO.target_position, SV660N.domain_data.RxPDO.target_position);
                }
            }
        }
        clock_gettime(CLOCK_REALTIME, &actualTime);
  		ecrt_master_sync_reference_clock_to(Master.master, TIMESPEC2NS(actualTime));
  		ecrt_master_sync_slave_clocks(Master.master);

        Master.EtherCAT_Master_Send();

        clock_gettime(CLOCK_REALTIME, &endTime);
        actualPeriodTime = timespec_minus(endTime, wakeupTime);
        if (TIMESPEC2NS(actualPeriodTime) > TASK_PERIOD_NS && SV660N.data.drive_state == dsOperationEnabled)
        {
            ETHERCAT_WARNING("ActualPeriodTime is larger than task period!");
            wakeupTime = endTime;
        }
        else
        {
            ETHERCAT_INFO("actualPeriodTime is ");
            wakeupTime = timespec_add(wakeupTime, cycleTime);
        }
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeupTime, NULL);
    Master.EtherCAT_Master_Exit();
}

void EtherCAT_Exit() noexcept{
    EtherCAT_Running_Flag = false;
}