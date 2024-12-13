#include "EL2889.h"
#include "HC_SV660N.h"
#include "Elmo.h"

static bool EtherCAT_Running_Flag = true;
static void EtherCAT_Exit(int signal){EtherCAT_Running_Flag = false;}

int main(int argc, char **argv)
{
    EtherCAT_Master Master;
    ELMO_SDO_Data SDO_Data = {1, 1, CSP};
    ELMO Elmo1(Master, 0, 0, SDO_Data);
    ELMO Elmo2(Master, 0, 1, SDO_Data);

    Master.EtherCAT_Master_Config();
    ETHERCAT_MESSAGE("*EtherCAT Master and Slaves config success*");
    signal(SIGINT, EtherCAT_Exit);

    uint16_t printCount = PRINT_COUNT;
    // uint16_t updateCount = UPDATE_COUNT;
    struct timespec cycleTime = {0, TASK_PERIOD_NS};
    struct timespec wakeupTime, endTime, actualPeriodTime, actualTime;
	clock_gettime(CLOCK_REALTIME, &wakeupTime);

    while (EtherCAT_Running_Flag)
    {
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeupTime, NULL);
#ifdef ENABLE_DC
        ecrt_master_application_time(Master.master, TIMESPEC2NS(wakeupTime));
        ecrt_master_sync_reference_clock(Master.master);
#endif

        // 接收过程数据
        Master.EtherCAT_Master_Receive();

#ifdef DEBUG
        Master.EtherCAT_Check_States();
        Elmo1.Check_Motor_State(1);
        Elmo2.Check_Motor_State(2);
#endif
        if(printCount == 0)
        {
            if(Elmo1.data.drive_state == dsOperationEnabled)
            {
                ETHERCAT_INFO("Elmo1: act position = %d, act torque = %d, status = 0x%x, drive_state = %d, drive_mode = %d",
                              Elmo1.domain_data.TxPDO.current_position, Elmo1.domain_data.TxPDO.torque_input, Elmo1.domain_data.TxPDO.status_word, Elmo1.data.drive_state, Elmo1.data.drive_mode);
            }
            if(Elmo2.data.drive_state == dsOperationEnabled)
            {
                ETHERCAT_INFO("Elmo2: act position = %d, act torque = %d, status = 0x%x, drive_state = %d, drive_mode =%d",
                              Elmo2.domain_data.TxPDO.current_position, Elmo2.domain_data.TxPDO.torque_input, Elmo2.domain_data.TxPDO.status_word, Elmo2.data.drive_state, Elmo2.data.drive_mode);
            }
            printCount = PRINT_COUNT;
        }
        else
        {
            printCount--;
        }

        /* ***************** ELMO 1 ******************** */
        // 轮询检测是否在进行复位、急停或者使能
        // 复位或急停
        if (Elmo1.data.reset_busy == true)
        {
            // 控制驱动器的状态转化到Switch On Disabled
            Elmo1.Reset_Motor_State();

            //该处急停状态调用Reset_Motor_State()实际上不符合标准，是一种简略的写法
            //实际急停状态应该将控制字6040H写入0x0002进入快速停机状态dsQuickStopActive，快速停机状态之后根据快速停机方式605AH自动切换状态，605AH为0~3自动进入伺服无故障状态dsSwitchOnDisabled，605AH为5~7自动进入伺服运行状态dsOperationEnabled，默认状态下快速停机方式605AH值为2
        }
        // 急停
        else if(Elmo1.data.quickstop_busy == true)
        {
            Elmo1.Motor_Quickstop();
        }
        // 使能
        else if (Elmo1.data.power_busy == true)
        {
            Elmo1.Enable_Motor(CSP);
        }

        else if(Elmo1.data.drive_state == dsSwitchOnDisabled)
        {
            Elmo1.Reset_Motor_State();
        }

        // 检测没有处于复位、急停或者使能状态，则进行运动逻辑
        if(Elmo1.data.drive_state == dsOperationEnabled && Elmo1.data.reset_busy == false && Elmo1.data.power_busy == false && Elmo1.data.quickstop_busy == false)
        {
            if (Elmo1.data.drive_mode  == CSP)
            { // 位置模式
                if (Elmo1.data.home_busy == true)
                {
                    Elmo1.Move_Motor_Home();
                }
                else
                {
                    if (Elmo1.data.position_move_enable == true)
                    {// 开始运动
                        if(MOVE_DIRECTION)
                        {
                            Elmo1.domain_data.RxPDO.target_position += MOVE_VELOCITY;
                        }
                        else
                        {
                        Elmo1.domain_data.RxPDO.target_position -= MOVE_VELOCITY;
                        }
                        // if(updateCount == 0)
                        // {
                        //     if(0)
                        //     {
                        //         // Elmo1.domain_data.RxPDO.target_position += (int32_t)((ENCODER_RESOLUTION * MOVE_VELOCITY) / TASK_FREQUENCY);
                        //         // Elmo1.domain_data.RxPDO.target_position += (int32_t)((UPDATE_COUNT * SCREW_MOVE_VELOCITY) / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY);
                        //         Elmo1.domain_data.RxPDO.target_position += (int32_t)(10);
                        //     }
                        //     else
                        //     {
                        //         // Elmo1.domain_data.RxPDO.target_position -= (int32_t)((ENCODER_RESOLUTION * MOVE_VELOCITY) / TASK_FREQUENCY);
                        //         // Elmo1.domain_data.RxPDO.target_position -= (int32_t)((UPDATE_COUNT * SCREW_MOVE_VELOCITY) / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY);
                        //         Elmo1.domain_data.RxPDO.target_position -= (int32_t)(10);
                        //     }
                        //     updateCount = UPDATE_COUNT;
                        // }
                        // else
                        // {
                        //     updateCount--;
                        // }
                    }
                    else
                    {
                        Elmo1.domain_data.RxPDO.target_position = Elmo1.domain_data.TxPDO.current_position;
                    }
                    EC_WRITE_S32(Elmo1.domain_pd + Elmo1.domain_offset.RxPDO.target_position, Elmo1.domain_data.RxPDO.target_position);
                }
            }
        }
        /* ***************** ELMO 1 ******************** */


        /* ***************** ELMO 2 ******************** */
        // 轮询检测是否在进行复位、急停或者使能
        // 复位或急停
        if (Elmo2.data.reset_busy == true)
        {
            // 控制驱动器的状态转化到Switch On Disabled
            Elmo2.Reset_Motor_State();

            //该处急停状态调用Reset_Motor_State()实际上不符合标准，是一种简略的写法
            //实际急停状态应该将控制字6040H写入0x0002进入快速停机状态dsQuickStopActive，快速停机状态之后根据快速停机方式605AH自动切换状态，605AH为0~3自动进入伺服无故障状态dsSwitchOnDisabled，605AH为5~7自动进入伺服运行状态dsOperationEnabled，默认状态下快速停机方式605AH值为2
        }
        // 急停
        else if(Elmo2.data.quickstop_busy == true)
        {
            Elmo2.Motor_Quickstop();
        }
        // 使能
        else if (Elmo2.data.power_busy == true)
        {
            Elmo2.Enable_Motor(CSP);
        }

        else if(Elmo2.data.drive_state == dsSwitchOnDisabled)
        {
            Elmo2.Reset_Motor_State();
        }

        // 检测没有处于复位、急停或者使能状态，则进行运动逻辑
        if(Elmo2.data.drive_state == dsOperationEnabled && Elmo2.data.reset_busy == false && Elmo2.data.power_busy == false && Elmo2.data.quickstop_busy == false)
        {
            if (Elmo2.data.drive_mode  == CSP)
            { // 位置模式
                if (Elmo2.data.home_busy == true)
                {
                    Elmo2.Move_Motor_Home();
                }
                else
                {
                    if (Elmo2.data.position_move_enable == true)
                    {// 开始运动
                        if(MOVE_DIRECTION)
                        {
                            Elmo2.domain_data.RxPDO.target_position += MOVE_VELOCITY;
                        }
                        else
                        {
                            Elmo2.domain_data.RxPDO.target_position -= MOVE_VELOCITY;
                        }
                    }
                    else
                    {
                        Elmo2.domain_data.RxPDO.target_position = Elmo2.domain_data.TxPDO.current_position;
                    }
                    EC_WRITE_S32(Elmo2.domain_pd + Elmo2.domain_offset.RxPDO.target_position, Elmo2.domain_data.RxPDO.target_position);
                }
            }
        }
        /* ***************** ELMO 2 ******************** */


#ifdef ENABLE_DC
        clock_gettime(CLOCK_REALTIME, &actualTime);
        ecrt_master_sync_reference_clock_to(Master.master, TIMESPEC2NS(actualTime));
        ecrt_master_sync_slave_clocks(Master.master);
#endif

        Master.EtherCAT_Master_Send();

        clock_gettime(CLOCK_REALTIME, &endTime);
        actualPeriodTime = timespec_minus(endTime, wakeupTime);
        if(TIMESPEC2NS(actualPeriodTime) > TASK_PERIOD_NS && Elmo1.data.drive_state == dsOperationEnabled && Elmo2.data.drive_state == dsOperationEnabled)
        {
            ETHERCAT_WARNING("ActualPeriodTime is larger than task period!");
            // wakeupTime = timespec_add(wakeupTime, cycleTime);
            wakeupTime = endTime;
        }
        else
        {
            // ETHERCAT_INFO("actualPeriodTime is %d ns", TIMESPEC2NS(actualPeriodTime));
            wakeupTime = timespec_add(wakeupTime, cycleTime);
        }
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeupTime, NULL);
    Master.EtherCAT_Master_Exit();
    return 0;
}
