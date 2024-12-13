#include <signal.h>
#include "EL2889.h"
#include "HC_SV660N.h"

bool EtherCAT_Running_Flag = true;
static void EtherCAT_Exit(int signal){EtherCAT_Running_Flag = false;}

int main(int argc, char **argv)
{
    EtherCAT_Master Master;
    HC_SV660N_Motor Motor = {MOTOR_ENCODER_RESOLUTION, MOTOR_RATED_TORQUE};
    HC_SV660N_Sdo_Data SDO_Data = {2097152, 25};
    HC_SV660N SV660N(Master, 0, 0, Motor, SDO_Data);

    Master.EtherCAT_Master_Config();
    ETHERCAT_MESSAGE("*EtherCAT Master and Slaves config success*");
    signal(SIGINT, EtherCAT_Exit);

    uint16_t PrintCount = PRINT_COUNT;
    // uint16_t UpdateCount = UPDATE_COUNT;                 // CSP模式下不建议使用
    struct timespec cycleTime = {0, TASK_PERIOD_NS};
    struct timespec wakeupTime, endTime, actualPeriodTime, actualTime;
	clock_gettime(CLOCK_REALTIME, &wakeupTime);

    while (EtherCAT_Running_Flag)
    {
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeupTime, NULL);
#ifdef ENABLE_DC
        ecrt_master_application_time(Master.master, TIMESPEC2NS(wakeupTime));
#endif

        // 接收过程数据
        Master.EtherCAT_Master_Receive();

#ifdef DEBUG
        Master.EtherCAT_Check_States();
        SV660N.Check_Motor_State();
#endif

        if(PrintCount == 0)
        {
            if(SV660N.data.drive_state == dsOperationEnabled)
            {
                ETHERCAT_INFO("SV660N: act position = %d, act torque = %lf N/m, status = 0x%x, drive_state = %d, opmode = 0x%x",
                        SV660N.domain_data.TxPDO.current_position, SV660N.data.actual_torque, SV660N.domain_data.TxPDO.status_word, SV660N.data.drive_state, SV660N.domain_data.RxPDO.operation_mode);
            }
            PrintCount = PRINT_COUNT;
        }
        else
        {
            PrintCount--;
        }

        // 轮询检测是否在进行复位、急停或者使能
        // 复位或急停
        if (SV660N.data.reset_busy == true || SV660N.data.quickstop_busy == true)
        {
            // 控制驱动器的状态转化到Switch On Disabled
            SV660N.Reset_Motor_State();

            //该处急停状态调用Reset_Motor_State()实际上不符合标准，是一种简略的写法
            //实际急停状态应该将控制字6040H写入0x0002进入快速停机状态dsQuickStopActive，快速停机状态之后根据快速停机方式605AH自动切换状态，605AH为0~3自动进入伺服无故障状态dsSwitchOnDisabled，605AH为5~7自动进入伺服运行状态dsOperationEnabled
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
                    SV660N.Motor_Move_To_Home();
                }
                else
                {
                    if (SV660N.data.position_move_enable == true)
                    {// 开始运动
                        // if(UpdateCount == 0)
                        // {
                        //     if(MOVE_DIRECTION)
                        //     {
                        //         // SV660N.domain_data.RxPDO.target_position += (int32_t)((ENCODER_RESOLUTION * MOVE_VELOCITY) / TASK_FREQUENCY);
                        //         SV660N.domain_data.RxPDO.target_position += (int32_t)(UPDATE_COUNT * SCREW_MOVE_VELOCITY / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY);
                        //     }
                        //     else
                        //     {
                        //         // SV660N.domain_data.RxPDO.target_position -= (int32_t)((ENCODER_RESOLUTION * MOVE_VELOCITY) / TASK_FREQUENCY);
                        //         SV660N.domain_data.RxPDO.target_position -= (int32_t)(UPDATE_COUNT * SCREW_MOVE_VELOCITY / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY);
                        //     }
                        //     UpdateCount = UPDATE_COUNT;
                        // }
                        // else
                        // {
                        //     UpdateCount--;
                        // }
                        if(MOVE_DIRECTION)
                            {
                                // SV660N.domain_data.RxPDO.target_position += (int32_t)((ENCODER_RESOLUTION * MOVE_VELOCITY) / TASK_FREQUENCY);
                                SV660N.domain_data.RxPDO.target_position += (int32_t)(SCREW_MOVE_VELOCITY / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY);
                            }
                            else
                            {
                                // SV660N.domain_data.RxPDO.target_position -= (int32_t)((ENCODER_RESOLUTION * MOVE_VELOCITY) / TASK_FREQUENCY);
                                SV660N.domain_data.RxPDO.target_position -= (int32_t)(SCREW_MOVE_VELOCITY / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY);
                            }
                    }
                    else
                    {
                        SV660N.domain_data.RxPDO.target_position = SV660N.domain_data.TxPDO.current_position;
                    }
                    EC_WRITE_S32(SV660N.domain_pd + SV660N.domain_offset.RxPDO.target_position, SV660N.domain_data.RxPDO.target_position);
                }
            }
        }
#ifdef ENABLE_DC
        clock_gettime(CLOCK_REALTIME, &actualTime);
        ecrt_master_sync_reference_clock_to(Master.master, TIMESPEC2NS(actualTime));
        ecrt_master_sync_slave_clocks(Master.master);
#endif

        Master.EtherCAT_Master_Send();

        clock_gettime(CLOCK_REALTIME, &endTime);
        actualPeriodTime = timespec_minus(endTime, wakeupTime);
        if(TIMESPEC2NS(actualPeriodTime) > TASK_PERIOD_NS && SV660N.data.drive_state == dsOperationEnabled)
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
    return 0;
}