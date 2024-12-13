#include "EL2889.h"
#include "HC_SV660N.h"
#include "Elmo.h"

#define _ELMO_1_ENABLE_
#define _ELMO_2_ENABLE_

static bool EtherCAT_Running_Flag = true;
static void EtherCAT_Exit(int signal){EtherCAT_Running_Flag = false;}

EtherCAT_Master Master;
ELMO_SDO_Data SDO_Data = {1, 1, 3, 2, 131072, 10000, 1, 1, 0, CSP};

#ifdef _ELMO_1_ENABLE_
ELMO Elmo1(Master, 0, 0, SDO_Data);
#endif
#ifdef _ELMO_2_ENABLE_
ELMO Elmo2(Master, 0, 1, SDO_Data);
#endif

void* EtherCAT_Thread(void *_arg)
{
    Master.EtherCAT_Master_Config();
    ETHERCAT_MESSAGE("*EtherCAT Master and Slaves config success*");
    signal(SIGINT, EtherCAT_Exit);

    uint16_t printCount = PRINT_COUNT;
    // uint16_t updateCount = UPDATE_COUNT;
    struct timespec cycleTime = {0, TASK_PERIOD_NS};
    struct timespec wakeupTime, endTime, actualPeriodTime, actualTime;
	  clock_gettime(CLOCK_REALTIME, &wakeupTime);

    uint16_t changedirection = 6000;
    bool MOVE_DIRECTION = true;

    while (EtherCAT_Running_Flag)
    {
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeupTime, NULL);
#ifdef ENABLE_DC
        ecrt_master_application_time(Master.master, TIMESPEC2NS(wakeupTime));
        ecrt_master_sync_reference_clock(Master.master);
#endif

        // 接收过程数据
        Master.EtherCAT_Master_Receive();

#ifdef ETHERCAT_DEBUG_MODE
        Master.EtherCAT_Check_States();
#ifdef _ELMO_1_ENABLE_
        Elmo1.Check_Motor_State(1);
#endif
#ifdef _ELMO_2_ENABLE_
        Elmo2.Check_Motor_State(2);
#endif
#endif
        if(printCount == 0)
        {
#ifdef _ELMO_1_ENABLE_
            // if(Elmo1.data.drive_state == dsOperationEnabled)
            {
                ETHERCAT_INFO("Elmo1: act position = %d, act torque = %d, status = 0x%x, drive_state = %d, drive_mode = %d",
                              Elmo1.domain_data.TxPDO.current_position, Elmo1.domain_data.TxPDO.torque_input, Elmo1.domain_data.TxPDO.status_word, Elmo1.data.drive_state, Elmo1.data.drive_mode);
                ETHERCAT_INFO("Actual Period Time: %ld", TIMESPEC2NS(actualPeriodTime));
                // ETHERCAT_INFO("Wakeup Time: %ld", TIMESPEC2NS(wakeupTime));
                // ETHERCAT_INFO("end Time: %ld", TIMESPEC2NS(endTime));
            }
#endif
#ifdef _ELMO_2_ENABLE_
            // if(Elmo2.data.drive_state == dsOperationEnabled)
            // {
            //     ETHERCAT_INFO("Elmo2: act position = %d, act torque = %d, status = 0x%x, drive_state = %d, drive_mode =%d",
            //                   Elmo2.domain_data.TxPDO.current_position, Elmo2.domain_data.TxPDO.torque_input, Elmo2.domain_data.TxPDO.status_word, Elmo2.data.drive_state，Elmo2.data.drive_mode);
            // }
#endif
            printCount = PRINT_COUNT;
        }
        else
        {
            printCount--;
        }

#ifdef _ELMO_1_ENABLE_
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
                            changedirection --;
                            if(changedirection == 0){
                                MOVE_DIRECTION = !(MOVE_DIRECTION);
                                changedirection = 3000;}
                        }
                        else
                        {
                            Elmo1.domain_data.RxPDO.target_position -= MOVE_VELOCITY;
                            changedirection --;
                            if(changedirection == 0){
                                MOVE_DIRECTION = !(MOVE_DIRECTION);
                                changedirection = 3000;}
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
#endif

#ifdef _ELMO_2_ENABLE_
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
                            Elmo2.domain_data.RxPDO.target_position += moveVolicity;
                            changedirection --;
                            if(changedirection == 0){
                                MOVE_DIRECTION = !(MOVE_DIRECTION);
                                changedirection = 3000;}
                        }
                        else
                        {
                            Elmo2.domain_data.RxPDO.target_position -= moveVolicity;
                            changedirection --;
                            if(changedirection == 0){
                                MOVE_DIRECTION = !(MOVE_DIRECTION);
                                changedirection = 3000;}
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
#endif


#ifdef ENABLE_DC
        clock_gettime(CLOCK_REALTIME, &actualTime);
        ecrt_master_sync_reference_clock_to(Master.master, TIMESPEC2NS(actualTime));
        ecrt_master_sync_slave_clocks(Master.master);
#endif

        Master.EtherCAT_Master_Send();

        clock_gettime(CLOCK_REALTIME, &endTime);
        actualPeriodTime = timespec_minus(endTime, wakeupTime);
        if(TIMESPEC2NS(actualPeriodTime) > TASK_PERIOD_NS)
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

int main(int argc, char **argv)
{
    int res = system("./SetPREOP.sh");
    if(res == -1 || res == 127){ ETHERCAT_CRITICAL("Reset Elmo to PREOP failed!");}
    pthread_t thread1;
    cpu_set_t cpuset;
    struct sched_param param1;

    // 创建线程1
    pthread_create(&thread1, nullptr, EtherCAT_Thread, NULL);

    // 设置线程1的优先级
    param1.sched_priority = 10;                        // 设置线程1优先级为10
    pthread_setschedparam(thread1, SCHED_RR, &param1); // pthread_setschedparam函数针对线程优先级进行设置，SCHED_RR表示将根据优先级使用轮转的实时调度策略，允许同优先级的线程进行轮转运行

    // 设置线程1的CPU亲和性（绑定到CPU 0）
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset); // 设置线程1只能在CPU 0上运行
    pthread_setaffinity_np(thread1, sizeof(cpu_set_t), &cpuset);

    pthread_join(thread1, nullptr);
}
