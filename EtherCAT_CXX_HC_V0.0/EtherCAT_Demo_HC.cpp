#include <stdio.h>
#include <ecrt.h>
#include <unistd.h>
#include "EL2889.h"
#include "HC_SV660N.h"
#include "EtherCAT_Master.h"
#include "EtherCAT_Slave.h"


#define ENCODER_RESOLUTION 8388608                                    // 编码器分辨率 2^23
#define MOTOR_RATED_TORQUE 0.64                                       // 电机额定转矩0.64N/m
#define HOME_VECOLITY 1                                               // r/s，回零速度
#define HOME_STEP HOME_VECOLITY * ENCODER_RESOLUTION * POSITION_STEP  // pulse 回零步长 = 回零速度 * 编码器分辨率 * 任务周期
#define MOVE_DIRECTION 1                                              // 电机旋转方向定义
#define MOVE_VELOCITY 1                                               // r/s，电机运动速度定义

static void check_domain_state(ec_domain_t *domain, ec_domain_state_t *domain_state)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain, &ds);
    if (ds.working_counter != domain_state->working_counter)
    {
        printf("Domain: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain_state->wc_state)
    {
        printf("Domain: State %u.\n", ds.wc_state);
    }
    *domain_state = ds;
}

static void check_master_state(ec_master_t *master, ec_master_state_t *master_state)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state->slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state->al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state->link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    *master_state = ms;
}

static void check_slave_config_states(ec_slave_config_t *slave, ec_slave_config_state_t *slave_state)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(slave, &s);
    if (s.al_state != slave_state->al_state)
    {
        printf("SV660N: State 0x%02X.\n", s.al_state);
    }
    if (s.online != slave_state->online)
    {
        printf("SV660N: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != slave_state->operational)
    {
        printf("SV660N: %soperational.\n", s.operational ? "" : "Not ");
    }
    *slave_state = s;
}


int main(int argc, char **argv)
{
    EtherCAT_Master Master;
    HC_SV660N_Motor Motor = {ENCODER_RESOLUTION, MOTOR_RATED_TORQUE};
    HC_SV660N SV660N(Master, 0, 0, Motor);
    Master.EtherCAT_Master_Config();
    printf("*It's working now*\n");

    int32_t MoveOriginPoint;
    bool GetOriginPoint = true;
    // int16_t position_count = 0;
    // double t = 0.00;

    uint16_t PrintCount = 1000;

    while (true)
    {
        usleep(1*1000*1000*POSITION_STEP); // 计算一次周期的时间

        // 接收过程数据
        ecrt_master_receive(Master.master); // EtherCAT 主站接收一次报文
        ecrt_domain_process(SV660N.domain); // 可以理解成根据报文更新一次domain域中的信息

        // 检查过程数据状态（可选）
        check_domain_state(SV660N.domain, &SV660N.domain_state);
        // 检查主站状态
        check_master_state(Master.master, &Master.master_state);
        // 检查从站配置状态
        check_slave_config_states(SV660N.slave_config, &SV660N.slave_config_state);

        // 读取数据，根据注册获得的PDO对象字典的偏移量进行读取
        SV660N.Read_Data();

        // DS402 CANOpen over EtherCAT status machine
        // 获取当前驱动器的驱动状态XZ
        if ((SV660N.domain_data.status_word & 0x004F) == 0x0000)
            SV660N.data.drive_state = dsNotReadyToSwitchOn;   //  初始化 未完成状态
        else if ((SV660N.domain_data.status_word & 0x004F) == 0x0040)
            SV660N.data.drive_state = dsSwitchOnDisabled;     //  初始化完成 伺服无故障状态
        else if ((SV660N.domain_data.status_word & 0x006F) == 0x0021)
            SV660N.data.drive_state = dsReadyToSwitchOn;      //  伺服准备好状态
        else if ((SV660N.domain_data.status_word & 0x006F) == 0x0023)
            SV660N.data.drive_state = dsSwitchedOn;           //  伺服启动，等待打开伺服使能
        else if ((SV660N.domain_data.status_word & 0x006F) == 0x0027)
            SV660N.data.drive_state = dsOperationEnabled;     //  伺服使能打开，伺服正常运行状态
            if(GetOriginPoint == true)
            {
                MoveOriginPoint = SV660N.domain_data.current_position;
                GetOriginPoint == false;
            }
        else if ((SV660N.domain_data.status_word & 0x006F) == 0x0007)
            SV660N.data.drive_state = dsQuickStopActive;      //  快速停机有效
        else if ((SV660N.domain_data.status_word & 0x004F) == 0x000F)
            SV660N.data.drive_state = dsFaultReactionActive;  //  故障停机，故障反应有效
        else if ((SV660N.domain_data.status_word & 0x004F) == 0x0008)
            SV660N.data.drive_state = dsFault;                //  故障状态

        if(PrintCount == 0)
        {
            switch (SV660N.data.drive_state)
            {
                case dsNotReadyToSwitchOn:
                    printf("*SV660N: dsNotReadyToSwitchOn*\n");
                    break;
                case dsSwitchOnDisabled:
                    printf("*SV660N: dsSwitchOnDisabled*\n");
                    break;
                case dsReadyToSwitchOn:
                    printf("*SV660N: dsReadyToSwitchOn*\n");
                    break;
                case dsSwitchedOn:
                    printf("*SV660N: dsSwitchedOn*\n");
                    break;
                case dsOperationEnabled:
                    printf("SV660N: act position = %d, act torque = %d N/m, status = 0x%x, drive_state = %d, opmode = 0x%x\n",
                            SV660N.domain_data.current_position, SV660N.domain_data.current_torque, SV660N.domain_data.status_word, SV660N.data.drive_state, SV660N.domain_data.operation_mode);
                    break;
                case dsQuickStopActive:
                    printf("*SV660N: dsQuickStopActive*\n");
                    break;
                case dsFaultReactionActive:
                    printf("*SV660N: dsFaultReactionActive*\n");
                    break;
                case dsFault:
                    printf("*SV660N: dsFault*\n");
                    break;
                default:
                    printf("EtherCAT: Read drive state failed!\n");
            }
           PrintCount = 1000;
        }
        else
        {
            PrintCount--;
        }

        // 轮询检测是否在进行复位、急停或者使能
        // 复位
        if (SV660N.data.reset_busy == true)
        {
            switch(SV660N.data.drive_state){
                case dsReadyToSwitchOn:
                case dsSwitchedOn:
                case dsOperationEnabled:
                    SV660N.domain_data.ctrl_word = 0x0000;
                    EC_WRITE_U16(SV660N.domain_pd + SV660N.domain_offset.ctrl_word, SV660N.domain_data.ctrl_word);
                    break;
                case dsFaultReactionActive:
                case dsFault:
                    SV660N.domain_data.ctrl_word = 0x0080;
                    EC_WRITE_U16(SV660N.domain_pd + SV660N.domain_offset.ctrl_word, SV660N.domain_data.ctrl_word);
                    break;
                case dsQuickStopActive:
                    break;
                case dsSwitchOnDisabled:
                    SV660N.data.reset_busy == false;
                    break;
            }
        }
        // 急停
        else if (SV660N.data.quickstop_busy == true)
        {
            // 控制驱动器的状态转化到Switch On Disabled
            switch (SV660N.data.drive_state)
            {
                case dsSwitchedOn:
                case dsReadyToSwitchOn:
                case dsOperationEnabled:
                    SV660N.domain_data.ctrl_word = 0x0000;
                    EC_WRITE_U16(SV660N.domain_pd + SV660N.domain_offset.ctrl_word, SV660N.domain_data.ctrl_word); // Disable Voltage
                    break;
                default:
                    SV660N.data.quickstop_busy = false;
                    SV660N.data.position_move_enable = false;
            }
        }
        // 使能
        else if (SV660N.data.power_busy == true)
        {
            switch (SV660N.data.drive_state)
            {
                case dsNotReadyToSwitchOn:
                    break;
                case dsSwitchOnDisabled:
                    // 设置运行模式为位置模式
                    SV660N.domain_data.operation_mode = 8;
                    EC_WRITE_U8(SV660N.domain_pd + SV660N.domain_offset.operation_mode, SV660N.domain_data.operation_mode);
                    EC_WRITE_U16(SV660N.domain_pd + SV660N.domain_offset.ctrl_word, 0x0006);
                    break;
                case dsReadyToSwitchOn:
                    SV660N.domain_data.operation_mode = 8;
                    EC_WRITE_U8(SV660N.domain_pd + SV660N.domain_offset.operation_mode, SV660N.domain_data.operation_mode);
                    EC_WRITE_U16(SV660N.domain_pd + SV660N.domain_offset.ctrl_word, 0x0007);
                    break;
                case dsSwitchedOn:
                    SV660N.domain_data.operation_mode = 8;
                    EC_WRITE_U8(SV660N.domain_pd + SV660N.domain_offset.operation_mode, SV660N.domain_data.operation_mode);
                    EC_WRITE_U16(SV660N.domain_pd + SV660N.domain_offset.ctrl_word, 0x000f); // enable operation
                    SV660N.domain_data.target_position = SV660N.domain_data.current_position; // 将当前位置复制给目标位置，防止使能后电机震动
                    EC_WRITE_S32(SV660N.domain_pd + SV660N.domain_offset.target_position, SV660N.domain_data.target_position);
                    break;
                case dsOperationEnabled:
                    printf("*SV660N: Success to enable*\n");
                    SV660N.data.power_busy = false;
                    SV660N.data.position_move_enable = true;
                    break;
            }
        }
        if (SV660N.data.drive_state == dsOperationEnabled && SV660N.data.reset_busy == 0 && SV660N.data.power_busy == 0 && SV660N.data.quickstop_busy == 0)
        {
            if (SV660N.domain_data.operation_mode == 8)
            { // 位置模式
                if (SV660N.data.home_busy == true)
                { // 开始回零
                    if (SV660N.domain_data.current_position - MoveOriginPoint > 0)
                    {
                        SV660N.domain_data.target_position = SV660N.domain_data.current_position - HOME_STEP;
                        if (SV660N.domain_data.target_position - MoveOriginPoint< 0)
                        {
                            SV660N.domain_data.target_position = MoveOriginPoint;
                        }
                    }
                    else if (SV660N.domain_data.current_position - MoveOriginPoint< 0)
                        {
                            SV660N.domain_data.target_position = SV660N.domain_data.current_position + HOME_STEP;
                            if (SV660N.domain_data.target_position - MoveOriginPoint> 0)
                            {
                                SV660N.domain_data.target_position = MoveOriginPoint;
                            }
                        }
                    else if (SV660N.domain_data.current_position == MoveOriginPoint)
                        {
                            SV660N.domain_data.target_position = MoveOriginPoint;
                            SV660N.data.home_busy = false; // 回零结束
                            SV660N.data.position_move_enable = false;
                        }
                    EC_WRITE_S32(SV660N.domain_pd + SV660N.domain_offset.target_position, SV660N.domain_data.target_position);
                }
                else
                {
                    if (SV660N.data.position_move_enable == true)
                    {// 开始运动
                        // if (position_count == 8000)
                        // {
                        //     position_count = 0;
                        //     t = 0;
                        // }
                        // SV660N.domain_data.target_position = MoveOriginPoint + (int32_t)(ENCODER_RESOLUTION * sin(Pi * t) / 2); // 位置模式时传送位置信息
                        // // printf("电机实时位置%d\t,电机目标位置%d\n",SV660N.domain_data.current_position,SV660N.domain_data.target_position);
                        // t = t + (double)POSITION_STEP;
                        // position_count = position_count + 1;
                        if(MOVE_DIRECTION){
                            SV660N.domain_data.target_position += ENCODER_RESOLUTION * MOVE_VELOCITY * POSITION_STEP;
                        }
                        else{
                            SV660N.domain_data.target_position -= ENCODER_RESOLUTION * MOVE_VELOCITY * POSITION_STEP;
                        }
                    }
                    else
                    {
                        SV660N.domain_data.target_position = SV660N.domain_data.current_position;
                    }
                    EC_WRITE_S32(SV660N.domain_pd + SV660N.domain_offset.target_position, SV660N.domain_data.target_position);
                }
            }
        }
        ecrt_domain_queue(SV660N.domain); // 根据domain数据生成帧
        ecrt_master_send(Master.master);  // EtherCAT主站发送
    }
    Master.EtherCAT_Master_Deactivate();
}