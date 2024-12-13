# pragma once
#include <cstdint>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <string>

// 控制指令结构体
struct AnkleMotorCmdData {
    int8_t motor_state;
    float tau;
    float limit_current;
    float spd;
    float pos;
    float kp;
    float kd;

};

struct UnitreeMotorCmdData {
    int8_t id;
    int8_t motorType;
    int8_t mode;
    float tau;
    float dq;
    float q;
    float kp;
    float kd;
};

struct LineMotorCmdData {
    int8_t id;
    int8_t mode;
    float tau;
    float x;
    float dx;
};



struct AllMotorCmdData
{
    int8_t enable;
    AnkleMotorCmdData ankle_motor_ll_cmd;
    AnkleMotorCmdData ankle_motor_lr_cmd;
    AnkleMotorCmdData ankle_motor_rl_cmd;
    AnkleMotorCmdData ankle_motor_rr_cmd;
    UnitreeMotorCmdData hip_motor_l_pitch_cmd;
    UnitreeMotorCmdData hip_motor_l_rotate_cmd;
    UnitreeMotorCmdData hip_motor_r_pitch_cmd;
    UnitreeMotorCmdData hip_motor_r_rotate_cmd;
    LineMotorCmdData line_motor_l_cmd;
    LineMotorCmdData line_motor_r_cmd;
};

// 接受状态结构体
struct UnitreeMotorStateData {
    int8_t id;
    float tau;
    float dq;
    float q;

};

struct LineMotorStateData {
    int8_t id;
    float tau;
    float dx;
    float x;
};


struct AnkleMotorStateData {
    float tau;
    float spd;
    float pos;
};

struct AllMotorStateData
{
    AnkleMotorStateData ankle_motor_ll_state;
    AnkleMotorStateData ankle_motor_lr_state;
    AnkleMotorStateData ankle_motor_rl_state;
    AnkleMotorStateData ankle_motor_rr_state;
    UnitreeMotorStateData hip_motor_l_pitch_state;
    UnitreeMotorStateData hip_motor_l_rotate_state;
    UnitreeMotorStateData hip_motor_r_pitch_state;
    UnitreeMotorStateData hip_motor_r_rotate_state;
    LineMotorStateData line_motor_l_state;
    LineMotorStateData line_motor_r_state;
};

struct ImuData {
    geometry_msgs::Quaternion orientation;
    float orientation_covariance[9];
    geometry_msgs::Vector3 angular_velocity;
    geometry_msgs::Vector3 linear_acceleration;
    geometry_msgs::Vector3 magnetic_field;
};


class SharedDataConfig{
    public:
        const char *FILE_PATH = "/home/a/humanoid_hust_rt_1001";
        const int key_num[5] = {1, 2, 3, 4, 5};
        const std::string sem_name[5] = {
            "/motorCmdSm",
            "/motorCmdSm_rl",
            "/motorStateSm",
            "/motorStateSm_rl",
            "/imuDataSm"
        };
        const int len[5] = {
            4 * 400,
            4 * 400,
            4 * 400,
            4 * 400,
            4 * 400
        };
        const int sem_flag = 0;
};