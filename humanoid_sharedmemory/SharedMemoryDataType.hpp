# pragma once
#ifndef __SHARED_MEMORY_DATA_TYPE_HPP
#define __SHARED_MEMORY_DATA_TYPE_HPP

#include <cstdint>
#include <string>

struct RotateMotorMixControl{
    float kp;
    float kd;
    float position; // rad
    float speed;    // rad/s
    float torque;   // Nm
};

struct RotateMotorPosControl {
    float position; // rad
    float speed;    // rad/s
    uint16_t maxPhaseCurrent; //A
};

struct RotateMotorTauControl {
    int16_t currentTorque;
    uint8_t ctrlStatus;
};

struct RotateMotorCmdData{
    uint8_t slave;              // 电机对应从站
    uint8_t passage;            // 电机在从站对应位置
    uint8_t motorId;            // 电机ID号
    uint8_t motorRunningMode = 1;   // 电机运行模式
    
    RotateMotorMixControl mixControl = {20.0, 5.0, 0, 0.0, 0.0};   // 混合控制模式
    RotateMotorPosControl posControl = {0, 2, 50};               // 位置控制模式
    RotateMotorTauControl tauControl = {0, 0};                // 力矩控制模式
};

struct LineMotorCmdData{
    uint8_t motorId;
    uint8_t motorRunningMode;
    float linearPos;
    float linearSpeed;
    float linearTau;
};

struct AllRotateMotorsCmdData{
    RotateMotorCmdData hipAnkleMotorsCmd[8];
};

struct AllLineMotorsCmdData{
    LineMotorCmdData kneeMotorsCmd[2];
};

#pragma pack(1)
struct RotateMotorStateData{
    uint8_t motorId;
    float torque;
    float position;
    float speed;
};
#pragma pack()

struct LineMotorStateData{
    uint8_t motorId;
    float linearTau;
    float linearPos;
    float linearSpeed;
};

struct AllRotateMotorsStateData{
    RotateMotorStateData hipAnkleMotorState[8];
};
struct AllLineMotorsStateData{
    LineMotorStateData kneeMotorsState[2];
};

struct ImuData3DType{
    float x;
    float y;
    float z;
};

struct ImuData4DType{
    float x;
    float y;
    float z;
    float w;
};

#pragma pack(4)
struct ImuData{
    ImuData3DType imuAcceleration;
    ImuData3DType imuAngularVelocity;
    ImuData3DType imuAngularDegree;
    ImuData4DType imuQuaternion;
};
#pragma pack();

class SharedDataConfig{
    public:
        const char *FILE_PATH = "/home/a/2th_humanoid_hust/";
        const int keyNum[8] = {1, 2, 3, 4, 5, 6, 7, 8};
        const std::string semName[8] = {
            "/rotateMotorCmdSharedMemory",
            "/lineMotorCmdSharedMemory",
            "/jointCmdSharedMemory_rl",
            "/rotateMotorStateSharedMemory",
            "/lineMotorStateSharedMemory",
            "/jointStateSharedMemory_rl",
            "/imuDataSharedMemory",
            "/rotateMotorPDSharedMemory"
        };
        const uint16_t length[8] = {
            4 * 400,
            4 * 400,
            4 * 400,
            4 * 400,
            4 * 400,
            4 * 400,
            4 * 400,
            4 * 400
        };  
        const int semFlag = 0;
};

#endif