/*
    Author : Xia tian
    Description :
        2024.9.21完成：
            1. 该类用于创建共享内存,实现进程间通信
            2. 该类提供了连接、断开、删除、读写等功能
            3. 该类提供了模板函数，可以读写任意类型的数据
        2024.9.23完成：
            4. 该类提供了信号量，用于保护共享内存的读写
            5. 该类提供了异常处理，当读写越界时，会抛出异常
*/
#pragma once
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <semaphore.h>
#include <fcntl.h>
#include <chrono>

class SharedMemory
{
    public:
        SharedMemory(){};
        SharedMemory(const char *filepath, int len, int key_num, int sem_flag, std::string sem_name);
        ~SharedMemory();

        void* connect();
        void disconnect();
        void remove();
        template <typename T> void write(T *data, int data_num, int offset=0){
            // 计时
            auto start = std::chrono::high_resolution_clock::now();
            // 检查是否越界
            if(offset + data_num > memLen){
                throw std::runtime_error("Out of memory range");
                return;
            }
            // BUG: offset + data_num * sizeof(T) > memLen ?
            if(sem_flag==0){
                sem_wait(sem); // 加锁
                memcpy(beginAddress + offset, data, data_num * sizeof(T));
                sem_post(sem); // 解锁
            }
            if(sem_flag==1){
                sem_wait(readSem);
                // 加读锁是为了reader_count++
                if (reader_count == 0) { //！ 有一个问题是reader_count不能多进程共享，除非多进程公用同一个实例
                    sem_wait(writeSem);
                }
                reader_count++;
                sem_post(readSem);
                memcpy(beginAddress + offset, data, data_num * sizeof(T));
                sem_wait(readSem);
                reader_count--;
                // 如果没有读者了，允许写者进行写操作
                if (reader_count == 0) {
                    sem_post(writeSem); // 允许写操作
                }
                sem_post(readSem);
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            // std::cout << "读数据花费时间: " << elapsed.count() * 1e6 << "us" << std::endl;
        };
        template <typename T> void read(T *data, int data_num, int offset=0){
            // 计时
            auto start = std::chrono::high_resolution_clock::now();
            // 检查是否越界
            if(offset + data_num > memLen){
                throw std::runtime_error("Out of memory range");
                return;
            }
            // !BUG
            if(sem_flag==0){
                sem_wait(sem); // 加锁
                memcpy(data, beginAddress + offset, data_num * sizeof(T));
                sem_post(sem); // 解锁
            }
            if(sem_flag==1){
                sem_wait(writeSem); // 没有在写，可以读
                memcpy(data, beginAddress + offset, data_num * sizeof(T));
                sem_post(writeSem);
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            // std::cout << "读数据花费时间: " << elapsed.count() * 1e6 << "us" << std::endl;
        }


    private:
        key_t key;
        int shmid;
        // 起始地址
        void *beginAddress;
        // 共享内存的大小
        int memLen;
        // 信号量， 用于保护共享内存的读写
        int sem_flag;
        sem_t *sem;
        sem_t *writeSem;
        sem_t *readSem;
        sem_t *readNumSem;
        int reader_count = 0;
};
