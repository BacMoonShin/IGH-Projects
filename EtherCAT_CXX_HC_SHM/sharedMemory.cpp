#include "sharedMemory.h"

/*
    函数名称： SharedMemory
    函数功能： 构造函数，创建共享内存
    函数参数： const char *filepath, int len, int key_num
    返回值： 无

    filepath 是一个已经存在的文件名或者目录名
    len 是共享内存的大小，以字节为单位
    key_num 是一个整数，用于生成key值
    sem_flag表示共享内存的读取方式，0表示互斥锁，1表示读写锁。读写锁有问题，暂时不用
    sem_name是信号量的名字，用于区分不同的信号量
    只要filepath和key_num一样，生成的key值就一样（不同应用程序也可以）
    */
SharedMemory::SharedMemory(const char *filepath, int len, int key_num, int sem_flag, std::string sem_name)
{
    // 创建key
    this->sem_flag = sem_flag;
    key = ftok(filepath, key_num);
    memLen = len;
    if(key < 0)
    {
        throw std::runtime_error("Failed to create key for shared memory");
        printf("create key error!\n");
        return;
    }
    // 创建共享内存
    shmid = shmget(key, len, IPC_CREAT|0666);
    if(shmid < 0)
    {
        throw std::runtime_error("Failed to create shared memory segment");
        printf("create share mem error!\n");
        return;
    }
    std::cout << "共享内存创建成功" << std::endl;
    // 互斥锁
    if(sem_flag == 0)
    {
        sem = sem_open(std::string(sem_name + "_mutex").c_str(), O_CREAT, 0666, 1);
        if (sem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed");
        }
        std::cout << "互斥锁创建成功" << std::endl;
    }
    // 读写锁
    else if(sem_flag == 1)
    {
        // 创建读写锁信号量
        readSem = sem_open(std::string(sem_name + "_read").c_str(), O_CREAT, 0666, 10);
        if (readSem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed for read semaphore");
        }
        writeSem = sem_open(std::string(sem_name + "_write").c_str(), O_CREAT, 0666, 1);
        if (writeSem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed for write semaphore");
        }
        readNumSem = sem_open(std::string(sem_name + "_readnum").c_str(), O_CREAT, 0666, 1);
        if (readNumSem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed for write semaphore");
        }
        std::cout << "读写锁创建成功" << std::endl;
    }

}

/*
    函数名称： ~SharedMemory
    函数功能： 析构函数，释放共享内存
    函数参数： 无
    返回值： 无
    */
SharedMemory::~SharedMemory()
{
    // 断开共享内存
    disconnect();
    // 删除共享内存
    // remove(); // 等待所有进程都结束后再删除
    // 关闭信号量
    sem_close(sem);
    // 删除信号量
    // sem_unlink("/semaphore");
    // 释放信号量
    // sem_destroy(sem); // 等待所有进程都结束后再释放
}

/*
    函数名称： connect
    函数功能： 连接共享内存
    函数参数： 无
    返回值： void* 共享内存的起始地址

    该函数用于将共享内存连接到当前进程的地址空间中
    mem是当前进程共享内存连接的地址
    */
void* SharedMemory::connect()
{
    void *mem = shmat(shmid, NULL, 0);
    if(mem < 0)
    {
        throw std::runtime_error("Failed to attach shared memory segment");
        printf("shmat error!\n");
        return NULL;
    }
    this->beginAddress = mem;
    return mem;
}

/*
    函数名称： disconnect
    函数功能： 断开共享内存
    函数参数： 无
    返回值： 无

    该函数用于将共享内存从当前进程的地址空间中断开
    */
void SharedMemory::disconnect()
{
    if(beginAddress == NULL)
    {
        return;
    }
    if(shmdt(beginAddress) == -1)
    {
        throw std::runtime_error("Failed to detach shared memory segment");
    }
    beginAddress = NULL;
}

/*
    函数名称： remove
    函数功能： 删除共享内存
    函数参数： 无
    返回值： 无

    该函数用于删除共享内存
    */
void SharedMemory::remove()
{
    if(shmctl(shmid, IPC_RMID, NULL) == -1)
    {
        throw std::runtime_error("Failed to remove shared memory segment");
    }
}



