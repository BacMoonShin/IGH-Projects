#include "EtherCAT_Global.h"

const struct timespec timespec_add(const struct timespec& time1, const struct timespec& time2)
{
    struct timespec result;
  
    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
      result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
      result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
      result.tv_sec = time1.tv_sec + time2.tv_sec;
      result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }
  
	return result;
}

const struct timespec timespec_minus(const struct timespec& time1, const struct timespec& time2)
{
	struct timespec result;
    uint64_t time1_ns = TIMESPEC2NS(time1);
    uint64_t time2_ns = TIMESPEC2NS(time2);
    if(time1_ns < time2_ns){
        result.tv_sec, result.tv_nsec = 0;
    }
    else{
        result.tv_sec = (time1_ns - time2_ns) / NSEC_PER_SEC;
        result.tv_nsec = (time1_ns - time2_ns) % NSEC_PER_SEC;
    }
        return result;
}