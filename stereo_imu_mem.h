#ifndef _STEREO_IMU_MEM
#define _STEREO_IMU_MEM

#include <linux/slab.h>

typedef enum {                    //
    ENTRY_TYPE_CALIB_MAG_SET,     // uses data_mag union field
    ENTRY_TYPE_CALIB_MAG_RESET,   // uses data_mag union field
    ENTRY_TYPE_DATA_MAG,          //
    ENTRY_TYPE_DATA_GYRO,         //
    ENTRY_TYPE_DATA_ACC
} entry_type_e;

typedef struct {
    uint32_t x, y, z;
} entry_data_mag_t;

typedef struct {
    int32_t x, y, z;
} entry_data_gyro_t;

typedef struct {
    int32_t x, y, z;
} entry_data_acc_t;

typedef struct {
    entry_type_e type;
    unsigned long timestamp;
    union {
        entry_data_mag_t data_mag;
        entry_data_gyro_t data_gyro;
        entry_data_acc_t data_acc;
    };
} entry_t;

extern wait_queue_head_t g_stereoImuMem_waitQueue;

int stereo_imu_mem_init(void);
void stereo_imu_mem_deinit(void);

void stereo_imu_mem_lock(void);
void stereo_imu_mem_unlock(void);

unsigned int stereo_imu_mem_full(void);
const entry_t* stereo_imu_mem_readAt(unsigned int offset);
void stereo_imu_mem_read(unsigned int count);

unsigned int stereo_imu_mem_empty(void);
entry_t* stereo_imu_mem_writeAt(unsigned int offset);
void stereo_imu_mem_write(unsigned int count);

#endif   // _STEREO_IMU_MEM