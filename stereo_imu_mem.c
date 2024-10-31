#include "stereo_imu_mem.h"

DECLARE_WAIT_QUEUE_HEAD(g_stereoImuMem_waitQueue);

#define ENTRY_COUNT (1024u * 16u)

static entry_t* mg_pEntry = NULL;
static unsigned int mg_entryRdPtr = 0, mg_entryWrPtr = 0;

static DEFINE_SPINLOCK(mg_spinlock);

int stereo_imu_mem_init(void) {
    mg_pEntry = (entry_t*) kmalloc_array(ENTRY_COUNT, sizeof(entry_t), GFP_KERNEL);
    if (NULL == mg_pEntry) {
        return -1;
    }

    return 0;
}

void stereo_imu_mem_deinit(void) {
    if (NULL != mg_pEntry) {
        kfree(mg_pEntry);
        mg_pEntry = NULL;
    }
}

void stereo_imu_mem_lock(void) { spin_lock(&mg_spinlock); }
void stereo_imu_mem_unlock(void) { spin_unlock(&mg_spinlock); }

unsigned int stereo_imu_mem_full(void) { return (mg_entryWrPtr + ENTRY_COUNT - mg_entryRdPtr) % ENTRY_COUNT; }
const entry_t* stereo_imu_mem_readAt(unsigned int offset) { return mg_pEntry + ((mg_entryRdPtr + offset) % ENTRY_COUNT); }
void stereo_imu_mem_read(unsigned int count) { mg_entryRdPtr = (mg_entryRdPtr + count) % ENTRY_COUNT; }

unsigned int stereo_imu_mem_empty(void) { return (mg_entryRdPtr + (ENTRY_COUNT - 1) - mg_entryWrPtr) % ENTRY_COUNT; }
entry_t* stereo_imu_mem_writeAt(unsigned int offset) { return mg_pEntry + ((mg_entryWrPtr + offset) % ENTRY_COUNT); }
void stereo_imu_mem_write(unsigned int count) {
    mg_entryWrPtr = (mg_entryWrPtr + count) % ENTRY_COUNT;
    wake_up(&g_stereoImuMem_waitQueue);
}
