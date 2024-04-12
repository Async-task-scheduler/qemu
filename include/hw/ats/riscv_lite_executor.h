#ifndef HW_RISCV_LITE_EXECUTOR_H
#define HW_RISCV_LITE_EXECUTOR_H

#include "hw/sysbus.h"
#include "qemu/queue.h"


#define TYPE_RISCV_LITE_EXECUTOR "riscv.lite_executor"

#define RISCV_LITE_EXECUTOR(obj) OBJECT_CHECK(RISCVLiteExecutor, (obj), TYPE_RISCV_LITE_EXECUTOR)

#define MAX_TASK_QUEUE              0x8
#define MAX_TASK_PER_QUEUE          0x100
#define MAX_ONLINE_STRUCT_GROUP     0x4
#define MAX_PROCESS                 0x10
#define MAX_EXTERNAL_INTR           0x10

#define RISCV_LITE_EXECUTOR_MMIO_SIZE           0x1000000

// MMIO Structure
#define PROCESS_MMIO_OFFSET                     0x0
#define PROCESS_MMIO_SIZE                       0x1000
#define PROCESS_MMIO_COUNT                      0x1000 // 4096
#define PROCESS_END_MMIO_OFFSET                 ((PROCESS_MMIO_OFFSET) + (PROCESS_MMIO_SIZE) * (PROCESS_MMIO_COUNT))

// Process MMIO Structure
#define PRIO_SCHEDULER_MMIO_OFFSET              0x0
#define PRIO_SCHEDULER_MMIO_SIZE                0x800
#define IPC_HANDLER_MMIO_OFFSET                 ((PRIO_SCHEDULER_MMIO_OFFSET) + (PRIO_SCHEDULER_MMIO_SIZE))
#define IPC_HANDLER_MMIO_SIZE                   0x100
#define EXTERNAL_INTERRUPT_HANDLER_MMIO_OFFSET  ((IPC_HANDLER_MMIO_OFFSET) + (IPC_HANDLER_MMIO_SIZE))
#define EXTERNAL_INTERRUPT_HANDLER_MMIO_SIZE    0x700

// Priority Scheduler MMIO Structure
#define PS_DEQUEUE_MMIO_OFFSET                  0x0
#define PS_DEQUEUE_MMIO_SIZE                    0x8
#define PS_ENQUEUE_MMIO_OFFSET                  ((PS_DEQUEUE_MMIO_OFFSET) + (PS_DEQUEUE_MMIO_SIZE))
#define PS_ENQUEUE_MMIO_SIZE                    0x8
#define PS_ENQUEUE_MMIO_COUNT                   0xff // 255
#define PS_END_MMIO_OFFSET                      ((PS_ENQUEUE_MMIO_OFFSET) + (PS_ENQUEUE_MMIO_SIZE) * (PS_ENQUEUE_MMIO_COUNT))

// IPC Handler MMIO Structure
#define IH_SEND_MMIO_OFFSET                     0x0
#define IH_SEND_MMIO_SIZE                       0x8
#define IH_BQ_MMIO_OFFSET                       ((IH_SEND_MMIO_OFFSET) + (IH_SEND_MMIO_SIZE))
#define IH_BQ_MMIO_SIZE                         0x8
#define IH_BQ_MMIO_COUNT                        0x1f // 31
#define IH_END_MMIO_OFFSET                      ((IH_BQ_MMIO_OFFSET) + (IH_BQ_MMIO_SIZE) * (IH_BQ_MMIO_COUNT))

// External Interrupt Handler MMIO Structure
#define EIH_ENQUEUE_MMIO_OFFSET                 0x0
#define EIH_ENQUEUE_MMIO_SIZE                   0x8
#define EIH_ENQUEUE_MMIO_COUNT                  0xe0 // 224
#define EIH_END_MMIO_OFFSET                     ((EIH_ENQUEUE_MMIO_OFFSET) + (EIH_ENQUEUE_MMIO_SIZE) * (EIH_ENQUEUE_MMIO_COUNT))

typedef struct {
    uint64_t ps_mbuf;
    uint64_t ipc_mbuf;
    uint64_t index;
}ProcStatus;

void proc_status_init(ProcStatus* ps);
void proc_status_set_online(ProcStatus* ps);
bool proc_status_is_online(ProcStatus* ps);
void proc_status_set_offline(ProcStatus* ps);
void proc_status_add_map(ProcStatus* ps, uint64_t index);

typedef QSIMPLEQ_HEAD(, QueueEntry) QueueHead;

struct QueueEntry {
    uint64_t data;
    QSIMPLEQ_ENTRY(QueueEntry) next;
};

typedef struct {
    QueueHead head;
} Queue;

void queue_init(Queue* queue);
void queue_push(Queue* queue, uint64_t data);
uint64_t queue_pop(Queue* queue);

typedef struct {
    Queue *task_queues;
}PriorityScheduler;

typedef struct {
    Queue *interrupt_queues;
}ExternalInterruptHandler;


void ps_init(PriorityScheduler* ps);
void ps_push(PriorityScheduler* ps, uint64_t priority, uint64_t data);
uint64_t ps_pop(PriorityScheduler* ps);

void eih_init(ExternalInterruptHandler* eih);
void eih_push(ExternalInterruptHandler* eih, uint64_t intr_num, uint64_t data);
uint64_t eih_pop(ExternalInterruptHandler* eih, uint64_t intr_num);

typedef struct RISCVLiteExecutor
{
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;
    ProcStatus *pst;
    PriorityScheduler *pschedulers;
    // external interrupt handler queues
    ExternalInterruptHandler *eihs;
    // rw buffer? convert 32bit read to 64bit read
    uint64_t read_buf;
    uint64_t expect_read_addr; // -1 indicates no expect address
    uint64_t write_buf;
    uint64_t expect_write_addr; // -1 indicates no expect address

    /* config */
    uint32_t num_sources; //中断处理相关，中断源的数目？

} RISCVLiteExecutor;

DeviceState *riscv_lite_executor_create(hwaddr addr, uint32_t num_sources);

#endif