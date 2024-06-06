/*
 * RISC-V MOIC (Multiple-Object Interaction Interrupt Controller) interface
 */

#ifndef HW_RISCV_MOIC_H
#define HW_RISCV_MOIC_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/queue.h"

#define MAX_IRQ                 0x100
#define RISCV_MOIC_MMIO_SIZE    0x1000000


#define TYPE_RISCV_MOIC "riscv.moic"

typedef struct RISCVMOICState RISCVMOICState;
DECLARE_INSTANCE_CHECKER(RISCVMOICState, RISCV_MOIC, TYPE_RISCV_MOIC)

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
    uint64_t os_id;
    uint64_t proc_id;
    uint64_t task_id;
} TotalIdentity;

typedef struct {
    uint64_t task_id;
    TotalIdentity target;
} Capability;

typedef struct {
    TotalIdentity current;
    Queue ready_queue;
    uint64_t* irq_table;
    Capability* send_cap;
    Capability* recv_cap;
} MoicHart;

struct RISCVMOICState {
    /*< private >*/
    SysBusDevice parent_obj;
    qemu_irq *external_irqs;

    /*< public >*/
    MemoryRegion mmio;

    /* properties */
    uint32_t hart_count;
    uint32_t external_irq_count;


    /* config */
    MoicHart* moicharts;
};

DeviceState *riscv_moic_create(hwaddr addr, uint32_t hart_count, uint32_t external_irq_count);

#endif
