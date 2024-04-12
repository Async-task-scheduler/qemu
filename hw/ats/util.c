#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "target/riscv/cpu.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "hw/ats/riscv_lite_executor.h"
#include "qemu/queue.h"


void proc_status_init(ProcStatus* ps) {
    ps->ipc_mbuf = 0;
    ps->ps_mbuf = 0;
    ps->index = 0;
}

void proc_status_set_online(ProcStatus* ps) {
    ps->ps_mbuf |= 0x1;
}

bool proc_status_is_online(ProcStatus* ps) {
    return ps->ps_mbuf &= 0x1;
}

void proc_status_set_offline(ProcStatus* ps) {
    ps->ps_mbuf &= ~0x1;
}

void proc_status_add_map(ProcStatus* ps, uint64_t index) {
    ps->index = index;
}

/************************************************/
void queue_init(Queue* queue) {
    QSIMPLEQ_INIT(&queue->head);
}

void queue_push(Queue* queue, uint64_t data) {
    struct QueueEntry *entry = g_new0(struct QueueEntry, 1);
    entry->data = data;
    QSIMPLEQ_INSERT_TAIL(&queue->head, entry, next);
}

uint64_t queue_pop(Queue* queue) {
    uint64_t res = 0;
    QueueHead *head = &queue->head;
    if (head->sqh_first != NULL) {
        struct QueueEntry *entry = head->sqh_first;
        res = entry->data;
        g_free(entry);
        QSIMPLEQ_REMOVE_HEAD(head, next);
        return res;
    }
    return res;
}

/************************************************/

void ps_init(PriorityScheduler* ps) {
    ps->task_queues = g_new0(Queue, MAX_TASK_QUEUE);
    int i = 0;
    for(i = 0; i < MAX_TASK_QUEUE; i++) {
        QSIMPLEQ_INIT(&ps->task_queues[i].head);
    }
}

void ps_push(PriorityScheduler* ps, uint64_t priority, uint64_t data) {
    struct QueueEntry *task_entry = g_new0(struct QueueEntry, 1);
    task_entry->data = data;
    QSIMPLEQ_INSERT_TAIL(&ps->task_queues[priority].head, task_entry, next);
}

uint64_t ps_pop(PriorityScheduler* ps) {
    uint64_t res = 0;
    int i = 0;
    for(i = 0; i < MAX_TASK_QUEUE; i++) {
        QueueHead *head = &ps->task_queues[i].head;
        if (head->sqh_first != NULL) {
            struct QueueEntry *task_entry = head->sqh_first;
            res = task_entry->data;
            g_free(task_entry);
            QSIMPLEQ_REMOVE_HEAD(head, next);
            return res;
        }
    }
    return res;
}

/************************************************/

void eih_init(ExternalInterruptHandler* eih) {
    eih->interrupt_queues = g_new0(Queue, MAX_EXTERNAL_INTR);
    int i = 0;
    for(i = 0; i < MAX_EXTERNAL_INTR; i++) {
        QSIMPLEQ_INIT(&eih->interrupt_queues[i].head);
    }
}

void eih_push(ExternalInterruptHandler* eih, uint64_t intr_num, uint64_t data) {
    struct QueueEntry *eih_entry = g_new0(struct QueueEntry, 1);
    eih_entry->data = data;
    QSIMPLEQ_INSERT_TAIL(&eih->interrupt_queues[intr_num].head, eih_entry, next);
}

uint64_t eih_pop(ExternalInterruptHandler* eih, uint64_t intr_num) {
    QueueHead *head = &eih->interrupt_queues[intr_num].head;
    if (head->sqh_first != NULL) {
        struct QueueEntry *eih_entry = head->sqh_first;
        uint64_t res = eih_entry->data;
        g_free(eih_entry);
        QSIMPLEQ_REMOVE_HEAD(head, next);
        return res;
    }
    else {
        return 0;
    }
}


