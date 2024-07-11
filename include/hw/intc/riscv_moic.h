/*
 * RISC-V MOIC (Multiple-Object Interaction Interrupt Controller) interface
 */

#ifndef HW_RISCV_MOIC_H
#define HW_RISCV_MOIC_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/queue.h"
#include "exec/cpu-common.h"

#define MAX_IRQ                 0x10
#define RISCV_MOIC_MMIO_SIZE    0x1000000
#define MAX_PRIORITY            32
#define SIZEOF_PERHART          0x1000

#define ADD_OP                          0x00
#define FETCH_OP                        0x08
#define SWITCH_PROC_OP                  0x10
#define SWITCH_OS_OP                    0x18
#define REGISTER_RECV_TASK_OP           0x20
#define REGISTER_RECV_TARGET_OS_OP      0x28
#define REGISTER_RECV_TARGET_PROC_OP    0x30
#define REGISTER_RECV_TARGET_TASK_OP    0x38
#define REGISTER_SEND_TASK_OP           0x40
#define REGISTER_SEND_TARGET_OS_OP      0x48
#define REGISTER_SEND_TARGET_PROC_OP    0x50
#define REGISTER_SEND_TARGET_TASK_OP    0x58
#define SEND_INTR_OS_OP                 0x60                
#define SEND_INTR_PROC_OP               0x68                
#define SEND_INTR_TASK_OP               0x70                
#define SWITCH_HYPERVISOR_OP            0x78
#define CURRENT_OP                      0x80
#define REMOVE_OP                       0x88
#define STATUS_OP                       0x90
#define DUMP_OP                         0x98

typedef enum {
    NORMAL = 0,
    PREEMPT = 1,
    KILL = 2,
} Cause;

#define TCB_ALIGN                       0x40
#define READY_QUEUE_OFFSET              0x00
#define READY_QUEUE_STRUCT_SIZE         0x28
#define DEVICE_CAP_OFFSET               READY_QUEUE_OFFSET + READY_QUEUE_STRUCT_SIZE
#define DEVICE_CAP_SIZE                 MAX_IRQ * 0x20
#define SEND_CAP_OFFSET                 DEVICE_CAP_OFFSET + DEVICE_CAP_SIZE
#define SEND_CAP_STRUCT_SIZE            0x28
#define RECV_CAP_OFFSET                 SEND_CAP_OFFSET + SEND_CAP_STRUCT_SIZE
#define RECV_CAP_STRUCT_SIZE            SEND_CAP_STRUCT_SIZE
#define STATUS_OFFSET                   RECV_CAP_OFFSET + RECV_CAP_STRUCT_SIZE

#define TYPE_RISCV_MOIC "riscv.moic"

typedef struct RISCVMOICState RISCVMOICState;
DECLARE_INSTANCE_CHECKER(RISCVMOICState, RISCV_MOIC, TYPE_RISCV_MOIC)

/****************** structures **************************/ 
typedef QSIMPLEQ_HEAD(, QueueEntry) QueueHead;

struct QueueEntry {
    uint64_t data;
    QSIMPLEQ_ENTRY(QueueEntry) next;
};

typedef struct {
    QueueHead head;
} Queue;

typedef struct {
    Queue *task_queues;
} PriorityQueue;

typedef struct {
    uint64_t os_id;
    uint64_t proc_id;
    uint64_t task_id;
} TotalIdentity;

typedef struct {
    uint64_t task_id;
    TotalIdentity target;
} Capability;

typedef QSIMPLEQ_HEAD(, CapQueueEntry) CapQueueHead;

struct CapQueueEntry {
    Capability cap;
    QSIMPLEQ_ENTRY(CapQueueEntry) next;
};

typedef struct {
    CapQueueHead head;
} CapQueue;

typedef struct {
    uint64_t hypervisor_id;
    TotalIdentity current;
    TotalIdentity send_intr_transaction;
    Capability register_receiver_transaction;
    Capability register_sender_transaction;
    Cause cause;
    int64_t datasheet_id;
} HartTransaction;

typedef struct {
    uint64_t ref_count;
    PriorityQueue ready_queue;
    Capability* device_cap;
    CapQueue send_cap;
    CapQueue recv_cap;
} DataSheet;

struct RISCVMOICState {
    /*< private >*/
    SysBusDevice parent_obj;
    qemu_irq *external_irqs;
    /*< public >*/
    MemoryRegion mmio;
    /* properties */
    uint32_t hart_count;
    uint32_t external_irq_count;
    qemu_irq* usoft_irqs;
    qemu_irq* ssoft_irqs;
    /* config */
    HartTransaction* transactions;
    DataSheet* datasheets;
};

/****************** Utilities Functions **************************/ 

static inline void queue_init(Queue* queue) {
    QSIMPLEQ_INIT(&queue->head);
}

static inline void queue_push(Queue* queue, uint64_t data) {
    struct QueueEntry *entry = g_new0(struct QueueEntry, 1);
    entry->data = data;
    QSIMPLEQ_INSERT_TAIL(&queue->head, entry, next);
}

static inline uint64_t queue_pop(Queue* queue) {
    uint64_t res = 0;
    QueueHead *head = &queue->head;
    if (head->sqh_first != NULL) {
        struct QueueEntry *entry = head->sqh_first;
        res = entry->data;
        QSIMPLEQ_REMOVE_HEAD(head, next);
        g_free(entry);
        return res;
    }
    return res;
}

static inline void pq_init(PriorityQueue* pq) {
    pq->task_queues = g_new0(Queue, MAX_PRIORITY);
    int i = 0;
    for(i = 0; i < MAX_PRIORITY; i++) {
        QSIMPLEQ_INIT(&pq->task_queues[i].head);
    }
}

static inline void pq_push(PriorityQueue* pq, uint64_t priority, uint64_t data) {
    struct QueueEntry *task_entry = g_new0(struct QueueEntry, 1);
    task_entry->data = data;
    QSIMPLEQ_INSERT_TAIL(&pq->task_queues[priority].head, task_entry, next);
}

static inline uint64_t pq_pop(PriorityQueue* pq) {
    uint64_t res = 0;
    int i = 0;
    for(i = 0; i < MAX_PRIORITY; i++) {
        QueueHead *head = &pq->task_queues[i].head;
        if (head->sqh_first != NULL) {
            struct QueueEntry *task_entry = head->sqh_first;
            res = task_entry->data;
            QSIMPLEQ_REMOVE_HEAD(head, next);
            g_free(task_entry);
            break;
        }
    }
    return res;
}

static inline bool pq_is_empty(PriorityQueue* pq) {
    int i = 0;
    bool res = true;
    for(i = 0; i < MAX_PRIORITY; i++) {
        QueueHead *head = &pq->task_queues[i].head;
        if (!QSIMPLEQ_EMPTY(head)) {
            res = false;
            break;
        }
    }
    return res;
}

static inline uint64_t pq_len(PriorityQueue* pq) {
    uint64_t len = 0;
    int i = 0;
    for(i = 0; i < MAX_PRIORITY; i++) {
        QueueHead *head = &pq->task_queues[i].head;
        if (QSIMPLEQ_EMPTY(head)) {
            continue;
        }
        struct QueueEntry* cur, *next_elem;
        QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
            len += 1;
        }
    }
    return len;
}

static inline uint64_t* pq_iter(PriorityQueue* pq) {
    uint64_t len = pq_len(pq);
    uint64_t* task_buf = g_new0(uint64_t, len);
    int i = 0, j = 0;
    for(i = 0; i < MAX_PRIORITY; i++) {
        QueueHead *head = &pq->task_queues[i].head;
        if (QSIMPLEQ_EMPTY(head)) {
            continue;
        }
        struct QueueEntry* cur, *next_elem;
        QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
            task_buf[j] = cur->data;
            // info_report("pq iter, task%d: 0x%lx", j, cur->data);
            j += 1;
            QSIMPLEQ_REMOVE_HEAD(head, next);
            g_free(cur);
        }
    }
    return task_buf;
}

static inline void pq_remove(PriorityQueue* pq, uint64_t task_id) {
    uint64_t priority = (task_id >> 1) % MAX_PRIORITY;
    QueueHead *head = &pq->task_queues[priority].head;
    struct QueueEntry* cur, *next_elem;
    QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
        if (cur->data == task_id) {
            QSIMPLEQ_REMOVE(head, cur, QueueEntry, next);
            g_free(cur);
        }
    }
    return;
}

static inline void store_ready_queue(uint64_t src_task_id, PriorityQueue* pq) {
    uint64_t len = pq_len(pq);
    uint64_t* src_task_buf = pq_iter(pq);
    pq_init(pq);
    if (src_task_id == 0) {
        g_free(src_task_buf);
        return;
    }
    uint64_t src_tcb = src_task_id & (~(TCB_ALIGN - 1));
    uint64_t src_rq_addr = src_tcb + READY_QUEUE_OFFSET;
    bool* src_rq_online = g_new0(bool, 1);
    *src_rq_online = false;
    cpu_physical_memory_write(src_rq_addr + 8 * 3, (void*)src_rq_online, 1);
    g_free(src_rq_online);  
    uint64_t* src_rq_len = g_new0(uint64_t, 1);
    *src_rq_len = len;
    // info_report("src_rq_len: 0x%lx", *src_rq_len);
    cpu_physical_memory_write(src_rq_addr + 8 * 2, (void*)src_rq_len, 8);
    g_free(src_rq_len);
    if (src_task_buf != NULL) {
        // checkout tasks of the src_task
        uint64_t* src_rq_cap = g_new0(uint64_t, 1);
        cpu_physical_memory_read(src_rq_addr, (void*)src_rq_cap, 8);
        // info_report("len: %ld, cap: %ld", len, *src_rq_cap);
        assert(len <= *src_rq_cap);
        g_free(src_rq_cap);
        // read the ready queue pointer
        uint64_t* src_rq_ptr = g_new0(uint64_t, 1);
        cpu_physical_memory_read(src_rq_addr + 8 * 1, (void*)src_rq_ptr, 8);
        // info_report("src_rq_ptr: 0x%lx", *src_rq_ptr);
        cpu_physical_memory_write(*src_rq_ptr, (void*)src_task_buf, len * 8);
        g_free(src_rq_ptr);
        g_free(src_task_buf);
    }
}

static inline void load_ready_queue(uint64_t dst_task_id, PriorityQueue* pq) {
    // load the ready tasks of dst_task from the memory.
    uint64_t dst_tcb = dst_task_id & (~(TCB_ALIGN - 1));
    uint64_t dst_rq_addr = dst_tcb + READY_QUEUE_OFFSET;
    uint64_t* dst_rq_ptr = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_rq_addr + 8 * 1, (void*)dst_rq_ptr, 8);
    // info_report("dst_rq_ptr: 0x%lx", *dst_rq_ptr);
    uint64_t* dst_rq_len = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_rq_addr + 8 * 2, (void*)dst_rq_len, 8);
    // info_report("dst_rq_len: 0x%lx", *dst_rq_len);
    uint64_t* dst_rq_cap = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_rq_addr, (void*)dst_rq_cap, 8);
    // info_report("dst_rq_cap: 0x%lx", *dst_rq_cap);
    bool* dst_rq_online = g_new0(bool, 1);
    // info_report("dst_rq_online: %d", *dst_rq_online);
    cpu_physical_memory_read(dst_rq_addr + 8 * 3, (void*)dst_rq_online, 1);
    *dst_rq_online = true;
    cpu_physical_memory_write(dst_rq_addr + 8 * 3, (void*)dst_rq_online, 1);
    uint64_t* task_buf = g_new0(uint64_t, *dst_rq_len);
    if (task_buf != NULL) {
        cpu_physical_memory_read(*dst_rq_ptr, (void*)task_buf, (*dst_rq_len) * 8);
        uint64_t* empty_task_buf = g_new0(uint64_t, *dst_rq_len);
        cpu_physical_memory_write(*dst_rq_ptr, (void*)empty_task_buf, (*dst_rq_len) * 8);
        g_free(empty_task_buf);
        uint64_t empty_len = 0;
        cpu_physical_memory_write(dst_rq_addr + 8 * 2, (void*)&empty_len, 8);
        int i = 0;
        for (i = 0; i < *dst_rq_len; i++) {
            uint64_t task_id = task_buf[i];
            // info_report("task_id: 0x%lx", task_id);
            uint64_t priority = (task_id >> 1) % MAX_PRIORITY;
            pq_push(pq, priority, task_id);
        }
    }
    g_free(dst_rq_ptr);
    g_free(dst_rq_len);
    g_free(dst_rq_cap);
    g_free(dst_rq_online);
    g_free(task_buf);
}

static inline void identity_init(TotalIdentity* identity) {
    identity->os_id = 0;
    identity->proc_id = 0;
    identity->task_id = 0;
}

static inline void capability_init(Capability* capability) {
    capability->task_id = 0;
    identity_init(&capability->target);
}

static inline void store_device_cap(uint64_t src_task_id, Capability* device_cap) {
    if (src_task_id != 0) {
        uint64_t src_tcb = src_task_id & (~(TCB_ALIGN - 1));
        uint64_t src_device_cap_ptr = src_tcb + DEVICE_CAP_OFFSET;
        cpu_physical_memory_write(src_device_cap_ptr, (void*)device_cap, MAX_IRQ * sizeof(Capability));
    }
}

static inline void load_device_cap(uint64_t dst_task_id, Capability* device_cap) {
    uint64_t dst_tcb = dst_task_id & (~(TCB_ALIGN - 1));
    uint64_t dst_device_cap_ptr = dst_tcb + DEVICE_CAP_OFFSET;
    cpu_physical_memory_read(dst_device_cap_ptr, (void*)device_cap, MAX_IRQ * sizeof(Capability));
}

static inline void cap_queue_init(CapQueue* cap_queue) {
    QSIMPLEQ_INIT(&cap_queue->head);
}

static inline void cap_queue_insert(CapQueue* cap_queue, uint64_t task_id, uint64_t target_os_id, uint64_t target_proc_id, uint64_t target_task_id) {
    CapQueueHead* head = &cap_queue->head;
    struct CapQueueEntry* cur, *next_elem;
    QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
        if ((cur->cap.task_id == task_id) && (cur->cap.target.os_id == target_os_id) && (cur->cap.target.proc_id == target_proc_id)
            && (cur->cap.target.task_id == target_task_id)) {
            return;
        }
    }
    struct CapQueueEntry *cap = g_new0(struct CapQueueEntry, 1);
    cap->cap.task_id = task_id;
    cap->cap.target.os_id = target_os_id;
    cap->cap.target.proc_id = target_proc_id;
    cap->cap.target.task_id = target_task_id;
    QSIMPLEQ_INSERT_TAIL(head, cap, next);
}

static inline struct CapQueueEntry* cap_queue_remove(CapQueue* cap_queue, uint64_t task_id, uint64_t target_os_id, uint64_t target_proc_id, uint64_t target_task_id) {
    CapQueueHead* head = &cap_queue->head;
    struct CapQueueEntry* cur, *next_elem;
    QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
        if ((cur->cap.task_id == task_id) && (cur->cap.target.os_id == target_os_id) && (cur->cap.target.proc_id == target_proc_id)
            && (cur->cap.target.task_id == target_task_id)) {
            QSIMPLEQ_REMOVE(head, cur, CapQueueEntry, next);
            return cur;
        }
    }
    return NULL;
}

static inline bool is_device_cap(Capability* cap) {
    return (cap->target.os_id == 0) && (cap->target.proc_id == 0);
}
static inline void device_cap_logout(Capability* cap, uint64_t task_id) {
    int i = 0;
    for (i = 0; i < MAX_IRQ; i ++) {
        if (cap[i].task_id == task_id) {
            cap[i].task_id = 0;
        }
    }
}

static inline uint64_t cap_queue_len(CapQueue* cap_queue) {
    uint64_t len = 0;
    CapQueueHead* head = &cap_queue->head;
    struct CapQueueEntry* cur, *next_elem;
    QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
        len += 1;
    }
    return len;
}

static inline Capability* cap_queue_iter(CapQueue* cap_queue) {
    uint64_t len = cap_queue_len(cap_queue);
    Capability* cap_buf = g_new0(Capability, len);
    CapQueueHead* head = &cap_queue->head;
    struct CapQueueEntry* cur, *next_elem;
    int i = 0;
    QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
        cap_buf[i].task_id = cur->cap.task_id;
        cap_buf[i].target.os_id = cur->cap.target.os_id;
        cap_buf[i].target.proc_id = cur->cap.target.proc_id;
        cap_buf[i].target.task_id = cur->cap.target.task_id;
        i += 1;
        QSIMPLEQ_REMOVE_HEAD(head, next);
        g_free(cur);
    }
    return cap_buf;
}

static inline uint64_t cap_queue_find(CapQueue* cap_queue, uint64_t target_os_id, uint64_t target_proc_id, uint64_t target_task_id) {
    CapQueueHead* head = &cap_queue->head;
    struct CapQueueEntry* cur, *next_elem;
    QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
        if ((cur->cap.target.os_id == target_os_id) && (cur->cap.target.proc_id == target_proc_id)
            && (cur->cap.target.task_id == target_task_id)) {
            return cur->cap.task_id;
        }
    }
    return 0;
}

static inline void cap_queue_logout(CapQueue* cap_queue, uint64_t task_id) {
    CapQueueHead* head = &cap_queue->head;
    struct CapQueueEntry* cur, *next_elem;
    QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
        if (cur->cap.task_id == task_id) {
            QSIMPLEQ_REMOVE(head, cur, CapQueueEntry, next);
            g_free(cur);
        }
    }
}

static inline void store_send_cap_queue(uint64_t src_task_id, CapQueue* cap_queue) {
    uint64_t len = cap_queue_len(cap_queue);
    Capability* src_cap_buf = cap_queue_iter(cap_queue);
    if (src_task_id == 0) {
        g_free(src_cap_buf);
        return;
    }
    uint64_t src_tcb = src_task_id & (~(TCB_ALIGN - 1));
    uint64_t src_sendcap_addr = src_tcb + SEND_CAP_OFFSET;
    bool* src_sendcap_online = g_new0(bool, 1);
    *src_sendcap_online = false;
    cpu_physical_memory_write(src_sendcap_addr + 8 * 3, (void*)src_sendcap_online, 1);
    g_free(src_sendcap_online); 
    uint64_t* src_sendcap_len = g_new0(uint64_t, 1);
    *src_sendcap_len = len;
    // info_report("src_sendcap_len: 0x%lx", *src_sendcap_len);
    cpu_physical_memory_write(src_sendcap_addr + 8 * 2, (void*)src_sendcap_len, 8);
    g_free(src_sendcap_len);       
    if (src_cap_buf != NULL) {
        // checkout tasks of the src_task
        uint64_t* src_sendcap_cap = g_new0(uint64_t, 1);
        cpu_physical_memory_read(src_sendcap_addr, (void*)src_sendcap_cap, 8);
        assert(len <= *src_sendcap_cap);
        g_free(src_sendcap_cap);
        // read the ready queue pointer
        uint64_t* src_sendcap_ptr = g_new0(uint64_t, 1);
        cpu_physical_memory_read(src_sendcap_addr + 8 * 1, (void*)src_sendcap_ptr, 8);
        // info_report("src_sendcap_ptr: 0x%lx", *src_sendcap_ptr);
        cpu_physical_memory_write(*src_sendcap_ptr, (void*)src_cap_buf, len * sizeof(Capability));
        g_free(src_sendcap_ptr);
        g_free(src_cap_buf);
    }
}

static inline void store_recv_cap_queue(uint64_t src_task_id, CapQueue* cap_queue) {
    uint64_t len = cap_queue_len(cap_queue);
    Capability* src_cap_buf = cap_queue_iter(cap_queue);
    if (src_task_id == 0) {
        g_free(src_cap_buf);
        return;
    }
    uint64_t src_tcb = src_task_id & (~(TCB_ALIGN - 1));
    uint64_t src_recvcap_addr = src_tcb + RECV_CAP_OFFSET;
    bool* src_recvcap_online = g_new0(bool, 1);
    *src_recvcap_online = false;
    cpu_physical_memory_write(src_recvcap_addr + 8 * 3, (void*)src_recvcap_online, 1);
    g_free(src_recvcap_online);   
    uint64_t* src_recvcap_len = g_new0(uint64_t, 1);
    *src_recvcap_len = len;
    // info_report("src_recvcap_len: 0x%lx", *src_recvcap_len);
    cpu_physical_memory_write(src_recvcap_addr + 8 * 2, (void*)src_recvcap_len, 8);
    g_free(src_recvcap_len);     
    if (src_cap_buf != NULL) {
        // checkout tasks of the src_task
        uint64_t* src_recvcap_cap = g_new0(uint64_t, 1);
        cpu_physical_memory_read(src_recvcap_addr, (void*)src_recvcap_cap, 8);
        assert(len <= *src_recvcap_cap);
        g_free(src_recvcap_cap);
        // read the ready queue pointer
        uint64_t* src_recvcap_ptr = g_new0(uint64_t, 1);
        cpu_physical_memory_read(src_recvcap_addr + 8 * 1, (void*)src_recvcap_ptr, 8);
        // info_report("src_recvcap_ptr: 0x%lx", *src_recvcap_ptr);
        cpu_physical_memory_write(*src_recvcap_ptr, (void*)src_cap_buf, len * sizeof(Capability));
        g_free(src_recvcap_ptr);
        g_free(src_cap_buf);
    }
}

static inline void load_send_cap_queue(uint64_t dst_task_id, CapQueue* cap_queue) {
    // load the ready tasks of dst_task from the memory.
    uint64_t dst_tcb = dst_task_id & (~(TCB_ALIGN - 1));
    uint64_t dst_sendcap_addr = dst_tcb + SEND_CAP_OFFSET;
    uint64_t* dst_sendcap_ptr = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_sendcap_addr + 8 * 1, (void*)dst_sendcap_ptr, 8);
    // info_report("dst_sendcap_ptr: 0x%lx", *dst_sendcap_ptr);
    uint64_t* dst_sendcap_len = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_sendcap_addr + 8 * 2, (void*)dst_sendcap_len, 8);
    // info_report("dst_sendcap_len: 0x%lx", *dst_sendcap_len);
    uint64_t* dst_sendcap_cap = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_sendcap_addr, (void*)dst_sendcap_cap, 8);
    // info_report("dst_sendcap_cap: 0x%lx", *dst_sendcap_cap);
    bool* dst_sendcap_online = g_new0(bool, 1);
    // info_report("dst_sendcap_online: %d", *dst_sendcap_online);
    cpu_physical_memory_read(dst_sendcap_addr + 8 * 3, (void*)dst_sendcap_online, 1);
    *dst_sendcap_online = true;
    cpu_physical_memory_write(dst_sendcap_addr + 8 * 3, (void*)dst_sendcap_online, 1);
    Capability* send_cap_buf = g_new0(Capability, *dst_sendcap_len);
    cpu_physical_memory_read(*dst_sendcap_ptr, (void*)send_cap_buf, (*dst_sendcap_len) * sizeof(Capability));
    Capability* empty_sendcap_buf = g_new0(Capability, *dst_sendcap_len);
    cpu_physical_memory_write(*dst_sendcap_ptr, (void*)empty_sendcap_buf, (*dst_sendcap_len) * sizeof(Capability));
    g_free(empty_sendcap_buf);
    uint64_t empty_len = 0;
    cpu_physical_memory_write(dst_sendcap_addr + 8 * 2, (void*)&empty_len, 8);
    int i = 0;
    for (i = 0; i < *dst_sendcap_len; i++) {
        uint64_t task_id = send_cap_buf[i].task_id;
        uint64_t target_os_id = send_cap_buf[i].target.os_id;
        uint64_t target_proc_id = send_cap_buf[i].target.proc_id;
        uint64_t target_task_id = send_cap_buf[i].target.task_id;
        cap_queue_insert(cap_queue, task_id, target_os_id, target_proc_id, target_task_id);
    }
    g_free(dst_sendcap_ptr);
    g_free(dst_sendcap_len);
    g_free(dst_sendcap_cap);
    g_free(dst_sendcap_online);
    g_free(send_cap_buf);
}

static inline void load_recv_cap_queue(uint64_t dst_task_id, CapQueue* cap_queue, PriorityQueue* pq) {
    // load the ready tasks of dst_task from the memory.
    uint64_t dst_tcb = dst_task_id & (~(TCB_ALIGN - 1));
    uint64_t dst_recvcap_addr = dst_tcb + RECV_CAP_OFFSET;
    uint64_t* dst_recvcap_ptr = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_recvcap_addr + 8 * 1, (void*)dst_recvcap_ptr, 8);
    // info_report("dst_recvcap_ptr: 0x%lx", *dst_recvcap_ptr);
    uint64_t* dst_recvcap_len = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_recvcap_addr + 8 * 2, (void*)dst_recvcap_len, 8);
    // info_report("dst_recvcap_len: 0x%lx", *dst_recvcap_len);
    uint64_t* dst_recvcap_cap = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_recvcap_addr, (void*)dst_recvcap_cap, 8);
    // info_report("dst_recvcap_cap: 0x%lx", *dst_recvcap_cap);
    bool* dst_recvcap_online = g_new0(bool, 1);
    // info_report("dst_recvcap_online: %d", *dst_recvcap_online);
    cpu_physical_memory_read(dst_recvcap_addr + 8 * 3, (void*)dst_recvcap_online, 1);
    *dst_recvcap_online = true;
    cpu_physical_memory_write(dst_recvcap_addr + 8 * 3, (void*)dst_recvcap_online, 1);
    Capability* recv_cap_buf = g_new0(Capability, *dst_recvcap_len);
    cpu_physical_memory_read(*dst_recvcap_ptr, (void*)recv_cap_buf, (*dst_recvcap_len) * sizeof(Capability));
    Capability* empty_recvcap_buf = g_new0(Capability, *dst_recvcap_len);
    cpu_physical_memory_write(*dst_recvcap_ptr, (void*)empty_recvcap_buf, (*dst_recvcap_len) * sizeof(Capability));
    g_free(empty_recvcap_buf);
    uint64_t empty_len = 0;
    cpu_physical_memory_write(dst_recvcap_addr + 8 * 2, (void*)&empty_len, 8);
    int i = 0;
    for (i = 0; i < *dst_recvcap_len; i++) {
        uint64_t task_id = recv_cap_buf[i].task_id;
        uint64_t target_os_id = recv_cap_buf[i].target.os_id;
        uint64_t target_proc_id = recv_cap_buf[i].target.proc_id;
        uint64_t target_task_id = recv_cap_buf[i].target.task_id;
        cap_queue_insert(cap_queue, task_id, target_os_id, target_proc_id, target_task_id);
        uint64_t tcb = task_id & (~(TCB_ALIGN - 1));
        uint64_t tcb_status_addr = tcb + STATUS_OFFSET;
        uint64_t* status = g_new0(uint64_t, 1);
        cpu_physical_memory_read(tcb_status_addr, (void*)status, 8);
        if (*status == 1) {
            uint64_t priority = (task_id >> 1) % MAX_PRIORITY;
            pq_push(pq, priority, task_id);
        }
        g_free(status);
    }
    g_free(dst_recvcap_ptr);
    g_free(dst_recvcap_len);
    g_free(dst_recvcap_cap);
    g_free(dst_recvcap_online);
    g_free(recv_cap_buf);
}

static inline void hart_transaction_init(HartTransaction* hart_transaction) {
    hart_transaction->hypervisor_id = 0;
    identity_init(&hart_transaction->current);
    identity_init(&hart_transaction->send_intr_transaction);
    capability_init(&hart_transaction->register_receiver_transaction);
    capability_init(&hart_transaction->register_sender_transaction);
    hart_transaction->cause = NORMAL;
    hart_transaction->datasheet_id = -1;
}

static inline uint64_t hart_transaction_cur_os(HartTransaction* hart_transaction) {
    return hart_transaction->current.os_id;
}

static inline void hart_transaction_set_cur_os(HartTransaction* hart_transaction, uint64_t os_id) {
    hart_transaction->current.os_id = os_id;
    hart_transaction->current.proc_id = 0;
    hart_transaction->current.task_id = 0;
}

static inline uint64_t hart_transaction_cur_proc(HartTransaction* hart_transaction) {
    return hart_transaction->current.proc_id;
}

static inline void hart_transaction_set_cur_proc(HartTransaction* hart_transaction, uint64_t proc_id) {
    hart_transaction->current.proc_id = proc_id;
    hart_transaction->current.task_id = 0;
}

static inline void hart_transaction_clear_cur_proc(HartTransaction* hart_transaction) {
    hart_transaction->current.proc_id = 0;
    hart_transaction->current.task_id = 0;
}

static inline void hart_transaction_set_cur_task(HartTransaction* hart_transaction, uint64_t task_id) {
    hart_transaction->current.task_id = task_id;
}

static inline void hart_transaction_clear_cur_task(HartTransaction* hart_transaction) {
    hart_transaction->current.task_id = 0;
}

static inline void hart_transaction_set_datasheet_id(HartTransaction* hart_transaction, int64_t datasheet_id) {
    hart_transaction->datasheet_id = datasheet_id;
}

static inline void hart_transaction_cpy(HartTransaction* src, HartTransaction* dst) {
    hart_transaction_init(dst);
    dst->hypervisor_id = src->hypervisor_id;
    dst->current.os_id = src->current.os_id;
    dst->current.proc_id = src->current.proc_id;
    dst->datasheet_id = src->datasheet_id;
}

static inline void hart_transaction_clear(HartTransaction* hart_transaction) {
    hart_transaction_init(hart_transaction);
}

static inline void datasheet_init(DataSheet* datasheet) {
    pq_init(&datasheet->ready_queue);
    datasheet->device_cap = g_new0(Capability, MAX_IRQ);
    cap_queue_init(&datasheet->send_cap);
    cap_queue_init(&datasheet->recv_cap);
}

static inline uint64_t datasheet_ref_count(DataSheet* datasheet) {
    return datasheet->ref_count;
}

static inline void datasheet_ref_inc(DataSheet* datasheet) {
    datasheet->ref_count += 1;
}

static inline void datasheet_ref_dec(DataSheet* datasheet) {
    datasheet->ref_count -= 1;
}

static inline void modify_task_status(uint64_t target_task_id) {
    uint64_t target_tcb = target_task_id & (~(TCB_ALIGN - 1));
    uint64_t target_tcb_status_addr = target_tcb + STATUS_OFFSET;
    uint64_t* status = g_new0(uint64_t, 1);
    *status = 1;
    cpu_physical_memory_write(target_tcb_status_addr, (void*)status, 8);
    info_report("modify task status");
    g_free(status);
}

static inline uint64_t current(HartTransaction* hart_transaction) {
    if (hart_transaction->current.proc_id != 0) {
        return hart_transaction->current.proc_id;
    } else if (hart_transaction->current.os_id != 0) {
        return hart_transaction->current.os_id;
    } else if (hart_transaction->hypervisor_id != 0) {
        return hart_transaction->hypervisor_id;
    }
    return 0;
}

static inline int64_t check_online(RISCVMOICState* moic, int64_t exclude_idx, uint64_t value) {
    int i = 0;
    uint64_t hart_count = moic->hart_count;
    for(i = 0; i < hart_count; i++) {
        if (i == exclude_idx) {
            continue;
        }
        if (current(&moic->transactions[i]) == value) {
            return i;
        }
    }
    return -1;
}

static inline int64_t search_empty_datasheet(RISCVMOICState* moic, int64_t start_idx) {
    int64_t choose_idx = -1;
    uint64_t hart_count = moic->hart_count;
    int i = 0;
    for (i = start_idx; i < start_idx + hart_count; i++) {
        if (moic->datasheets[i % hart_count].ref_count == 0) {
            choose_idx = i % hart_count;
            break;
        }
    }
    return choose_idx;
}

static inline void rq_task_count_inc(uint64_t task_id) {
    // Increment the ready task count
    uint64_t tcb = task_id & (~(TCB_ALIGN - 1));
    uint64_t rq_addr = tcb + READY_QUEUE_OFFSET;
    uint64_t rq_count = 0;
    cpu_physical_memory_read(rq_addr + 8 * 4, (void*)&rq_count, 8);
    rq_count += 1;
    cpu_physical_memory_write(rq_addr + 8 * 4, (void*)&rq_count, 8);
}

static inline void rq_task_count_dec(uint64_t task_id) {
    // Decrement the ready task count
    uint64_t tcb = task_id & (~(TCB_ALIGN - 1));
    uint64_t rq_addr = tcb + READY_QUEUE_OFFSET;
    uint64_t rq_count = 0;
    cpu_physical_memory_read(rq_addr + 8 * 4, (void*)&rq_count, 8);
    rq_count -= 1;
    cpu_physical_memory_write(rq_addr + 8 * 4, (void*)&rq_count, 8);
}

static inline void send_cap_count_inc(uint64_t task_id) {
    // Increment the send capability count
    uint64_t tcb = task_id & (~(TCB_ALIGN - 1));
    uint64_t send_cap_addr = tcb + SEND_CAP_OFFSET;
    uint64_t send_cap_count = 0;
    cpu_physical_memory_read(send_cap_addr + 8 * 4, (void*)&send_cap_count, 8);
    send_cap_count += 1;
    cpu_physical_memory_write(send_cap_addr + 8 * 4, (void*)&send_cap_count, 8);
}

static inline void send_cap_count_dec(uint64_t task_id) {
    // Decrement the send capability count
    uint64_t tcb = task_id & (~(TCB_ALIGN - 1));
    uint64_t send_cap_addr = tcb + SEND_CAP_OFFSET;
    uint64_t send_cap_count = 0;
    cpu_physical_memory_read(send_cap_addr + 8 * 4, (void*)&send_cap_count, 8);
    send_cap_count -= 1;
    cpu_physical_memory_write(send_cap_addr + 8 * 4, (void*)&send_cap_count, 8);
}

static inline void recv_cap_count_inc(uint64_t task_id) {
    // Increment the send capability count
    uint64_t tcb = task_id & (~(TCB_ALIGN - 1));
    uint64_t recv_cap_addr = tcb + RECV_CAP_OFFSET;
    uint64_t recv_cap_count = 0;
    cpu_physical_memory_read(recv_cap_addr + 8 * 4, (void*)&recv_cap_count, 8);
    recv_cap_count += 1;
    cpu_physical_memory_write(recv_cap_addr + 8 * 4, (void*)&recv_cap_count, 8);
}

static inline void recv_cap_count_dec(uint64_t task_id) {
    // Increment the send capability count
    uint64_t tcb = task_id & (~(TCB_ALIGN - 1));
    uint64_t recv_cap_addr = tcb + RECV_CAP_OFFSET;
    uint64_t recv_cap_count = 0;
    cpu_physical_memory_read(recv_cap_addr + 8 * 4, (void*)&recv_cap_count, 8);
    recv_cap_count -= 1;
    cpu_physical_memory_write(recv_cap_addr + 8 * 4, (void*)&recv_cap_count, 8);
}

DeviceState *riscv_moic_create(hwaddr addr, uint32_t hart_count, uint32_t external_irq_count);

#endif
