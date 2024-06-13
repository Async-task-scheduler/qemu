/*
 * RISC-V MOIC (Multiple-Object Interaction Interrupt Controller) interface
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "target/riscv/cpu.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "hw/intc/riscv_moic.h"
#include "exec/cpu-common.h"

/******************* Utils ***************************************/

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

void pq_init(PriorityQueue* pq) {
    pq->task_queues = g_new0(Queue, MAX_PRIORITY);
    int i = 0;
    for(i = 0; i < MAX_PRIORITY; i++) {
        QSIMPLEQ_INIT(&pq->task_queues[i].head);
    }
}

void pq_push(PriorityQueue* pq, uint64_t priority, uint64_t data) {
    struct QueueEntry *task_entry = g_new0(struct QueueEntry, 1);
    task_entry->data = data;
    QSIMPLEQ_INSERT_TAIL(&pq->task_queues[priority].head, task_entry, next);
}

uint64_t pq_pop(PriorityQueue* pq) {
    uint64_t res = 0;
    int i = 0;
    for(i = 0; i < MAX_PRIORITY; i++) {
        QueueHead *head = &pq->task_queues[i].head;
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

bool pq_is_empty(PriorityQueue* pq) {
    int i = 0;
    for(i = 0; i < MAX_PRIORITY; i++) {
        QueueHead *head = &pq->task_queues[i].head;
        if (!QSIMPLEQ_EMPTY(head)) {
            return false;
        }
    }
    return true;
}

uint64_t pq_len(PriorityQueue* pq) {
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

uint64_t* pq_iter(PriorityQueue* pq) {
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
            // info_report("pq iter, task: 0x%lx", cur->data);
            j += 1;
            g_free(cur);
            QSIMPLEQ_REMOVE_HEAD(head, next);
        }
    }
    return task_buf;
}

// load the ready tasks in the hardware into the src_task
// load the ready tasks of dst_task in the memory into the hardware.
void switch_ready_queue(uint64_t src_task_id, uint64_t dst_task_id, PriorityQueue* pq) {
    uint64_t len = pq_len(pq);
    uint64_t* src_task_buf = pq_iter(pq);
    if (src_task_id == 0) {
        g_free(src_task_buf);
    } else {
        if (src_task_buf != NULL) {
            // checkout tasks of the src_task
            uint64_t src_tcb = src_task_id & (~(TCB_ALIGN - 1));
            uint64_t src_rq_addr = src_tcb + READY_QUEUE_OFFSET;
            uint64_t* src_rq_cap = g_new0(uint64_t, 1);
            cpu_physical_memory_read(src_rq_addr + 8 * 2, (void*)src_rq_cap, 8);
            assert(len < *src_rq_cap);
            g_free(src_rq_cap);
            // read the ready queue pointer
            uint64_t* src_rq_ptr = g_new0(uint64_t, 1);
            cpu_physical_memory_read(src_rq_addr, (void*)src_rq_ptr, 8);
            // info_report("src_rq_ptr: 0x%lx", *src_rq_ptr);
            cpu_physical_memory_write(*src_rq_ptr, (void*)src_task_buf, len * 8);
            g_free(src_rq_ptr);
            g_free(src_task_buf);
            uint64_t* src_rq_len = g_new0(uint64_t, 1);
            *src_rq_len = len;
            // info_report("src_rq_len: 0x%lx", *src_rq_len);
            cpu_physical_memory_write(src_rq_addr + 8 * 1, (void*)src_rq_len, 8);
            g_free(src_rq_len);
            bool* src_rq_online = g_new0(bool, 1);
            *src_rq_online = false;
            cpu_physical_memory_write(src_rq_addr + 8 * 3, (void*)src_rq_online, 1);
            g_free(src_rq_online);        
        }
    }
    // load the ready tasks of dst_task from the memory.
    uint64_t dst_tcb = dst_task_id & (~(TCB_ALIGN - 1));
    uint64_t dst_rq_addr = dst_tcb + READY_QUEUE_OFFSET;
    uint64_t* dst_rq_ptr = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_rq_addr, (void*)dst_rq_ptr, 8);
    // info_report("dst_rq_ptr: 0x%lx", *dst_rq_ptr);
    uint64_t* dst_rq_len = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_rq_addr + 8 * 1, (void*)dst_rq_len, 8);
    // info_report("dst_rq_len: 0x%lx", *dst_rq_len);
    uint64_t* dst_rq_cap = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_rq_addr + 8 * 2, (void*)dst_rq_cap, 8);
    // info_report("dst_rq_cap: 0x%lx", *dst_rq_cap);
    bool* dst_rq_online = g_new0(bool, 1);
    // info_report("dst_rq_online: %d", *dst_rq_online);
    cpu_physical_memory_read(dst_rq_addr + 8 * 3, (void*)dst_rq_online, 1);
    *dst_rq_online = true;
    cpu_physical_memory_write(dst_rq_addr + 8 * 3, (void*)dst_rq_online, 1);
    uint64_t* task_buf = g_new0(uint64_t, *dst_rq_len);
    cpu_physical_memory_read(*dst_rq_ptr, (void*)task_buf, (*dst_rq_len) * 8);
    uint64_t* empty_task_buf = g_new0(uint64_t, *dst_rq_len);
    cpu_physical_memory_write(*dst_rq_ptr, (void*)empty_task_buf, (*dst_rq_len) * 8);
    g_free(empty_task_buf);
    int i = 0;
    for (i = 0; i < *dst_rq_len; i++) {
        uint64_t task_id = task_buf[i];
        // info_report("task_id: 0x%lx", task_id);
        uint64_t priority = (task_id >> 1) % MAX_PRIORITY;
        pq_push(pq, priority, task_id);
    }
    g_free(dst_rq_ptr);
    g_free(dst_rq_len);
    g_free(dst_rq_cap);
    g_free(dst_rq_online);
    g_free(task_buf);
}

// switch device capability table
void switch_device_cap(uint64_t src_task_id, uint64_t dst_task_id, Capability* device_cap) {
    if (src_task_id != 0) {
        uint64_t src_tcb = src_task_id & (~(TCB_ALIGN - 1));
        uint64_t src_device_cap_ptr_addr = src_tcb + DEVICE_CAP_PTR_OFFSET;
        uint64_t* src_device_cap_ptr = g_new0(uint64_t, 1);
        cpu_physical_memory_read(src_device_cap_ptr_addr, (void*)src_device_cap_ptr, 8);
        cpu_physical_memory_write(*src_device_cap_ptr, (void*)device_cap, MAX_IRQ * sizeof(Capability));
        g_free(src_device_cap_ptr);
    }
    uint64_t dst_tcb = dst_task_id & (~(TCB_ALIGN - 1));
    uint64_t dst_device_cap_ptr_addr = dst_tcb + DEVICE_CAP_PTR_OFFSET;
    uint64_t* dst_device_cap_ptr = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_device_cap_ptr_addr, (void*)dst_device_cap_ptr, 8);
    cpu_physical_memory_read(*dst_device_cap_ptr, (void*)device_cap, MAX_IRQ * sizeof(Capability));
    g_free(dst_device_cap_ptr);
}


void cap_queue_init(CapQueue* cap_queue) {
    QSIMPLEQ_INIT(&cap_queue->head);
}

// If the cap_queue has the same item, it will do nothing.
// Otherwise, it will insert a new capability.
void cap_queue_insert(CapQueue* cap_queue, uint64_t task_id, 
    uint64_t target_os_id, 
    uint64_t target_proc_id, 
    uint64_t target_task_id) {
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

// If the target capability is in the cap_queue, remove it.
struct CapQueueEntry* cap_queue_remove(CapQueue* cap_queue, 
    uint64_t task_id, 
    uint64_t target_os_id, 
    uint64_t target_proc_id, 
    uint64_t target_task_id) {
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

bool is_device_cap(Capability* cap) {
    return (cap->target.os_id == 0) && (cap->target.proc_id == 0);
}

uint64_t cap_queue_len(CapQueue* cap_queue) {
    uint64_t len = 0;
    CapQueueHead* head = &cap_queue->head;
    struct CapQueueEntry* cur, *next_elem;
    QSIMPLEQ_FOREACH_SAFE(cur, head, next, next_elem) {
        len += 1;
    }
    return len;
}

Capability* cap_queue_iter(CapQueue* cap_queue) {
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
        g_free(cur);
        QSIMPLEQ_REMOVE_HEAD(head, next);
    }
    return cap_buf;
}

void switch_send_cap_queue(uint64_t src_task_id, uint64_t dst_task_id, CapQueue* cap_queue) {
    uint64_t len = cap_queue_len(cap_queue);
    Capability* src_cap_buf = cap_queue_iter(cap_queue);
    if (src_task_id == 0) {
        g_free(src_cap_buf);
    } else {
        if (src_cap_buf != NULL) {
            // checkout tasks of the src_task
            uint64_t src_tcb = src_task_id & (~(TCB_ALIGN - 1));
            uint64_t src_sendcap_addr = src_tcb + SEND_CAP_OFFSET;
            uint64_t* src_sendcap_cap = g_new0(uint64_t, 1);
            cpu_physical_memory_read(src_sendcap_addr + 8 * 2, (void*)src_sendcap_cap, 8);
            assert(len < *src_sendcap_cap);
            g_free(src_sendcap_cap);
            // read the ready queue pointer
            uint64_t* src_sendcap_ptr = g_new0(uint64_t, 1);
            cpu_physical_memory_read(src_sendcap_addr, (void*)src_sendcap_ptr, 8);
            // info_report("src_sendcap_ptr: 0x%lx", *src_sendcap_ptr);
            cpu_physical_memory_write(*src_sendcap_ptr, (void*)src_cap_buf, len * sizeof(Capability));
            g_free(src_sendcap_ptr);
            g_free(src_cap_buf);
            uint64_t* src_sendcap_len = g_new0(uint64_t, 1);
            *src_sendcap_len = len;
            // info_report("src_sendcap_len: 0x%lx", *src_sendcap_len);
            cpu_physical_memory_write(src_sendcap_addr + 8 * 1, (void*)src_sendcap_len, 8);
            g_free(src_sendcap_len);
            bool* src_sendcap_online = g_new0(bool, 1);
            *src_sendcap_online = false;
            cpu_physical_memory_write(src_sendcap_addr + 8 * 3, (void*)src_sendcap_online, 1);
            g_free(src_sendcap_online);        
        }
    }
    // load the ready tasks of dst_task from the memory.
    uint64_t dst_tcb = dst_task_id & (~(TCB_ALIGN - 1));
    uint64_t dst_sendcap_addr = dst_tcb + SEND_CAP_OFFSET;
    uint64_t* dst_sendcap_ptr = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_sendcap_addr, (void*)dst_sendcap_ptr, 8);
    // info_report("dst_sendcap_ptr: 0x%lx", *dst_sendcap_ptr);
    uint64_t* dst_sendcap_len = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_sendcap_addr + 8 * 1, (void*)dst_sendcap_len, 8);
    // info_report("dst_sendcap_len: 0x%lx", *dst_sendcap_len);
    uint64_t* dst_sendcap_cap = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_sendcap_addr + 8 * 2, (void*)dst_sendcap_cap, 8);
    // info_report("dst_sendcap_cap: 0x%lx", *dst_sendcap_cap);
    bool* dst_sendcap_online = g_new0(bool, 1);
    // info_report("dst_sendcap_online: %d", *dst_sendcap_online);
    cpu_physical_memory_read(dst_sendcap_addr + 8 * 3, (void*)dst_sendcap_online, 1);
    *dst_sendcap_online = true;
    cpu_physical_memory_write(dst_sendcap_addr + 8 * 3, (void*)dst_sendcap_online, 1);
    Capability* send_cap_buf = g_new0(Capability, *dst_sendcap_len);
    cpu_physical_memory_read(*dst_sendcap_ptr, (void*)send_cap_buf, (*dst_sendcap_len) * sizeof(Capability));
    uint64_t* empty_sendcap_buf = g_new0(uint64_t, *dst_sendcap_len);
    cpu_physical_memory_write(*dst_sendcap_ptr, (void*)empty_sendcap_buf, (*dst_sendcap_len) * sizeof(Capability));
    g_free(empty_sendcap_buf);
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

void switch_recv_cap_queue(uint64_t src_task_id, uint64_t dst_task_id, CapQueue* cap_queue) {
    uint64_t len = cap_queue_len(cap_queue);
    Capability* src_cap_buf = cap_queue_iter(cap_queue);
    if (src_task_id == 0) {
        g_free(src_cap_buf);
    } else {
        if (src_cap_buf != NULL) {
            // checkout tasks of the src_task
            uint64_t src_tcb = src_task_id & (~(TCB_ALIGN - 1));
            uint64_t src_recvcap_addr = src_tcb + RECV_CAP_OFFSET;
            uint64_t* src_recvcap_cap = g_new0(uint64_t, 1);
            cpu_physical_memory_read(src_recvcap_addr + 8 * 2, (void*)src_recvcap_cap, 8);
            assert(len < *src_recvcap_cap);
            g_free(src_recvcap_cap);
            // read the ready queue pointer
            uint64_t* src_recvcap_ptr = g_new0(uint64_t, 1);
            cpu_physical_memory_read(src_recvcap_addr, (void*)src_recvcap_ptr, 8);
            // info_report("src_recvcap_ptr: 0x%lx", *src_recvcap_ptr);
            cpu_physical_memory_write(*src_recvcap_ptr, (void*)src_cap_buf, len * sizeof(Capability));
            g_free(src_recvcap_ptr);
            g_free(src_cap_buf);
            uint64_t* src_recvcap_len = g_new0(uint64_t, 1);
            *src_recvcap_len = len;
            info_report("src_recvcap_len: 0x%lx", *src_recvcap_len);
            cpu_physical_memory_write(src_recvcap_addr + 8 * 1, (void*)src_recvcap_len, 8);
            g_free(src_recvcap_len);
            bool* src_recvcap_online = g_new0(bool, 1);
            *src_recvcap_online = false;
            cpu_physical_memory_write(src_recvcap_addr + 8 * 3, (void*)src_recvcap_online, 1);
            g_free(src_recvcap_online);        
        }
    }
    // load the ready tasks of dst_task from the memory.
    uint64_t dst_tcb = dst_task_id & (~(TCB_ALIGN - 1));
    uint64_t dst_recvcap_addr = dst_tcb + RECV_CAP_OFFSET;
    uint64_t* dst_recvcap_ptr = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_recvcap_addr, (void*)dst_recvcap_ptr, 8);
    // info_report("dst_recvcap_ptr: 0x%lx", *dst_recvcap_ptr);
    uint64_t* dst_recvcap_len = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_recvcap_addr + 8 * 1, (void*)dst_recvcap_len, 8);
    // info_report("dst_recvcap_len: 0x%lx", *dst_recvcap_len);
    uint64_t* dst_recvcap_cap = g_new0(uint64_t, 1);
    cpu_physical_memory_read(dst_recvcap_addr + 8 * 2, (void*)dst_recvcap_cap, 8);
    // info_report("dst_recvcap_cap: 0x%lx", *dst_recvcap_cap);
    bool* dst_recvcap_online = g_new0(bool, 1);
    // info_report("dst_recvcap_online: %d", *dst_recvcap_online);
    cpu_physical_memory_read(dst_recvcap_addr + 8 * 3, (void*)dst_recvcap_online, 1);
    *dst_recvcap_online = true;
    cpu_physical_memory_write(dst_recvcap_addr + 8 * 3, (void*)dst_recvcap_online, 1);
    Capability* recv_cap_buf = g_new0(Capability, *dst_recvcap_len);
    cpu_physical_memory_read(*dst_recvcap_ptr, (void*)recv_cap_buf, (*dst_recvcap_len) * sizeof(Capability));
    uint64_t* empty_recvcap_buf = g_new0(uint64_t, *dst_recvcap_len);
    cpu_physical_memory_write(*dst_recvcap_ptr, (void*)empty_recvcap_buf, (*dst_recvcap_len) * sizeof(Capability));
    g_free(empty_recvcap_buf);
    int i = 0;
    for (i = 0; i < *dst_recvcap_len; i++) {
        uint64_t task_id = recv_cap_buf[i].task_id;
        uint64_t target_os_id = recv_cap_buf[i].target.os_id;
        uint64_t target_proc_id = recv_cap_buf[i].target.proc_id;
        uint64_t target_task_id = recv_cap_buf[i].target.task_id;
        cap_queue_insert(cap_queue, task_id, target_os_id, target_proc_id, target_task_id);
    }
    g_free(dst_recvcap_ptr);
    g_free(dst_recvcap_len);
    g_free(dst_recvcap_cap);
    g_free(dst_recvcap_online);
    g_free(recv_cap_buf);
}

/**********************************************************/

static uint64_t riscv_moic_read(void *opaque, hwaddr addr, unsigned size) {
    RISCVMOICState *moic = opaque;
    int idx = addr / SIZEOF_PERHART;
    uint64_t op = addr % SIZEOF_PERHART;
    if (op == FETCH_OP) {
        uint64_t task_id = pq_pop(&moic->moicharts[idx].ready_queue);
        if (task_id != 0) {
            moic->moicharts[idx].current.task_id = task_id;
            return task_id;
        } else {
            // steal a task in the same process on the local ready queue of other harts.
            uint64_t proc_id = moic->moicharts[idx].current.proc_id;
            uint32_t hart_count = moic->hart_count;
            int i = 0;
            for(i = 0; i < hart_count; i++) {
                if ((i != idx) && (moic->moicharts[i].current.proc_id == proc_id)) {
                    task_id = pq_pop(&moic->moicharts[i].ready_queue);
                    if (task_id != 0) {
                        moic->moicharts[idx].current.task_id = task_id;
                        return task_id;
                    } 
                }
            }
        }
    } else {
        error_report("Operation is not supported");
    }
    return 0;
}

static void riscv_moic_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    RISCVMOICState *moic = opaque;
    int idx = addr / SIZEOF_PERHART;
    uint64_t op = addr % SIZEOF_PERHART;
    if (op == ADD_OP) {
        uint64_t priority = (value >> 1) % MAX_PRIORITY;
        pq_push(&moic->moicharts[idx].ready_queue, priority, value);
    } else if (op == SWITCH_HYPERVISOR_OP) {
        // non -> hypervisor
        if (value != 0) {
            uint64_t dst_task_id = moic->moicharts[idx].hypervisor_id = value;
            switch_ready_queue(0, dst_task_id, &moic->moicharts[idx].ready_queue);
            switch_device_cap(0, dst_task_id, moic->moicharts[idx].device_cap);
            switch_send_cap_queue(0, dst_task_id, &moic->moicharts[idx].send_cap);
            switch_recv_cap_queue(0, dst_task_id, &moic->moicharts[idx].recv_cap);
        }
    } else if (op == SWITCH_OS_OP) {
        // update the current os identity
        if (value == 0 && moic->moicharts[idx].hypervisor_id != 0) {
            uint64_t src_task_id = moic->moicharts[idx].current.os_id;
            uint64_t dst_task_id = moic->moicharts[idx].hypervisor_id;
            // if no hypervisor or no os, return directly
            if (dst_task_id == 0 || src_task_id == 0) {
                return;
            } 
            // os -> hypervisor
            moic->moicharts[idx].current.proc_id = 0;
            switch_ready_queue(src_task_id, dst_task_id, &moic->moicharts[idx].ready_queue);
            switch_device_cap(src_task_id, dst_task_id, moic->moicharts[idx].device_cap);
            switch_send_cap_queue(src_task_id, dst_task_id, &moic->moicharts[idx].send_cap);
            switch_recv_cap_queue(src_task_id, dst_task_id, &moic->moicharts[idx].recv_cap);
        } else if (value != 0) {
            // hypervisor -> os | non -> os
            moic->moicharts[idx].current.proc_id = 0;
            uint64_t src_task_id = moic->moicharts[idx].hypervisor_id;
            uint64_t dst_task_id = moic->moicharts[idx].current.os_id = value;
            switch_ready_queue(src_task_id, dst_task_id, &moic->moicharts[idx].ready_queue);
            switch_device_cap(src_task_id, dst_task_id, moic->moicharts[idx].device_cap);
            switch_send_cap_queue(src_task_id, dst_task_id, &moic->moicharts[idx].send_cap);
            switch_recv_cap_queue(src_task_id, dst_task_id, &moic->moicharts[idx].recv_cap);
        }
    } else if (op == SWITCH_PROC_OP) {
        if (value == 0 && moic->moicharts[idx].current.os_id != 0) {
            // switch to os
            uint64_t src_task_id = moic->moicharts[idx].current.proc_id;
            uint64_t dst_task_id = moic->moicharts[idx].current.os_id;
            if (src_task_id == 0 || dst_task_id == 0) {
                return;
            }
            switch_ready_queue(src_task_id, dst_task_id, &moic->moicharts[idx].ready_queue);
            switch_device_cap(src_task_id, dst_task_id, moic->moicharts[idx].device_cap);
            switch_send_cap_queue(src_task_id, dst_task_id, &moic->moicharts[idx].send_cap);
            switch_recv_cap_queue(src_task_id, dst_task_id, &moic->moicharts[idx].recv_cap);
        } else if (value != 0 && moic->moicharts[idx].current.os_id != 0) {
            // switch to process
            uint64_t src_task_id = moic->moicharts[idx].current.os_id;
            uint64_t dst_task_id = moic->moicharts[idx].current.proc_id = value;
            switch_ready_queue(src_task_id, dst_task_id, &moic->moicharts[idx].ready_queue);
            switch_device_cap(src_task_id, dst_task_id, moic->moicharts[idx].device_cap);
            switch_send_cap_queue(src_task_id, dst_task_id, &moic->moicharts[idx].send_cap);
            switch_recv_cap_queue(src_task_id, dst_task_id, &moic->moicharts[idx].recv_cap);
        }
    } else if (op == REGISTER_RECV_TASK_OP) {
        moic->moicharts[idx].register_receiver_transaction.task_id = value;
    } else if (op == REGISTER_RECV_TARGET_OS_OP) {
        moic->moicharts[idx].register_receiver_transaction.target.os_id = value;
    } else if (op == REGISTER_RECV_TARGET_PROC_OP) {
        moic->moicharts[idx].register_receiver_transaction.target.proc_id = value;
    } else if (op == REGISTER_RECV_TARGET_TASK_OP) {
        moic->moicharts[idx].register_receiver_transaction.target.task_id = value;
        // register receiver
        uint64_t task_id = moic->moicharts[idx].register_receiver_transaction.task_id;
        
        if (is_device_cap(&moic->moicharts[idx].register_receiver_transaction)) {
            // Sender is device, so don't record the sender identity.
            moic->moicharts[idx].device_cap[value].task_id = task_id;
            // info_report("register external irq ok");
        } else {
            // Sender is another task, record the sender identity.
            // Remove the duplicate item
            uint64_t target_os_id = moic->moicharts[idx].register_receiver_transaction.target.os_id;
            uint64_t target_proc_id = moic->moicharts[idx].register_receiver_transaction.target.proc_id;
            uint64_t target_task_id = moic->moicharts[idx].register_receiver_transaction.target.task_id;
            cap_queue_insert(&moic->moicharts[idx].recv_cap, task_id, target_os_id, target_proc_id, target_task_id);

        }

    } else if (op == REGISTER_SEND_TASK_OP) {
        moic->moicharts[idx].register_sender_transaction.task_id = value;
    } else if (op == REGISTER_SEND_TARGET_OS_OP) {
        moic->moicharts[idx].register_sender_transaction.target.os_id = value;
    } else if (op == REGISTER_SEND_TARGET_PROC_OP) {
        moic->moicharts[idx].register_sender_transaction.target.proc_id = value;
    } else if (op == REGISTER_SEND_TARGET_TASK_OP) {
        // register sender
        uint64_t task_id = moic->moicharts[idx].register_sender_transaction.task_id;
        uint64_t target_os_id = moic->moicharts[idx].register_sender_transaction.target.os_id;
        uint64_t target_proc_id = moic->moicharts[idx].register_sender_transaction.target.proc_id;
        uint64_t target_task_id = moic->moicharts[idx].register_sender_transaction.target.task_id = value;
        if (target_os_id == 0 || target_proc_id == 0 || target_task_id == 0 || task_id == 0) {
            return;
        }
        cap_queue_insert(&moic->moicharts[idx].send_cap, task_id, target_os_id, target_proc_id, target_task_id);

    } else if (op == SEND_INTR_OS_OP) {
        moic->moicharts[idx].send_intr_transaction.os_id = value;
    } else if (op == SEND_INTR_PROC_OP) {
        moic->moicharts[idx].send_intr_transaction.proc_id = value;
    } else if (op == SEND_INTR_TASK_OP) {
        // send intr
        moic->moicharts[idx].send_intr_transaction.task_id = value;
        
    } else {
        error_report("Operation is not supported");
    }
}

static void riscv_moic_irq_request(void *opaque, int irq, int level) {

}

static const MemoryRegionOps riscv_moic_ops = {
    .read = riscv_moic_read,
    .write = riscv_moic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 8,
        .max_access_size = 8
    },
    .impl = {
        .min_access_size = 8,
        .max_access_size = 8
    }
};

static void riscv_moic_realize(DeviceState *dev, Error **errp)
{
    
    RISCVMOICState *moic = RISCV_MOIC(dev);

    info_report("riscv moic realize");

    memory_region_init_io(&moic->mmio, OBJECT(dev), &riscv_moic_ops, moic,
                          TYPE_RISCV_MOIC, RISCV_MOIC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &moic->mmio);

    info_report("low 0x%x high 0x%x", (uint32_t)moic->mmio.addr, (uint32_t)moic->mmio.size);

    // init external_irqs
    uint32_t external_irq_count = moic->external_irq_count;
    moic->external_irqs = g_malloc(sizeof(qemu_irq) * external_irq_count);
    qdev_init_gpio_in(dev, riscv_moic_irq_request, external_irq_count);

    // init moic_hart
    uint32_t hart_count = moic->hart_count;
    moic->moicharts = g_new0(MoicHart, hart_count);
    int i = 0;
    for(i = 0; i < hart_count; i++) {
        pq_init(&moic->moicharts[i].ready_queue);
        cap_queue_init(&moic->moicharts[i].send_cap);
        cap_queue_init(&moic->moicharts[i].recv_cap);
        moic->moicharts[i].device_cap = g_new0(Capability, MAX_IRQ);
    }

}

static Property riscv_moic_properties[] = {
    DEFINE_PROP_UINT32("hart_count", RISCVMOICState, hart_count, 0),
    DEFINE_PROP_UINT32("external_irq_count", RISCVMOICState, external_irq_count, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void riscv_moic_class_init(ObjectClass *obj, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(obj);
    device_class_set_props(dc, riscv_moic_properties);
    dc->realize = riscv_moic_realize;
    
}

static const TypeInfo riscv_moic_info = {
    .name          = TYPE_RISCV_MOIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RISCVMOICState),
    .class_init    = riscv_moic_class_init,
};

DeviceState *riscv_moic_create(hwaddr addr, uint32_t hart_count, uint32_t external_irq_count) {
    qemu_log("create moic\n");

    DeviceState *dev = qdev_new(TYPE_RISCV_MOIC);
    qdev_prop_set_uint32(dev, "hart_count", hart_count);
    qdev_prop_set_uint32(dev, "external_irq_count", external_irq_count);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);


    return dev;
}

static void riscv_moic_register_types(void)
{
    type_register_static(&riscv_moic_info);
}

type_init(riscv_moic_register_types)