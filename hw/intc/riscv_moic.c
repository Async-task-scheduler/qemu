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
            // steal a task from other hart

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
    } else if (op == SWITCH_OS_OP) {
        // update the current os identity
        moic->moicharts[idx].current.os_id = value;
    } else if (op == SWITCH_PROC_OP) {
        moic->moicharts[idx].current.proc_id = value;
    } else if (op == REGISTER_RECV_TASK_OP) {
        moic->moicharts[idx].register_receiver_transaction.task_id = value;
    } else if (op == REGISTER_RECV_TARGET_OS_OP) {
        moic->moicharts[idx].register_receiver_transaction.target.os_id = value;
    } else if (op == REGISTER_RECV_TARGET_PROC_OP) {
        moic->moicharts[idx].register_receiver_transaction.target.proc_id = value;
    } else if (op == REGISTER_RECV_TARGET_TASK_OP) {
        // register receiver
        uint64_t task_id = moic->moicharts[idx].register_receiver_transaction.task_id;
        
        if (is_device_cap(&moic->moicharts[idx].register_receiver_transaction)) {
            // Sender is device, so don't record the sender identity.
            moic->moicharts[idx].device_cap[value].task_id = task_id;
            info_report("register external irq ok");
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
        uint64_t target_task_id = value;
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