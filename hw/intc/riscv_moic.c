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

/**********************************************************/

static uint64_t riscv_moic_read(void *opaque, hwaddr addr, unsigned size) {

    return 0;
}

static void riscv_moic_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {

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
        queue_init(&moic->moicharts[i].ready_queue);
        moic->moicharts[i].irq_table = g_new0(uint64_t, MAX_IRQ);
        moic->moicharts[i].send_cap = g_new0(Capability, MAX_IRQ);
        moic->moicharts[i].recv_cap = g_new0(Capability, MAX_IRQ);
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