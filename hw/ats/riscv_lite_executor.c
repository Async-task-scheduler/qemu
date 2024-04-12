/*
 * RISC-V LITE EXECUTOR 
 *
 * Copyright (c) 2021 Western Digital Corporation or its affiliates.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
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
#include "hw/ats/riscv_lite_executor.h"
#include "qemu/queue.h"

static uint64_t riscv_lite_executor_read(void *opaque, hwaddr addr, unsigned size)
{
    assert(size == 8);
    RISCVLiteExecutor *lite_executor = opaque;

    unsigned process_vec_addr = addr;
    unsigned process_index = process_vec_addr / PROCESS_MMIO_SIZE;
    unsigned process_addr = process_vec_addr % PROCESS_MMIO_SIZE;
    assert(process_index < MAX_PROCESS);
    if (!proc_status_is_online(&lite_executor->pst[process_index])) {
        info_report("THE TARGET PROCESS IS OFFLINE");
        return 0;
    }
    if(process_addr < IPC_HANDLER_MMIO_OFFSET) {
        // Priority scheduler area
        unsigned ps_addr = process_addr;
        if(ps_addr < PS_ENQUEUE_MMIO_OFFSET) {
            // Priority scheduler dequeue field
            uint64_t index = lite_executor->pst[process_index].index;
            // info_report(" READ LITE EXECUTOR: addr 0x%08lx -> Priority scheduler dequeue field, process %d", addr, process_index);
            return ps_pop(&lite_executor->pschedulers[index]);
        }
        else {
            // Priority scheduler enqueue field
            unsigned enqueue_vec_addr = ps_addr - PS_ENQUEUE_MMIO_OFFSET;
            unsigned enqueue_index = enqueue_vec_addr / PS_ENQUEUE_MMIO_SIZE;
            info_report(" READ LITE EXECUTOR: addr 0x%08lx -> Priority scheduler enqueue field, process %d, queue %d", addr, process_index, enqueue_index);
            return 0;
        }
    }
    else if(process_addr < EXTERNAL_INTERRUPT_HANDLER_MMIO_OFFSET) {
        // IPC handler area
        unsigned ih_addr = process_addr - IPC_HANDLER_MMIO_OFFSET;
        if(ih_addr < IH_BQ_MMIO_OFFSET) {
            // IPC handler send field
            info_report(" READ LITE EXECUTOR: addr 0x%08lx -> IPC handler send field, process %d", addr, process_index);
            return 0;
        }
        else {
            // IPC handler bq field
            unsigned bq_vec_addr = ih_addr - IH_BQ_MMIO_OFFSET;
            unsigned bq_index = bq_vec_addr / IH_BQ_MMIO_SIZE;
            info_report(" READ LITE EXECUTOR: addr 0x%08lx -> IPC handler bq field, process %d, bq %d", addr, process_index, bq_index);
            return 0;
        }
    }
    else {
        // Extern interrupt handler area 
        unsigned eih_addr = process_addr - EXTERNAL_INTERRUPT_HANDLER_MMIO_OFFSET;
        unsigned enqueue_vec_addr = eih_addr - EIH_ENQUEUE_MMIO_OFFSET;
        unsigned enqueue_index = enqueue_vec_addr / EIH_ENQUEUE_MMIO_SIZE;
        info_report(" READ LITE EXECUTOR: addr 0x%08lx -> Extern interrupt handler enqueue field, process %d, queue %d", addr, process_index, enqueue_index);
        return 0;
    }
    return 0;
}

static void riscv_lite_executor_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    assert(size == 8);
    RISCVLiteExecutor *lite_executor = opaque;

    // Process area
    unsigned process_vec_addr = addr;
    unsigned process_index = process_vec_addr / PROCESS_MMIO_SIZE;
    unsigned process_addr = process_vec_addr % PROCESS_MMIO_SIZE;
    assert(process_index < MAX_PROCESS);
    if (!proc_status_is_online(&lite_executor->pst[process_index])) {
        info_report("THE TARGET PROCESS IS OFFLINE");
        return;
    }
    if(process_addr < IPC_HANDLER_MMIO_OFFSET) {
        // Priority scheduler area
        unsigned ps_addr = process_addr;
        if(ps_addr < PS_ENQUEUE_MMIO_OFFSET) {
            // Priority scheduler dequeue field
            info_report("WRITE LITE EXECUTOR: addr 0x%08lx, value 0x%016lx -> Priority scheduler dequeue field, process %d", addr, value, process_index);
        }
        else {
            // Priority scheduler enqueue field
            unsigned enqueue_vec_addr = ps_addr - PS_ENQUEUE_MMIO_OFFSET;
            unsigned enqueue_index = enqueue_vec_addr / PS_ENQUEUE_MMIO_SIZE;
            assert(enqueue_index < MAX_TASK_QUEUE);
            uint64_t index = lite_executor->pst[process_index].index;
            ps_push(&lite_executor->pschedulers[index], enqueue_index, value);
            info_report("WRITE LITE EXECUTOR: addr 0x%08lx, value 0x%016lx -> Priority scheduler enqueue field, process %d, queue %d", addr, value, process_index, enqueue_index);
        }
    }
    else if(process_addr < EXTERNAL_INTERRUPT_HANDLER_MMIO_OFFSET) {
        // IPC handler area
        unsigned ih_addr = process_addr - IPC_HANDLER_MMIO_OFFSET;
        if(ih_addr < IH_BQ_MMIO_OFFSET) {
            // IPC handler send field
            info_report("WRITE LITE EXECUTOR: addr 0x%08lx, value 0x%016lx -> IPC handler send field, process %d", addr, value, process_index);
        }
        else {
            // IPC handler bq field
            unsigned bq_vec_addr = ih_addr - IH_BQ_MMIO_OFFSET;
            unsigned bq_index = bq_vec_addr / IH_BQ_MMIO_SIZE;
            info_report("WRITE LITE EXECUTOR: addr 0x%08lx, value 0x%016lx -> IPC handler bq field, process %d, bq %d", addr, value, process_index, bq_index);
        }
    }
    else {
        // Extern interrupt handler area
        unsigned eih_addr = process_addr - EXTERNAL_INTERRUPT_HANDLER_MMIO_OFFSET;
        unsigned enqueue_vec_addr = eih_addr - EIH_ENQUEUE_MMIO_OFFSET;
        unsigned enqueue_index = enqueue_vec_addr / EIH_ENQUEUE_MMIO_SIZE;
        uint64_t index = lite_executor->pst[process_index].index;
        eih_push(&lite_executor->eihs[index], enqueue_index, value);
        info_report("WRITE LITE EXECUTOR: addr 0x%08lx, value 0x%016lx -> Extern interrupt handler enqueue field, process %d, queue %d", addr, value, process_index, enqueue_index);
    }
}

static const MemoryRegionOps riscv_lite_executor_ops = {
    .read = riscv_lite_executor_read,
    .write = riscv_lite_executor_write,
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

static void riscv_lite_executor_irq_request(void *opaque, int irq, int level)
{
    // 这句是用来测试是否成功接收中断的，但保留这句会在一直打印刷屏，所以注释了。
    // info_report("RISCV LITE EXECUTOR RECEIVE IRQ: %d", irq);

    RISCVLiteExecutor *lite_executor = opaque;
    // test serial interrupt handler
    assert(irq < MAX_EXTERNAL_INTR);
    uint64_t handler = eih_pop(&lite_executor->eihs[0], irq); // 目前将所有中断交由0号进程处理
    if (handler != 0) {
        uint64_t index = lite_executor->pst[0].index;
        ps_push(&lite_executor->pschedulers[index], 0, handler);
        info_report("external interrupt handler 0x%016lx, process %d", handler, 0);
    }

    // 外部中断到来后的操作，待实现
}

static void riscv_lite_executor_realize(DeviceState *dev, Error **errp)
{
    
    RISCVLiteExecutor *lite_executor = RISCV_LITE_EXECUTOR(dev);

    info_report("RISCV LITE EXECUTOR REALIZE");

    memory_region_init_io(&lite_executor->mmio, OBJECT(dev), &riscv_lite_executor_ops, lite_executor,
                          TYPE_RISCV_LITE_EXECUTOR, RISCV_LITE_EXECUTOR_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &lite_executor->mmio);

    info_report("LOW 0x%x HIGH 0x%x", (uint32_t)lite_executor->mmio.addr, (uint32_t)lite_executor->mmio.size);

    // init process status table 
    lite_executor->pst = g_new0(ProcStatus, MAX_PROCESS);
    int i = 0;
    for(i = 0; i < MAX_PROCESS; i++) {
        proc_status_init(&lite_executor->pst[i]);
        // TODO: This line should be removed. But We just assue that the top MAX_ONLINE_STRUCT_GROUP processes are online.
        if (i < MAX_ONLINE_STRUCT_GROUP) {
            proc_status_set_online(&lite_executor->pst[i]);
            proc_status_add_map(&lite_executor->pst[i], i);
        }
    }
    // init external interrupt handler queues
    lite_executor->eihs = g_new0(ExternalInterruptHandler, MAX_ONLINE_STRUCT_GROUP);
    for(i = 0; i < MAX_ONLINE_STRUCT_GROUP; i++) {
        eih_init(&lite_executor->eihs[i]);
    }
    // init priority schedulers
    lite_executor->pschedulers = g_new0(PriorityScheduler, MAX_ONLINE_STRUCT_GROUP);
    for(i = 0; i < MAX_ONLINE_STRUCT_GROUP; i++) {
        ps_init(&lite_executor->pschedulers[i]);
    }

    // init rw expect addr, -1 indicates no expect address
    lite_executor->expect_read_addr = (uint64_t)(-1);
    lite_executor->expect_write_addr = (uint64_t)(-1);

    //注册GPIO端口，参考sifive_plic.c:380..387
    {
        qdev_init_gpio_in(dev, riscv_lite_executor_irq_request, lite_executor->num_sources);

        //这几句应该是用于向CPU通知中断的输出端口？所以应该在lite_executor中不需要？
        // s->s_external_irqs = g_malloc(sizeof(qemu_irq) * s->num_harts);
        // qdev_init_gpio_out(dev, s->s_external_irqs, s->num_harts);

        // s->m_external_irqs = g_malloc(sizeof(qemu_irq) * s->num_harts);
        // qdev_init_gpio_out(dev, s->m_external_irqs, s->num_harts);
    }

    // lite executor应该不需要调用riscv_cpu_claim_interrupts函数，因为它不会直接向CPU发中断信号。
}

// 为调度器新增的属性，参考了sifive_plic.c:426..442
static Property riscv_lite_executor_properties[] = {
    /* number of interrupt sources including interrupt source 0 */
    DEFINE_PROP_UINT32("num-sources", RISCVLiteExecutor, num_sources, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void riscv_lite_executor_class_init(ObjectClass *obj, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(obj);

    // 为调度器注册新增的属性
    device_class_set_props(dc, riscv_lite_executor_properties);

    dc->realize = riscv_lite_executor_realize;
}

static const TypeInfo riscv_lite_executor_info = {
    .name          = TYPE_RISCV_LITE_EXECUTOR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RISCVLiteExecutor),
    .class_init    = riscv_lite_executor_class_init,
};

DeviceState *riscv_lite_executor_create(hwaddr addr, uint32_t num_sources)
{
    qemu_log("CREATE LITE EXECUTOR\n"); // 改了create的大小写:)

    DeviceState *dev = qdev_new(TYPE_RISCV_LITE_EXECUTOR);

    // 设置新增的属性
    qdev_prop_set_uint32(dev, "num-sources", num_sources);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);

    // 因为lite_executor没有和CPU连接，所以不需要调用`qdev_connect_gpio_out`连接到CPU的中断线。

    return dev;
}

static void riscv_lite_executor_register_types(void)
{
    type_register_static(&riscv_lite_executor_info);
}

type_init(riscv_lite_executor_register_types)