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
#include "hw/irq.h"

static uint64_t riscv_moic_read(void *opaque, hwaddr addr, unsigned size) {
    RISCVMOICState *moic = opaque;
    int idx = addr / SIZEOF_PERHART;
    uint64_t op = addr % SIZEOF_PERHART;
    uint64_t hart_count = moic->hart_count;
    if (op == FETCH_OP) {
        int64_t datasheet_id = moic->transactions[idx].datasheet_id;
        if (datasheet_id >= 0 && datasheet_id < hart_count) {
            uint64_t task_id = pq_pop(&moic->datasheets[datasheet_id].ready_queue);
            if (task_id != 0) {
                uint64_t current_task = current(&moic->transactions[idx]);
                rq_task_count_dec(current_task);
                hart_transaction_set_cur_task(&moic->transactions[idx], task_id);
                qemu_log_mask(LOG_UNIMP, "hart %d Fetch, actual datasheet_id: %ld, taskid: %lx\n", idx, datasheet_id, task_id);
            } else {
                qemu_log_mask(LOG_UNIMP, "hart %d Fetch, actual datasheet_id: %ld, no task\n", idx, datasheet_id);
            }
            return task_id;
        } else {
            qemu_log_mask(LOG_UNIMP, "hart %d Fetch, but error\n", idx);
            return -1;
        }        
    } else if (op == CURRENT_OP) {
        return current(&moic->transactions[idx]);
    } else if (op == STATUS_OP) {
        int64_t datasheet_id = moic->transactions[idx].datasheet_id;
        uint64_t value = 0;
        if (datasheet_id >= 0 && datasheet_id < hart_count) {
            value |= moic->transactions[idx].cause;
            value |= (moic->datasheets[datasheet_id].ref_count << 4);
        }
        return value;
    } else {
        error_report("Operation is not supported");
    }
    return 0;
}

static void riscv_moic_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    RISCVMOICState *moic = opaque;
    int idx = addr / SIZEOF_PERHART;
    uint64_t op = addr % SIZEOF_PERHART;
    uint32_t hart_count = moic->hart_count;
    if (op == ADD_OP) {
        uint64_t current_task = current(&moic->transactions[idx]);
        rq_task_count_inc(current_task);
        int64_t datasheet_id = moic->transactions[idx].datasheet_id;
        assert(datasheet_id < hart_count);
        uint64_t priority = (value >> 1) % MAX_PRIORITY;
        pq_push(&moic->datasheets[datasheet_id].ready_queue, priority, value);
        qemu_log_mask(LOG_UNIMP, "hart %d Add, actual datasheet_id: %ld, task_id: %lx\n", idx, datasheet_id, value);
    } else if (op == SWITCH_HYPERVISOR_OP) {

    } else if (op == SWITCH_OS_OP) {
        if (value == 0) {
            // os -> non
            int64_t datasheet_id = moic->transactions[idx].datasheet_id;
            assert(datasheet_id < hart_count);
            datasheet_ref_dec(&moic->datasheets[datasheet_id]);
            if (datasheet_ref_count(&moic->datasheets[datasheet_id]) == 0) {
                // Store information into the memory.
                uint64_t src_task_id = moic->transactions[idx].current.os_id;
                qemu_log_mask(LOG_UNIMP, "hart %d os -> non, os is not run\n", idx);
                store_ready_queue(src_task_id, &moic->datasheets[datasheet_id].ready_queue);
                store_device_cap(src_task_id, moic->datasheets[datasheet_id].device_cap);
                store_send_cap_queue(src_task_id, &moic->datasheets[datasheet_id].send_cap);
                store_recv_cap_queue(src_task_id, &moic->datasheets[datasheet_id].recv_cap);
            } else {
                qemu_log_mask(LOG_UNIMP, "hart %d os -> non\n", idx);
            }
            hart_transaction_clear(&moic->transactions[idx]);
        } else if (value != 0) {
            // non -> os
            int64_t online_idx = check_online(moic, idx, value);
            if (online_idx >= 0 && online_idx < hart_count) {
                hart_transaction_cpy(&moic->transactions[online_idx], &moic->transactions[idx]);
                int64_t datasheet_id = moic->transactions[online_idx].datasheet_id;
                datasheet_ref_inc(&moic->datasheets[datasheet_id]);
                qemu_log_mask(LOG_UNIMP, "hart %d non -> os, os is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                return;
            }
            int64_t choose_idx = search_empty_datasheet(moic, idx);
            if (choose_idx >= 0) {
                qemu_log_mask(LOG_UNIMP, "hart %d non -> os, os is not online, choose idx: %ld, os_id: %lx\n", idx, choose_idx, value);
                hart_transaction_set_cur_os(&moic->transactions[idx], value);
                hart_transaction_set_datasheet_id(&moic->transactions[idx], choose_idx);
                datasheet_ref_inc(&moic->datasheets[choose_idx]);
                uint64_t dst_task_id = value;
                load_ready_queue(dst_task_id, &moic->datasheets[choose_idx].ready_queue);
                load_device_cap(dst_task_id, moic->datasheets[choose_idx].device_cap);
                load_send_cap_queue(dst_task_id, &moic->datasheets[idx].send_cap);
                load_recv_cap_queue(dst_task_id, &moic->datasheets[choose_idx].recv_cap, &moic->datasheets[choose_idx].ready_queue);
            }
        }
    } else if (op == SWITCH_PROC_OP) {
        if (value == 0 && hart_transaction_cur_os(&moic->transactions[idx]) != 0) {
            // process -> os
            qemu_log_mask(LOG_UNIMP, "hart %d process -> os\n", idx);
            int64_t datasheet_id = moic->transactions[idx].datasheet_id;
            assert(datasheet_id < hart_count);
            datasheet_ref_dec(&moic->datasheets[datasheet_id]);
            if (datasheet_ref_count(&moic->datasheets[datasheet_id]) == 0) {
                // Store information into the memory.
                uint64_t src_task_id = moic->transactions[idx].current.proc_id;
                qemu_log_mask(LOG_UNIMP, "hart %d process -> os, process is not run\n", idx);
                store_ready_queue(src_task_id, &moic->datasheets[datasheet_id].ready_queue);
                store_device_cap(src_task_id, moic->datasheets[datasheet_id].device_cap);
                store_send_cap_queue(src_task_id, &moic->datasheets[datasheet_id].send_cap);
                store_recv_cap_queue(src_task_id, &moic->datasheets[datasheet_id].recv_cap);
            } else {
                qemu_log_mask(LOG_UNIMP, "hart %d process -> os, process run on other hart\n", idx);
            }
            int64_t online_idx = check_online(moic, idx, moic->transactions[idx].current.os_id);
            if (online_idx >= 0 && online_idx < hart_count) {
                hart_transaction_cpy(&moic->transactions[online_idx], &moic->transactions[idx]);
                datasheet_id = moic->transactions[online_idx].datasheet_id;
                datasheet_ref_inc(&moic->datasheets[datasheet_id]);
                qemu_log_mask(LOG_UNIMP, "hart %d process -> os, os is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                return;
            }
            int64_t choose_idx = search_empty_datasheet(moic, idx);
            if (choose_idx >= 0) {
                qemu_log_mask(LOG_UNIMP, "hart %d process -> os, os is not online, choose idx: %ld\n", idx, choose_idx);
                hart_transaction_clear_cur_proc(&moic->transactions[idx]);
                hart_transaction_set_datasheet_id(&moic->transactions[idx], choose_idx);
                datasheet_ref_inc(&moic->datasheets[choose_idx]);
                uint64_t dst_task_id = hart_transaction_cur_os(&moic->transactions[idx]);
                load_ready_queue(dst_task_id, &moic->datasheets[choose_idx].ready_queue);
                load_device_cap(dst_task_id, moic->datasheets[choose_idx].device_cap);
                load_send_cap_queue(dst_task_id, &moic->datasheets[idx].send_cap);
                load_recv_cap_queue(dst_task_id, &moic->datasheets[choose_idx].recv_cap, &moic->datasheets[choose_idx].ready_queue);
            }
        } else if (value != 0 && hart_transaction_cur_os(&moic->transactions[idx]) != 0 && hart_transaction_cur_proc(&moic->transactions[idx]) == 0) {
            // os -> process
            int64_t datasheet_id = moic->transactions[idx].datasheet_id;
            assert(datasheet_id < hart_count);
            datasheet_ref_dec(&moic->datasheets[datasheet_id]);
            if (datasheet_ref_count(&moic->datasheets[datasheet_id]) == 0) {
                // Store information into the memory.
                uint64_t src_task_id = moic->transactions[idx].current.os_id;
                qemu_log_mask(LOG_UNIMP, "hart %d os -> process, os is not run\n", idx);
                store_ready_queue(src_task_id, &moic->datasheets[datasheet_id].ready_queue);
                store_device_cap(src_task_id, moic->datasheets[datasheet_id].device_cap);
                store_send_cap_queue(src_task_id, &moic->datasheets[datasheet_id].send_cap);
                store_recv_cap_queue(src_task_id, &moic->datasheets[datasheet_id].recv_cap);
            } else {
                qemu_log_mask(LOG_UNIMP, "hart %d os -> process, os run on other hart\n", idx);
            }
            int64_t online_idx = check_online(moic, idx, value);
            if (online_idx >= 0 && online_idx < hart_count) {
                hart_transaction_cpy(&moic->transactions[online_idx], &moic->transactions[idx]);
                datasheet_id = moic->transactions[online_idx].datasheet_id;
                datasheet_ref_inc(&moic->datasheets[datasheet_id]);
                qemu_log_mask(LOG_UNIMP, "hart %d os -> process, process is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                return;
            }
            int64_t choose_idx = search_empty_datasheet(moic, idx);
            if (choose_idx >= 0) {
                qemu_log_mask(LOG_UNIMP, "hart %d os -> process, process is not online, choose idx: %ld\n", idx, choose_idx);
                hart_transaction_set_cur_proc(&moic->transactions[idx], value);
                hart_transaction_set_datasheet_id(&moic->transactions[idx], choose_idx);
                datasheet_ref_inc(&moic->datasheets[choose_idx]);
                uint64_t dst_task_id = value;
                load_ready_queue(dst_task_id, &moic->datasheets[choose_idx].ready_queue);
                load_device_cap(dst_task_id, moic->datasheets[choose_idx].device_cap);
                load_send_cap_queue(dst_task_id, &moic->datasheets[idx].send_cap);
                load_recv_cap_queue(dst_task_id, &moic->datasheets[choose_idx].recv_cap, &moic->datasheets[choose_idx].ready_queue);
            }
        }
    } else if (op == REGISTER_RECV_TASK_OP) {
        moic->transactions[idx].register_receiver_transaction.task_id = value;
    } else if (op == REGISTER_RECV_TARGET_OS_OP) {
        moic->transactions[idx].register_receiver_transaction.target.os_id = value;
    } else if (op == REGISTER_RECV_TARGET_PROC_OP) {
        moic->transactions[idx].register_receiver_transaction.target.proc_id = value;
    } else if (op == REGISTER_RECV_TARGET_TASK_OP) {
        moic->transactions[idx].register_receiver_transaction.target.task_id = value;
        int64_t datasheet_id = moic->transactions[idx].datasheet_id;
        assert(datasheet_id < hart_count);
        // register receiver
        if (is_device_cap(&moic->transactions[idx].register_receiver_transaction)) {
            uint64_t task_id = moic->transactions[idx].register_receiver_transaction.task_id;
            moic->datasheets[datasheet_id].device_cap[value].task_id = task_id;
            qemu_log_mask(LOG_UNIMP, "hart %d register external irq\n", idx);
        } else {
            uint64_t task_id = moic->transactions[idx].register_receiver_transaction.task_id;
            uint64_t target_os_id = moic->transactions[idx].register_receiver_transaction.target.os_id;
            uint64_t target_proc_id = moic->transactions[idx].register_receiver_transaction.target.proc_id;
            uint64_t target_task_id = moic->transactions[idx].register_receiver_transaction.target.task_id;
            cap_queue_insert(&moic->datasheets[datasheet_id].recv_cap, task_id, target_os_id, target_proc_id, target_task_id);
            qemu_log_mask(LOG_UNIMP, "hart %d register ipc receiver.\n", idx);
            uint64_t current_task = current(&moic->transactions[idx]);
            recv_cap_count_inc(current_task);
        }
    } else if (op == REGISTER_SEND_TASK_OP) {
        moic->transactions[idx].register_sender_transaction.task_id = value;
    } else if (op == REGISTER_SEND_TARGET_OS_OP) {
        moic->transactions[idx].register_sender_transaction.target.os_id = value;
    } else if (op == REGISTER_SEND_TARGET_PROC_OP) {
        moic->transactions[idx].register_sender_transaction.target.proc_id = value;
    } else if (op == REGISTER_SEND_TARGET_TASK_OP) {
        moic->transactions[idx].register_sender_transaction.target.task_id = value;
        int64_t datasheet_id = moic->transactions[idx].datasheet_id;
        assert(datasheet_id < hart_count);
        // register sender
        uint64_t task_id = moic->transactions[idx].register_sender_transaction.task_id;
        uint64_t target_os_id = moic->transactions[idx].register_sender_transaction.target.os_id;
        uint64_t target_proc_id = moic->transactions[idx].register_sender_transaction.target.proc_id;
        uint64_t target_task_id = moic->transactions[idx].register_sender_transaction.target.task_id;
        if (target_os_id == 0 || task_id == 0 || target_task_id == 0) {
            return;
        }
        cap_queue_insert(&moic->datasheets[datasheet_id].send_cap, task_id, target_os_id, target_proc_id, target_task_id);
        qemu_log_mask(LOG_UNIMP, "hart %d register ipc sender.\n", idx);
        uint64_t current_task = current(&moic->transactions[idx]);
        send_cap_count_inc(current_task);
    } else if (op == SEND_INTR_OS_OP) {
        moic->transactions[idx].send_intr_transaction.os_id = value;
    } else if (op == SEND_INTR_PROC_OP) {
        moic->transactions[idx].send_intr_transaction.proc_id = value;
    } else if (op == SEND_INTR_TASK_OP) {
        // send intr
        moic->transactions[idx].send_intr_transaction.task_id = value;
        uint64_t target_os_id = moic->transactions[idx].send_intr_transaction.os_id;
        uint64_t target_proc_id = moic->transactions[idx].send_intr_transaction.proc_id;
        uint64_t target_task_id = moic->transactions[idx].send_intr_transaction.task_id;
        int64_t datasheet_id = moic->transactions[idx].datasheet_id;
        assert(datasheet_id < hart_count);
        // check whether the sender has the send_cap
        uint64_t task_id_sender = cap_queue_find(&moic->datasheets[datasheet_id].send_cap, target_os_id, target_proc_id, target_task_id);
        if (task_id_sender == 0) {
            return;
        }
        // check whether the receiver is online
        int64_t online_idx = -1;
        if (target_proc_id != 0) {
            // the receiver is process
            online_idx = check_online(moic, idx, target_proc_id);
            if (online_idx >= 0 && online_idx < hart_count) {       // receive process is online.
                datasheet_id = moic->transactions[online_idx].datasheet_id;
                uint64_t sender_os_id = moic->transactions[idx].current.os_id;
                uint64_t sender_proc_id = moic->transactions[idx].current.proc_id;
                uint64_t sender_task_id = moic->transactions[idx].current.task_id;
                uint64_t receiver_task = cap_queue_find(&moic->datasheets[datasheet_id].recv_cap, sender_os_id, sender_proc_id, sender_task_id);
                if (receiver_task != 0) {
                    assert(target_task_id == receiver_task);
                    uint64_t priority = (receiver_task >> 1) % MAX_PRIORITY;
                    pq_push(&moic->datasheets[datasheet_id].ready_queue, priority, receiver_task);
                    uint64_t current_task = current(&moic->transactions[online_idx]);
                    rq_task_count_inc(current_task);
                    // check whether the task is preemptible
                    bool is_preempt = (receiver_task & 1) != 0;
                    if (is_preempt) {
                        qemu_irq_raise(moic->usoft_irqs[online_idx]);
                        moic->transactions[online_idx].cause = PREEMPT;
                        qemu_log_mask(LOG_UNIMP, "hart %d send intr, preempt, receiver process is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                    } else {
                        qemu_log_mask(LOG_UNIMP, "hart %d send intr, receiver process is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                    }
                    return;
                } else {
                    qemu_log_mask(LOG_UNIMP, "hart %d send intr, receiver process is online, but has no receive task\n", idx);
                }
            } else {    // receive process is not online
                // check whether the os is online?
                online_idx = check_online(moic, idx, target_os_id);
                if (online_idx >= 0 && online_idx < hart_count) {       // the os that receive process belong to is online.
                    // wake the receive process
                    datasheet_id = moic->transactions[online_idx].datasheet_id;
                    uint64_t sender_os_id = moic->transactions[idx].current.os_id;
                    uint64_t sender_proc_id = moic->transactions[idx].current.proc_id;
                    uint64_t sender_task_id = moic->transactions[idx].current.task_id;                    
                    uint64_t receiver_process = cap_queue_find(&moic->datasheets[datasheet_id].recv_cap, sender_os_id, sender_proc_id, sender_task_id);
                    if (receiver_process != 0) {
                        assert(receiver_process == target_proc_id);
                        uint64_t priority = (receiver_process >> 1) % MAX_PRIORITY;
                        pq_push(&moic->datasheets[datasheet_id].ready_queue, priority, receiver_process);
                        uint64_t current_task = current(&moic->transactions[online_idx]);
                        rq_task_count_inc(current_task);
                        // The target task is in the recv_cap. So just modify the target task status.

                        // TODO: the task is in process address space. Need MMU.
                        modify_task_status(target_task_id);
                        // check the receive process is preemptible.
                        bool is_preempt = (receiver_process & 1) != 0;
                        if (is_preempt) {
                            qemu_irq_raise(moic->ssoft_irqs[online_idx]);
                            moic->transactions[online_idx].cause = PREEMPT;
                            qemu_log_mask(LOG_UNIMP, "hart %d send intr, preempt, receiver process is not online, os is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                        } else {
                            qemu_log_mask(LOG_UNIMP, "hart %d send intr, receiver process is not online, os is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                        }
                    } else {
                        qemu_log_mask(LOG_UNIMP, "hart %d send intr, receiver process is not online, but has no receive process\n", idx);
                    }
                } else {    // os is not online, all harts run process.
                    qemu_log_mask(LOG_UNIMP, "hart %d send intr, receiver process and os is not online\n", idx);
                    // modify process status
                    modify_task_status(target_proc_id);
                    // modify task status
                    modify_task_status(target_task_id);
                }
            }
        } else {
            // the receiver is os
            online_idx = check_online(moic, idx, target_os_id);
            if (online_idx >= 0 && online_idx < hart_count) {       // receive os is online.
                datasheet_id = moic->transactions[online_idx].datasheet_id;
                uint64_t sender_os_id = moic->transactions[idx].current.os_id;
                uint64_t sender_proc_id = moic->transactions[idx].current.proc_id;
                uint64_t sender_task_id = moic->transactions[idx].current.task_id;
                info_report("sender os: %lx, sender proc: %lx, sender task: %lx", sender_os_id, sender_proc_id, sender_task_id);
                uint64_t receiver_task = cap_queue_find(&moic->datasheets[datasheet_id].recv_cap, sender_os_id, sender_proc_id, sender_task_id);
                if (receiver_task != 0) {
                    assert(target_task_id == receiver_task);
                    uint64_t priority = (receiver_task >> 1) % MAX_PRIORITY;
                    pq_push(&moic->datasheets[datasheet_id].ready_queue, priority, receiver_task);
                    uint64_t current_task = current(&moic->transactions[online_idx]);
                    rq_task_count_inc(current_task);
                    // check whether the task is preemptible
                    bool is_preempt = (receiver_task & 1) != 0;
                    if (is_preempt) {
                        qemu_irq_raise(moic->ssoft_irqs[online_idx]);
                        moic->transactions[online_idx].cause = PREEMPT;
                        qemu_log_mask(LOG_UNIMP, "hart %d send intr, preempt, receiver os is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                    } else {
                        qemu_log_mask(LOG_UNIMP, "hart %d send intr, receiver os is running on hart %ld, actual datasheet_id: %ld\n", idx, online_idx, datasheet_id);
                    }
                    return;
                } else {
                    qemu_log_mask(LOG_UNIMP, "hart %d send intr, receiver os is online, but has no receive task\n", idx);
                }
            } else {
                qemu_log_mask(LOG_UNIMP, "hart %d send intr, receiver os is not online\n", idx);
            }
        }
    } else if (op == REMOVE_OP) {
        // // remove the target task
        // if (value != 0) {
        //     pq_remove(&moic->moicharts[idx].ready_queue, value);
        //     cap_queue_logout(&moic->moicharts[idx].send_cap, value);
        //     cap_queue_logout(&moic->moicharts[idx].recv_cap, value);
        //     device_cap_logout(moic->moicharts[idx].device_cap, value);
        //     // check whether the task is running on the other hart. No matter the task is os/process/normal task.
        //     int i = 0;
        //     for (i = 0; i < moic->hart_count; i++) {
        //         if ((i != idx)) {
        //             uint64_t proc_id = moic->moicharts[i].current.proc_id;
        //             uint64_t task_id = moic->moicharts[i].current.task_id;
        //             if (task_id == value) {     // normal task run on the other hart, send user interrupt
        //                 qemu_irq_pulse(moic->usoft_irqs[i]);
        //                 moic->moicharts[i].cause = KILL;
        //             } else if (proc_id == value) {  // process run on the other hart, send supervisor interrupt
        //                 qemu_irq_pulse(moic->ssoft_irqs[i]);
        //                 moic->moicharts[i].cause = KILL;
        //             }
        //             // not supported os, because of privilege level
        //         }
        //     }
        // }
    }  else if (op == DUMP_OP) {
        // if (value != 0) {
        //     uint64_t ptr = value;
        //     uint64_t hypervisor_id = moic->moicharts[idx].hypervisor_id;
        //     uint64_t current_os_id = moic->moicharts[idx].current.os_id;
        //     uint64_t current_proc_id = moic->moicharts[idx].current.proc_id;
        //     uint64_t current_task_id = moic->moicharts[idx].current.task_id;
        //     uint64_t rq_len = pq_len(&moic->moicharts[idx].ready_queue);
        //     uint64_t send_cap_len = cap_queue_len(&moic->moicharts[idx].send_cap);
        //     uint64_t recv_cap_len = cap_queue_len(&moic->moicharts[idx].recv_cap);
        //     cpu_physical_memory_write(ptr + 8 * 0, (void*)&hypervisor_id, 8);
        //     cpu_physical_memory_write(ptr + 8 * 1, (void*)&current_os_id, 8);
        //     cpu_physical_memory_write(ptr + 8 * 2, (void*)&current_proc_id, 8);
        //     cpu_physical_memory_write(ptr + 8 * 3, (void*)&current_task_id, 8);
        //     cpu_physical_memory_write(ptr + 8 * 4, (void*)&rq_len, 8);
        //     cpu_physical_memory_write(ptr + 8 * 5, (void*)&send_cap_len, 8);
        //     cpu_physical_memory_write(ptr + 8 * 6, (void*)&recv_cap_len, 8);
        // }
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
    moic->transactions = g_new0(HartTransaction, hart_count);
    moic->datasheets = g_new0(DataSheet, hart_count);
    // create output irqs
    moic->ssoft_irqs = g_malloc(sizeof(qemu_irq) * hart_count);
    qdev_init_gpio_out(dev, moic->ssoft_irqs, hart_count);
    moic->usoft_irqs = g_malloc(sizeof(qemu_irq) * hart_count);
    qdev_init_gpio_out(dev, moic->usoft_irqs, hart_count);
    int i = 0;
    for(i = 0; i < hart_count; i++) {
        hart_transaction_init(&moic->transactions[i]);
        datasheet_init(&moic->datasheets[i]);
        RISCVCPU *cpu = RISCV_CPU(qemu_get_cpu(i));
        /* Claim software interrupt bits */
        if (riscv_cpu_claim_interrupts(cpu, MIP_USIP) < 0) {
            error_report("USIP already claimed");
            exit(1);
        }
        if (riscv_cpu_claim_interrupts(cpu, MIP_SSIP) < 0) {
            error_report("SSIP already claimed");
            exit(1);
        }
    }

}

static Property riscv_moic_properties[] = {
    DEFINE_PROP_UINT32("hart_count", RISCVMOICState, hart_count, 0),
    DEFINE_PROP_UINT32("external_irq_count", RISCVMOICState, external_irq_count, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void riscv_moic_class_init(ObjectClass *obj, void *data) {
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
    int i = 0;
    for (i = 0; i < hart_count; i++) {
        CPUState *cpu = qemu_get_cpu(i);
        qdev_connect_gpio_out(dev, i, qdev_get_gpio_in(DEVICE(cpu), IRQ_S_SOFT));
        qdev_connect_gpio_out(dev, i + hart_count, qdev_get_gpio_in(DEVICE(cpu), IRQ_U_SOFT));
    }
    return dev;
}

static void riscv_moic_register_types(void) {
    type_register_static(&riscv_moic_info);
}

type_init(riscv_moic_register_types)