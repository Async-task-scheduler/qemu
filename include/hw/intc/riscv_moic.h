/*
 * RISC-V MOIC (Multiple-Object Interaction Interrupt Controller) interface
 */

#ifndef HW_RISCV_MOIC_H
#define HW_RISCV_MOIC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_RISCV_MOIC "riscv.moic"

typedef struct RISCVMOICState RISCVMOICState;
DECLARE_INSTANCE_CHECKER(RISCVMOICState, RISCV_MOIC, TYPE_RISCV_MOIC)

#define MOIC_MMIO_PAGE_SHIFT          12
#define MOIC_MMIO_PAGE_SZ             (1UL << MOIC_MMIO_PAGE_SHIFT)
#define MOIC_MMIO_SIZE(__num_pages)   ((__num_pages) * MOIC_MMIO_PAGE_SZ)

#define MOIC_MMIO_HART_GUEST_MAX_BTIS 6
#define MOIC_MMIO_GROUP_MIN_SHIFT     24

#define MOIC_HART_NUM_GUESTS(__guest_bits)           \
    (1U << (__guest_bits))
#define MOIC_HART_SIZE(__guest_bits)                 \
    (MOIC_HART_NUM_GUESTS(__guest_bits) * MOIC_MMIO_PAGE_SZ)
#define MOIC_GROUP_NUM_HARTS(__hart_bits)            \
    (1U << (__hart_bits))
#define MOIC_GROUP_SIZE(__hart_bits, __guest_bits)   \
    (MOIC_GROUP_NUM_HARTS(__hart_bits) * MOIC_HART_SIZE(__guest_bits))

struct RISCVMOICState {
    /*< private >*/
    SysBusDevice parent_obj;
    qemu_irq *external_irqs;

    /*< public >*/
    MemoryRegion mmio;
    uint32_t num_eistate;
    uint32_t *eidelivery;
    uint32_t *eithreshold;
    uint32_t *eistate;

    /* config */
    bool mmode;
    uint32_t hartid;
    uint32_t num_pages;
    uint32_t num_irqs;
};

DeviceState *riscv_moic_create(hwaddr addr, uint32_t hartid, bool mmode,
                                uint32_t num_pages, uint32_t num_ids);

#endif
