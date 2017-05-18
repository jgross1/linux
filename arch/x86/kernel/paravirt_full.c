/*
    Paravirtualization interfaces for fully paravirtualized guests
    Copyright (C) 2017 Juergen Gross SUSE Linux GmbH

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include <linux/percpu.h>
#include <linux/kprobes.h>

#include <asm/paravirt.h>
#include <asm/debugreg.h>
#include <asm/desc.h>
#include <asm/pgalloc.h>
#include <asm/processor.h>
#include <asm/tlbflush.h>

/* These are in entry.S */
extern void native_iret(void);
extern void native_usergs_sysret64(void);

static DEFINE_PER_CPU(enum paravirt_lazy_mode, paravirt_lazy_mode) =
	PARAVIRT_LAZY_NONE;

static inline void enter_lazy(enum paravirt_lazy_mode mode)
{
	BUG_ON(this_cpu_read(paravirt_lazy_mode) != PARAVIRT_LAZY_NONE);

	this_cpu_write(paravirt_lazy_mode, mode);
}

static void leave_lazy(enum paravirt_lazy_mode mode)
{
	BUG_ON(this_cpu_read(paravirt_lazy_mode) != mode);

	this_cpu_write(paravirt_lazy_mode, PARAVIRT_LAZY_NONE);
}

void paravirt_enter_lazy_mmu(void)
{
	enter_lazy(PARAVIRT_LAZY_MMU);
}

void paravirt_leave_lazy_mmu(void)
{
	leave_lazy(PARAVIRT_LAZY_MMU);
}

void paravirt_flush_lazy_mmu(void)
{
	preempt_disable();

	if (paravirt_get_lazy_mode() == PARAVIRT_LAZY_MMU) {
		arch_leave_lazy_mmu_mode();
		arch_enter_lazy_mmu_mode();
	}

	preempt_enable();
}

void paravirt_start_context_switch(struct task_struct *prev)
{
	BUG_ON(preemptible());

	if (this_cpu_read(paravirt_lazy_mode) == PARAVIRT_LAZY_MMU) {
		arch_leave_lazy_mmu_mode();
		set_ti_thread_flag(task_thread_info(prev),
				   TIF_LAZY_MMU_UPDATES);
	}
	enter_lazy(PARAVIRT_LAZY_CPU);
}

void paravirt_end_context_switch(struct task_struct *next)
{
	BUG_ON(preemptible());

	leave_lazy(PARAVIRT_LAZY_CPU);

	if (test_and_clear_ti_thread_flag(task_thread_info(next),
					  TIF_LAZY_MMU_UPDATES))
		arch_enter_lazy_mmu_mode();
}

enum paravirt_lazy_mode paravirt_get_lazy_mode(void)
{
	if (in_interrupt())
		return PARAVIRT_LAZY_NONE;

	return this_cpu_read(paravirt_lazy_mode);
}

static void native_flush_tlb(void)
{
	__native_flush_tlb();
}

/*
 * Global pages have to be flushed a bit differently. Not a real
 * performance problem because this does not happen often.
 */
static void native_flush_tlb_global(void)
{
	__native_flush_tlb_global();
}

static void native_flush_tlb_single(unsigned long addr)
{
	__native_flush_tlb_single(addr);
}

__visible struct pvfull_cpu_ops pvfull_cpu_ops = {
	.cpuid = native_cpuid,
	.get_debugreg = native_get_debugreg,
	.set_debugreg = native_set_debugreg,
	.read_cr0 = native_read_cr0,
	.write_cr0 = native_write_cr0,
	.read_cr4 = native_read_cr4,
	.write_cr4 = native_write_cr4,
#ifdef CONFIG_X86_64
	.read_cr8 = native_read_cr8,
	.write_cr8 = native_write_cr8,
#endif
	.wbinvd = native_wbinvd,
	.read_msr = native_read_msr,
	.write_msr = native_write_msr,
	.read_msr_safe = native_read_msr_safe,
	.write_msr_safe = native_write_msr_safe,
	.read_pmc = native_read_pmc,
	.load_tr_desc = native_load_tr_desc,
	.set_ldt = native_set_ldt,
	.load_gdt = native_load_gdt,
	.load_idt = native_load_idt,
	.store_idt = native_store_idt,
	.store_tr = native_store_tr,
	.load_tls = native_load_tls,
#ifdef CONFIG_X86_64
	.load_gs_index = native_load_gs_index,
#endif
	.write_ldt_entry = native_write_ldt_entry,
	.write_gdt_entry = native_write_gdt_entry,
	.write_idt_entry = native_write_idt_entry,

	.alloc_ldt = paravirt_nop,
	.free_ldt = paravirt_nop,

	.load_sp0 = native_load_sp0,

#ifdef CONFIG_X86_64
	.usergs_sysret64 = native_usergs_sysret64,
#endif
	.iret = native_iret,
	.swapgs = native_swapgs,

	.set_iopl_mask = native_set_iopl_mask,

	.start_context_switch = paravirt_nop,
	.end_context_switch = paravirt_nop,
};

__visible struct pvfull_irq_ops pvfull_irq_ops = {
	.safe_halt = native_safe_halt,
	.halt = native_halt,
#ifdef CONFIG_X86_64
	.adjust_exception_frame = paravirt_nop,
#endif
};

#if defined(CONFIG_X86_32) && !defined(CONFIG_X86_PAE)
/* 32-bit pagetable entries */
#define PTE_IDENT       __PV_IS_CALLEE_SAVE(_paravirt_ident_32)
#else
/* 64-bit pagetable entries */
#define PTE_IDENT       __PV_IS_CALLEE_SAVE(_paravirt_ident_64)
#endif

struct pvfull_mmu_ops pvfull_mmu_ops = {
	.read_cr2 = native_read_cr2,
	.write_cr2 = native_write_cr2,
	.read_cr3 = native_read_cr3,
	.write_cr3 = native_write_cr3,

	.flush_tlb_user = native_flush_tlb,
	.flush_tlb_kernel = native_flush_tlb_global,
	.flush_tlb_single = native_flush_tlb_single,

	.pgd_alloc = __paravirt_pgd_alloc,
	.pgd_free = paravirt_nop,

	.alloc_pte = paravirt_nop,
	.alloc_pmd = paravirt_nop,
	.alloc_pud = paravirt_nop,
	.alloc_p4d = paravirt_nop,
	.release_pte = paravirt_nop,
	.release_pmd = paravirt_nop,
	.release_pud = paravirt_nop,
	.release_p4d = paravirt_nop,

	.set_pte = native_set_pte,
	.set_pte_at = native_set_pte_at,
	.set_pmd = native_set_pmd,
	.set_pmd_at = native_set_pmd_at,
	.pte_update = paravirt_nop,

	.ptep_modify_prot_start = __ptep_modify_prot_start,
	.ptep_modify_prot_commit = __ptep_modify_prot_commit,

#if CONFIG_PGTABLE_LEVELS >= 3
#ifdef CONFIG_X86_PAE
	.set_pte_atomic = native_set_pte_atomic,
	.pte_clear = native_pte_clear,
	.pmd_clear = native_pmd_clear,
#endif
	.set_pud = native_set_pud,
	.set_pud_at = native_set_pud_at,

	.pmd_val = PTE_IDENT,
	.make_pmd = PTE_IDENT,

#if CONFIG_PGTABLE_LEVELS >= 4
	.pud_val = PTE_IDENT,
	.make_pud = PTE_IDENT,

	.set_p4d = native_set_p4d,

#if CONFIG_PGTABLE_LEVELS >= 5
	.p4d_val = PTE_IDENT,
	.make_p4d = PTE_IDENT,

	.set_pgd = native_set_pgd,
#endif /* CONFIG_PGTABLE_LEVELS >= 5 */
#endif /* CONFIG_PGTABLE_LEVELS >= 4 */
#endif /* CONFIG_PGTABLE_LEVELS >= 3 */

	.pte_val = PTE_IDENT,
	.pgd_val = PTE_IDENT,

	.make_pte = PTE_IDENT,
	.make_pgd = PTE_IDENT,

	.dup_mmap = paravirt_nop,
	.activate_mm = paravirt_nop,

	.lazy_mode = {
		.enter = paravirt_nop,
		.leave = paravirt_nop,
		.flush = paravirt_nop,
	},

	.set_fixmap = native_set_fixmap,
};

/* At this point, native_get/set_debugreg has real function entries */
NOKPROBE_SYMBOL(native_get_debugreg);
NOKPROBE_SYMBOL(native_set_debugreg);
NOKPROBE_SYMBOL(native_load_idt);

EXPORT_SYMBOL(pvfull_cpu_ops);
EXPORT_SYMBOL_GPL(pvfull_irq_ops);
EXPORT_SYMBOL(pvfull_mmu_ops);
