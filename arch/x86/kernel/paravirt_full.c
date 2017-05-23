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
#include <asm/processor.h>

/* These are in entry.S */
extern void native_iret(void);
extern void native_usergs_sysret64(void);

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

/* At this point, native_get/set_debugreg has real function entries */
NOKPROBE_SYMBOL(native_get_debugreg);
NOKPROBE_SYMBOL(native_set_debugreg);
NOKPROBE_SYMBOL(native_load_idt);

EXPORT_SYMBOL(pvfull_cpu_ops);
EXPORT_SYMBOL_GPL(pvfull_irq_ops);
