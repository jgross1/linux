/*  Paravirtualization interfaces
    Copyright (C) 2006 Rusty Russell IBM Corporation

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    2007 - x86_64 support added by Glauber de Oliveira Costa, Red Hat Inc
*/

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/efi.h>
#include <linux/bcd.h>
#include <linux/highmem.h>

#include <asm/bug.h>
#include <asm/paravirt.h>
#include <asm/setup.h>
#include <asm/time.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/apic.h>
#include <asm/tlbflush.h>
#include <asm/timer.h>
#include <asm/special_insns.h>

/*
 * nop stub, which must not clobber anything *including the stack* to
 * avoid confusing the entry prologues.
 */
extern void _paravirt_nop(void);
asm (".pushsection .entry.text, \"ax\"\n"
     ".global _paravirt_nop\n"
     "_paravirt_nop:\n\t"
     "ret\n\t"
     ".size _paravirt_nop, . - _paravirt_nop\n\t"
     ".type _paravirt_nop, @function\n\t"
     ".popsection");

/* identity function, which can be inlined */
u32 notrace _paravirt_ident_32(u32 x)
{
	return x;
}

u64 notrace _paravirt_ident_64(u64 x)
{
	return x;
}

void __init default_banner(void)
{
	printk(KERN_INFO "Booting paravirtualized kernel on %s\n",
	       pv_info.name);
}

/* Undefined instruction for dealing with missing ops pointers. */
static const unsigned char ud2a[] = { 0x0f, 0x0b };

struct branch {
	unsigned char opcode;
	u32 delta;
} __attribute__((packed));

unsigned paravirt_patch_call(void *insnbuf,
			     const void *target, u16 tgt_clobbers,
			     unsigned long addr, u16 site_clobbers,
			     unsigned len)
{
	struct branch *b = insnbuf;
	unsigned long delta = (unsigned long)target - (addr+5);

	if (tgt_clobbers & ~site_clobbers)
		return len;	/* target would clobber too much for this site */
	if (len < 5)
		return len;	/* call too long for patch site */

	b->opcode = 0xe8; /* call */
	b->delta = delta;
	BUILD_BUG_ON(sizeof(*b) != 5);

	return 5;
}

unsigned paravirt_patch_jmp(void *insnbuf, const void *target,
			    unsigned long addr, unsigned len)
{
	struct branch *b = insnbuf;
	unsigned long delta = (unsigned long)target - (addr+5);

	if (len < 5)
		return len;	/* call too long for patch site */

	b->opcode = 0xe9;	/* jmp */
	b->delta = delta;

	return 5;
}

/* Neat trick to map patch type back to the call within the
 * corresponding structure. */
static void *get_call_destination(u8 type)
{
	struct paravirt_patch_template tmpl = {
		.pv_init_ops = pv_init_ops,
		.pv_time_ops = pv_time_ops,
		.pv_cpu_ops = pv_cpu_ops,
		.pv_irq_ops = pv_irq_ops,
		.pv_mmu_ops = pv_mmu_ops,
#ifdef CONFIG_PARAVIRT_SPINLOCKS
		.pv_lock_ops = pv_lock_ops,
#endif
#ifdef CONFIG_PARAVIRT_FULL
		.pvfull_cpu_ops = pvfull_cpu_ops,
		.pvfull_irq_ops = pvfull_irq_ops,
		.pvfull_mmu_ops = pvfull_mmu_ops,
#endif
	};
	return *((void **)&tmpl + type);
}

unsigned paravirt_patch_default(u8 type, u16 clobbers, void *insnbuf,
				unsigned long addr, unsigned len)
{
	void *opfunc = get_call_destination(type);
	unsigned ret;

	if (opfunc == NULL)
		/* If there's no function, patch it with a ud2a (BUG) */
		ret = paravirt_patch_insns(insnbuf, len, ud2a, ud2a+sizeof(ud2a));
	else if (opfunc == _paravirt_nop)
		ret = 0;

	/* identity functions just return their single argument */
	else if (opfunc == _paravirt_ident_32)
		ret = paravirt_patch_ident_32(insnbuf, len);
	else if (opfunc == _paravirt_ident_64)
		ret = paravirt_patch_ident_64(insnbuf, len);

#ifdef CONFIG_PARAVIRT_FULL
	else if (type == PARAVIRT_PATCH(pvfull_cpu_ops.iret) ||
		 type == PARAVIRT_PATCH(pvfull_cpu_ops.usergs_sysret64))
		/* If operation requires a jmp, then jmp */
		ret = paravirt_patch_jmp(insnbuf, opfunc, addr, len);
#endif
	else
		/* Otherwise call the function; assume target could
		   clobber any caller-save reg */
		ret = paravirt_patch_call(insnbuf, opfunc, CLBR_ANY,
					  addr, clobbers, len);

	return ret;
}

unsigned paravirt_patch_insns(void *insnbuf, unsigned len,
			      const char *start, const char *end)
{
	unsigned insn_len = end - start;

	if (insn_len > len || start == NULL)
		insn_len = len;
	else
		memcpy(insnbuf, start, insn_len);

	return insn_len;
}

struct static_key paravirt_steal_enabled;
struct static_key paravirt_steal_rq_enabled;

static u64 native_steal_clock(int cpu)
{
	return 0;
}

struct pv_info pv_info = {
	.name = "bare hardware",
};

struct pv_init_ops pv_init_ops = {
	.patch = native_patch,
};

struct pv_time_ops pv_time_ops = {
	.sched_clock = native_sched_clock,
	.steal_clock = native_steal_clock,
};

__visible struct pv_irq_ops pv_irq_ops = {
	.save_fl = __PV_IS_CALLEE_SAVE(native_save_fl),
	.restore_fl = __PV_IS_CALLEE_SAVE(native_restore_fl),
	.irq_disable = __PV_IS_CALLEE_SAVE(native_irq_disable),
	.irq_enable = __PV_IS_CALLEE_SAVE(native_irq_enable),
};

__visible struct pv_cpu_ops pv_cpu_ops = {
	.io_delay = native_io_delay,
};

struct pv_mmu_ops pv_mmu_ops __ro_after_init = {
	.flush_tlb_others = native_flush_tlb_others,
	.exit_mmap = paravirt_nop,
};

EXPORT_SYMBOL_GPL(pv_time_ops);
EXPORT_SYMBOL    (pv_cpu_ops);
EXPORT_SYMBOL    (pv_mmu_ops);
EXPORT_SYMBOL    (pv_info);
EXPORT_SYMBOL    (pv_irq_ops);
