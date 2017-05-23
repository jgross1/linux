#ifndef _ASM_X86_PARAVIRT_FULL_H
#define _ASM_X86_PARAVIRT_FULL_H

#ifndef __ASSEMBLY__

static inline void load_sp0(struct tss_struct *tss,
			    struct thread_struct *thread)
{
	PVOP_VCALL2(pvfull_cpu_ops.load_sp0, tss, thread);
}

/* The paravirtualized CPUID instruction. */
static inline void __cpuid(unsigned int *eax, unsigned int *ebx,
			   unsigned int *ecx, unsigned int *edx)
{
	PVOP_VCALL4(pvfull_cpu_ops.cpuid, eax, ebx, ecx, edx);
}

/*
 * These special macros can be used to get or set a debugging register
 */
static inline unsigned long paravirt_get_debugreg(int reg)
{
	return PVOP_CALL1(unsigned long, pvfull_cpu_ops.get_debugreg, reg);
}
#define get_debugreg(var, reg) var = paravirt_get_debugreg(reg)
static inline void set_debugreg(unsigned long val, int reg)
{
	PVOP_VCALL2(pvfull_cpu_ops.set_debugreg, reg, val);
}

static inline unsigned long read_cr0(void)
{
	return PVOP_CALL0(unsigned long, pvfull_cpu_ops.read_cr0);
}

static inline void write_cr0(unsigned long x)
{
	PVOP_VCALL1(pvfull_cpu_ops.write_cr0, x);
}

static inline unsigned long __read_cr4(void)
{
	return PVOP_CALL0(unsigned long, pvfull_cpu_ops.read_cr4);
}

static inline void __write_cr4(unsigned long x)
{
	PVOP_VCALL1(pvfull_cpu_ops.write_cr4, x);
}

#ifdef CONFIG_X86_64
static inline unsigned long read_cr8(void)
{
	return PVOP_CALL0(unsigned long, pvfull_cpu_ops.read_cr8);
}

static inline void write_cr8(unsigned long x)
{
	PVOP_VCALL1(pvfull_cpu_ops.write_cr8, x);
}
#endif

static inline void wbinvd(void)
{
	PVOP_VCALL0(pvfull_cpu_ops.wbinvd);
}

static inline u64 paravirt_read_msr(unsigned msr)
{
	return PVOP_CALL1(u64, pvfull_cpu_ops.read_msr, msr);
}

static inline void paravirt_write_msr(unsigned msr,
				      unsigned low, unsigned high)
{
	return PVOP_VCALL3(pvfull_cpu_ops.write_msr, msr, low, high);
}

static inline u64 paravirt_read_msr_safe(unsigned msr, int *err)
{
	return PVOP_CALL2(u64, pvfull_cpu_ops.read_msr_safe, msr, err);
}

static inline int paravirt_write_msr_safe(unsigned msr,
					  unsigned low, unsigned high)
{
	return PVOP_CALL3(int, pvfull_cpu_ops.write_msr_safe, msr, low, high);
}

#define rdmsr(msr, val1, val2)			\
do {						\
	u64 _l = paravirt_read_msr(msr);	\
	val1 = (u32)_l;				\
	val2 = _l >> 32;			\
} while (0)

#define wrmsr(msr, val1, val2)			\
do {						\
	paravirt_write_msr(msr, val1, val2);	\
} while (0)

#define rdmsrl(msr, val)			\
do {						\
	val = paravirt_read_msr(msr);		\
} while (0)

static inline void wrmsrl(unsigned msr, u64 val)
{
	wrmsr(msr, (u32)val, (u32)(val>>32));
}

#define wrmsr_safe(msr, a, b)	paravirt_write_msr_safe(msr, a, b)

/* rdmsr with exception handling */
#define rdmsr_safe(msr, a, b)				\
({							\
	int _err;					\
	u64 _l = paravirt_read_msr_safe(msr, &_err);	\
	(*a) = (u32)_l;					\
	(*b) = _l >> 32;				\
	_err;						\
})

static inline int rdmsrl_safe(unsigned msr, unsigned long long *p)
{
	int err;

	*p = paravirt_read_msr_safe(msr, &err);
	return err;
}

static inline unsigned long long paravirt_read_pmc(int counter)
{
	return PVOP_CALL1(u64, pvfull_cpu_ops.read_pmc, counter);
}

#define rdpmc(counter, low, high)		\
do {						\
	u64 _l = paravirt_read_pmc(counter);	\
	low = (u32)_l;				\
	high = _l >> 32;			\
} while (0)

#define rdpmcl(counter, val) ((val) = paravirt_read_pmc(counter))

static inline void paravirt_alloc_ldt(struct desc_struct *ldt, unsigned entries)
{
	PVOP_VCALL2(pvfull_cpu_ops.alloc_ldt, ldt, entries);
}

static inline void paravirt_free_ldt(struct desc_struct *ldt, unsigned entries)
{
	PVOP_VCALL2(pvfull_cpu_ops.free_ldt, ldt, entries);
}

static inline void load_TR_desc(void)
{
	PVOP_VCALL0(pvfull_cpu_ops.load_tr_desc);
}

static inline void load_gdt(const struct desc_ptr *dtr)
{
	PVOP_VCALL1(pvfull_cpu_ops.load_gdt, dtr);
}

static inline void load_idt(const struct desc_ptr *dtr)
{
	PVOP_VCALL1(pvfull_cpu_ops.load_idt, dtr);
}

static inline void set_ldt(const void *addr, unsigned entries)
{
	PVOP_VCALL2(pvfull_cpu_ops.set_ldt, addr, entries);
}

static inline void store_idt(struct desc_ptr *dtr)
{
	PVOP_VCALL1(pvfull_cpu_ops.store_idt, dtr);
}

static inline unsigned long paravirt_store_tr(void)
{
	return PVOP_CALL0(unsigned long, pvfull_cpu_ops.store_tr);
}

#define store_tr(tr)	((tr) = paravirt_store_tr())

static inline void load_TLS(struct thread_struct *t, unsigned cpu)
{
	PVOP_VCALL2(pvfull_cpu_ops.load_tls, t, cpu);
}

#ifdef CONFIG_X86_64
static inline void load_gs_index(unsigned int gs)
{
	PVOP_VCALL1(pvfull_cpu_ops.load_gs_index, gs);
}
#endif

static inline void write_ldt_entry(struct desc_struct *dt, int entry,
				   const void *desc)
{
	PVOP_VCALL3(pvfull_cpu_ops.write_ldt_entry, dt, entry, desc);
}

static inline void write_gdt_entry(struct desc_struct *dt, int entry,
				   void *desc, int type)
{
	PVOP_VCALL4(pvfull_cpu_ops.write_gdt_entry, dt, entry, desc, type);
}

static inline void write_idt_entry(gate_desc *dt, int entry, const gate_desc *g)
{
	PVOP_VCALL3(pvfull_cpu_ops.write_idt_entry, dt, entry, g);
}

static inline void set_iopl_mask(unsigned mask)
{
	PVOP_VCALL1(pvfull_cpu_ops.set_iopl_mask, mask);
}

#define  __HAVE_ARCH_START_CONTEXT_SWITCH
static inline void arch_start_context_switch(struct task_struct *prev)
{
	PVOP_VCALL1(pvfull_cpu_ops.start_context_switch, prev);
}

static inline void arch_end_context_switch(struct task_struct *next)
{
	PVOP_VCALL1(pvfull_cpu_ops.end_context_switch, next);
}

#else /* __ASSEMBLY__ */

#define INTERRUPT_RETURN						\
	PARA_SITE(PARA_PATCH(pvfull_cpu_ops, PV_CPU_iret), CLBR_NONE,	\
		  jmp PARA_INDIRECT(pvfull_cpu_ops+PV_CPU_iret))

#ifdef CONFIG_X86_32
#define GET_CR0_INTO_EAX						\
	push %ecx; push %edx;						\
	call PARA_INDIRECT(pvfull_cpu_ops+PV_CPU_read_cr0);		\
	pop %edx; pop %ecx
#else   /* !CONFIG_X86_32 */

/*
 * If swapgs is used while the userspace stack is still current,
 * there's no way to call a pvop.  The PV replacement *must* be
 * inlined, or the swapgs instruction must be trapped and emulated.
 */
#define SWAPGS_UNSAFE_STACK						\
	PARA_SITE(PARA_PATCH(pvfull_cpu_ops, PV_CPU_swapgs), CLBR_NONE,	\
		  swapgs)

/*
 * Note: swapgs is very special, and in practise is either going to be
 * implemented with a single "swapgs" instruction or something very
 * special.  Either way, we don't need to save any registers for
 * it.
 */
#define SWAPGS								\
	PARA_SITE(PARA_PATCH(pvfull_cpu_ops, PV_CPU_swapgs), CLBR_NONE,	\
		  call PARA_INDIRECT(pvfull_cpu_ops+PV_CPU_swapgs))

#define USERGS_SYSRET64							\
	PARA_SITE(PARA_PATCH(pvfull_cpu_ops, PV_CPU_usergs_sysret64),	\
		  CLBR_NONE,						\
		  jmp PARA_INDIRECT(pvfull_cpu_ops+PV_CPU_usergs_sysret64))
#endif  /* CONFIG_X86_32 */

#endif /* __ASSEMBLY__ */
#endif /* _ASM_X86_PARAVIRT_FULL_H */
