#ifndef _ASM_X86_PARAVIRT_FULL_H
#define _ASM_X86_PARAVIRT_FULL_H

#ifndef __ASSEMBLY__

#define get_kernel_rpl()	(pvfull_info.kernel_rpl)

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

static inline void arch_safe_halt(void)
{
	PVOP_VCALL0(pvfull_irq_ops.safe_halt);
}

static inline void halt(void)
{
	PVOP_VCALL0(pvfull_irq_ops.halt);
}

static inline unsigned long read_cr2(void)
{
	return PVOP_CALL0(unsigned long, pvfull_mmu_ops.read_cr2);
}

static inline void write_cr2(unsigned long x)
{
	PVOP_VCALL1(pvfull_mmu_ops.write_cr2, x);
}

static inline unsigned long read_cr3(void)
{
	return PVOP_CALL0(unsigned long, pvfull_mmu_ops.read_cr3);
}

static inline void write_cr3(unsigned long x)
{
	PVOP_VCALL1(pvfull_mmu_ops.write_cr3, x);
}

static inline void paravirt_activate_mm(struct mm_struct *prev,
					struct mm_struct *next)
{
	PVOP_VCALL2(pvfull_mmu_ops.activate_mm, prev, next);
}

static inline void paravirt_arch_dup_mmap(struct mm_struct *oldmm,
					  struct mm_struct *mm)
{
	PVOP_VCALL2(pvfull_mmu_ops.dup_mmap, oldmm, mm);
}

static inline void __flush_tlb(void)
{
	PVOP_VCALL0(pvfull_mmu_ops.flush_tlb_user);
}

static inline void __flush_tlb_global(void)
{
	PVOP_VCALL0(pvfull_mmu_ops.flush_tlb_kernel);
}

static inline void __flush_tlb_single(unsigned long addr)
{
	PVOP_VCALL1(pvfull_mmu_ops.flush_tlb_single, addr);
}

static inline int paravirt_pgd_alloc(struct mm_struct *mm)
{
	return PVOP_CALL1(int, pvfull_mmu_ops.pgd_alloc, mm);
}

static inline void paravirt_pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
	PVOP_VCALL2(pvfull_mmu_ops.pgd_free, mm, pgd);
}

static inline void paravirt_alloc_pte(struct mm_struct *mm, unsigned long pfn)
{
	PVOP_VCALL2(pvfull_mmu_ops.alloc_pte, mm, pfn);
}

static inline void paravirt_release_pte(unsigned long pfn)
{
	PVOP_VCALL1(pvfull_mmu_ops.release_pte, pfn);
}

static inline void paravirt_alloc_pmd(struct mm_struct *mm, unsigned long pfn)
{
	PVOP_VCALL2(pvfull_mmu_ops.alloc_pmd, mm, pfn);
}

static inline void paravirt_release_pmd(unsigned long pfn)
{
	PVOP_VCALL1(pvfull_mmu_ops.release_pmd, pfn);
}

static inline void paravirt_alloc_pud(struct mm_struct *mm, unsigned long pfn)
{
	PVOP_VCALL2(pvfull_mmu_ops.alloc_pud, mm, pfn);
}

static inline void paravirt_release_pud(unsigned long pfn)
{
	PVOP_VCALL1(pvfull_mmu_ops.release_pud, pfn);
}

static inline void paravirt_alloc_p4d(struct mm_struct *mm, unsigned long pfn)
{
	PVOP_VCALL2(pvfull_mmu_ops.alloc_p4d, mm, pfn);
}

static inline void paravirt_release_p4d(unsigned long pfn)
{
	PVOP_VCALL1(pvfull_mmu_ops.release_p4d, pfn);
}

static inline void pte_update(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep)
{
	PVOP_VCALL3(pvfull_mmu_ops.pte_update, mm, addr, ptep);
}

static inline pte_t __pte(pteval_t val)
{
	pteval_t ret;

	if (sizeof(pteval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pteval_t,
				   pvfull_mmu_ops.make_pte,
				   val, (u64)val >> 32);
	else
		ret = PVOP_CALLEE1(pteval_t,
				   pvfull_mmu_ops.make_pte,
				   val);

	return (pte_t) { .pte = ret };
}

static inline pteval_t pte_val(pte_t pte)
{
	pteval_t ret;

	if (sizeof(pteval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pteval_t, pvfull_mmu_ops.pte_val,
				   pte.pte, (u64)pte.pte >> 32);
	else
		ret = PVOP_CALLEE1(pteval_t, pvfull_mmu_ops.pte_val,
				   pte.pte);

	return ret;
}

static inline pgd_t __pgd(pgdval_t val)
{
	pgdval_t ret;

	if (sizeof(pgdval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pgdval_t, pvfull_mmu_ops.make_pgd,
				   val, (u64)val >> 32);
	else
		ret = PVOP_CALLEE1(pgdval_t, pvfull_mmu_ops.make_pgd,
				   val);

	return (pgd_t) { ret };
}

static inline pgdval_t pgd_val(pgd_t pgd)
{
	pgdval_t ret;

	if (sizeof(pgdval_t) > sizeof(long))
		ret =  PVOP_CALLEE2(pgdval_t, pvfull_mmu_ops.pgd_val,
				    pgd.pgd, (u64)pgd.pgd >> 32);
	else
		ret =  PVOP_CALLEE1(pgdval_t, pvfull_mmu_ops.pgd_val,
				    pgd.pgd);

	return ret;
}

#define  __HAVE_ARCH_PTEP_MODIFY_PROT_TRANSACTION
static inline pte_t ptep_modify_prot_start(struct mm_struct *mm,
					   unsigned long addr, pte_t *ptep)
{
	pteval_t ret;

	ret = PVOP_CALL3(pteval_t, pvfull_mmu_ops.ptep_modify_prot_start,
			 mm, addr, ptep);

	return (pte_t) { .pte = ret };
}

static inline void ptep_modify_prot_commit(struct mm_struct *mm,
					   unsigned long addr, pte_t *ptep,
					   pte_t pte)
{
	if (sizeof(pteval_t) > sizeof(long))
		/* 5 arg words */
		pvfull_mmu_ops.ptep_modify_prot_commit(mm, addr, ptep, pte);
	else
		PVOP_VCALL4(pvfull_mmu_ops.ptep_modify_prot_commit,
			    mm, addr, ptep, pte.pte);
}

static inline void set_pte(pte_t *ptep, pte_t pte)
{
	if (sizeof(pteval_t) > sizeof(long))
		PVOP_VCALL3(pvfull_mmu_ops.set_pte, ptep,
			    pte.pte, (u64)pte.pte >> 32);
	else
		PVOP_VCALL2(pvfull_mmu_ops.set_pte, ptep,
			    pte.pte);
}

static inline void set_pte_at(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep, pte_t pte)
{
	if (sizeof(pteval_t) > sizeof(long))
		/* 5 arg words */
		pvfull_mmu_ops.set_pte_at(mm, addr, ptep, pte);
	else
		PVOP_VCALL4(pvfull_mmu_ops.set_pte_at, mm, addr, ptep, pte.pte);
}

static inline void set_pmd_at(struct mm_struct *mm, unsigned long addr,
			      pmd_t *pmdp, pmd_t pmd)
{
	if (sizeof(pmdval_t) > sizeof(long))
		/* 5 arg words */
		pvfull_mmu_ops.set_pmd_at(mm, addr, pmdp, pmd);
	else
		PVOP_VCALL4(pvfull_mmu_ops.set_pmd_at, mm, addr, pmdp,
			    native_pmd_val(pmd));
}

static inline void set_pud_at(struct mm_struct *mm, unsigned long addr,
			      pud_t *pudp, pud_t pud)
{
	if (sizeof(pudval_t) > sizeof(long))
		/* 5 arg words */
		pvfull_mmu_ops.set_pud_at(mm, addr, pudp, pud);
	else
		PVOP_VCALL4(pvfull_mmu_ops.set_pud_at, mm, addr, pudp,
			    native_pud_val(pud));
}

static inline void set_pmd(pmd_t *pmdp, pmd_t pmd)
{
	pmdval_t val = native_pmd_val(pmd);

	if (sizeof(pmdval_t) > sizeof(long))
		PVOP_VCALL3(pvfull_mmu_ops.set_pmd, pmdp, val, (u64)val >> 32);
	else
		PVOP_VCALL2(pvfull_mmu_ops.set_pmd, pmdp, val);
}

#if CONFIG_PGTABLE_LEVELS >= 3
static inline pmd_t __pmd(pmdval_t val)
{
	pmdval_t ret;

	if (sizeof(pmdval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pmdval_t, pvfull_mmu_ops.make_pmd,
				   val, (u64)val >> 32);
	else
		ret = PVOP_CALLEE1(pmdval_t, pvfull_mmu_ops.make_pmd,
				   val);

	return (pmd_t) { ret };
}

static inline pmdval_t pmd_val(pmd_t pmd)
{
	pmdval_t ret;

	if (sizeof(pmdval_t) > sizeof(long))
		ret =  PVOP_CALLEE2(pmdval_t, pvfull_mmu_ops.pmd_val,
				    pmd.pmd, (u64)pmd.pmd >> 32);
	else
		ret =  PVOP_CALLEE1(pmdval_t, pvfull_mmu_ops.pmd_val,
				    pmd.pmd);

	return ret;
}

static inline void set_pud(pud_t *pudp, pud_t pud)
{
	pudval_t val = native_pud_val(pud);

	if (sizeof(pudval_t) > sizeof(long))
		PVOP_VCALL3(pvfull_mmu_ops.set_pud, pudp,
			    val, (u64)val >> 32);
	else
		PVOP_VCALL2(pvfull_mmu_ops.set_pud, pudp,
			    val);
}

#if CONFIG_PGTABLE_LEVELS >= 4
static inline pud_t __pud(pudval_t val)
{
	pudval_t ret;

	if (sizeof(pudval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pudval_t, pvfull_mmu_ops.make_pud,
				   val, (u64)val >> 32);
	else
		ret = PVOP_CALLEE1(pudval_t, pvfull_mmu_ops.make_pud,
				   val);

	return (pud_t) { ret };
}

static inline pudval_t pud_val(pud_t pud)
{
	pudval_t ret;

	if (sizeof(pudval_t) > sizeof(long))
		ret =  PVOP_CALLEE2(pudval_t, pvfull_mmu_ops.pud_val,
				    pud.pud, (u64)pud.pud >> 32);
	else
		ret =  PVOP_CALLEE1(pudval_t, pvfull_mmu_ops.pud_val,
				    pud.pud);

	return ret;
}

static inline void pud_clear(pud_t *pudp)
{
	set_pud(pudp, __pud(0));
}

static inline void set_p4d(p4d_t *p4dp, p4d_t p4d)
{
	p4dval_t val = native_p4d_val(p4d);

	if (sizeof(p4dval_t) > sizeof(long))
		PVOP_VCALL3(pvfull_mmu_ops.set_p4d, p4dp,
			    val, (u64)val >> 32);
	else
		PVOP_VCALL2(pvfull_mmu_ops.set_p4d, p4dp,
			    val);
}

#if CONFIG_PGTABLE_LEVELS >= 5
static inline p4d_t __p4d(p4dval_t val)
{
	p4dval_t ret = PVOP_CALLEE1(p4dval_t, pvfull_mmu_ops.make_p4d, val);

	return (p4d_t) { ret };
}

static inline p4dval_t p4d_val(p4d_t p4d)
{
	return PVOP_CALLEE1(p4dval_t, pvfull_mmu_ops.p4d_val, p4d.p4d);
}

static inline void set_pgd(pgd_t *pgdp, pgd_t pgd)
{
	pgdval_t val = native_pgd_val(pgd);

	PVOP_VCALL2(pvfull_mmu_ops.set_pgd, pgdp, val);
}

static inline void pgd_clear(pgd_t *pgdp)
{
	set_pgd(pgdp, __pgd(0));
}

#endif  /* CONFIG_PGTABLE_LEVELS >= 5 */

static inline void p4d_clear(p4d_t *p4dp)
{
	set_p4d(p4dp, __p4d(0));
}

#endif  /* CONFIG_PGTABLE_LEVELS >= 4 */

#endif  /* CONFIG_PGTABLE_LEVELS >= 3 */

#ifdef CONFIG_X86_PAE
/* Special-case pte-setting operations for PAE, which can't update a
   64-bit pte atomically */
static inline void set_pte_atomic(pte_t *ptep, pte_t pte)
{
	PVOP_VCALL3(pvfull_mmu_ops.set_pte_atomic, ptep,
		    pte.pte, pte.pte >> 32);
}

static inline void pte_clear(struct mm_struct *mm, unsigned long addr,
			     pte_t *ptep)
{
	PVOP_VCALL3(pvfull_mmu_ops.pte_clear, mm, addr, ptep);
}

static inline void pmd_clear(pmd_t *pmdp)
{
	PVOP_VCALL1(pvfull_mmu_ops.pmd_clear, pmdp);
}
#else  /* !CONFIG_X86_PAE */
static inline void set_pte_atomic(pte_t *ptep, pte_t pte)
{
	set_pte(ptep, pte);
}

static inline void pte_clear(struct mm_struct *mm, unsigned long addr,
			     pte_t *ptep)
{
	set_pte_at(mm, addr, ptep, __pte(0));
}

static inline void pmd_clear(pmd_t *pmdp)
{
	set_pmd(pmdp, __pmd(0));
}
#endif  /* CONFIG_X86_PAE */

#define  __HAVE_ARCH_ENTER_LAZY_MMU_MODE
static inline void arch_enter_lazy_mmu_mode(void)
{
	PVOP_VCALL0(pvfull_mmu_ops.lazy_mode.enter);
}

static inline void arch_leave_lazy_mmu_mode(void)
{
	PVOP_VCALL0(pvfull_mmu_ops.lazy_mode.leave);
}

static inline void arch_flush_lazy_mmu_mode(void)
{
	PVOP_VCALL0(pvfull_mmu_ops.lazy_mode.flush);
}

static inline void __set_fixmap(unsigned /* enum fixed_addresses */ idx,
				phys_addr_t phys, pgprot_t flags)
{
	pvfull_mmu_ops.set_fixmap(idx, phys, flags);
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

#define PARAVIRT_ADJUST_EXCEPTION_FRAME					\
	PARA_SITE(PARA_PATCH(pvfull_irq_ops, PV_IRQ_adjust_exception_frame), \
		  CLBR_NONE,						\
		  call PARA_INDIRECT(pvfull_irq_ops +			\
				     PV_IRQ_adjust_exception_frame))

#define GET_CR2_INTO_RAX						\
	call PARA_INDIRECT(pvfull_mmu_ops+PV_MMU_read_cr2)

#endif  /* CONFIG_X86_32 */

#endif /* __ASSEMBLY__ */
#endif /* _ASM_X86_PARAVIRT_FULL_H */
