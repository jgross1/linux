#ifndef _ASM_X86_PARAVIRT_H
#define _ASM_X86_PARAVIRT_H
/* Various instructions on x86 need to be replaced for
 * para-virtualization: those hooks are defined here. */

#ifdef CONFIG_PARAVIRT
#include <asm/pgtable_types.h>
#include <asm/asm.h>

#include <asm/paravirt_types.h>

#ifndef __ASSEMBLY__
#include <linux/bug.h>
#include <linux/types.h>
#include <linux/cpumask.h>
#include <asm/frame.h>

#ifdef CONFIG_PARAVIRT_FULL
#include <asm/paravirt_full.h>
#endif

static inline unsigned long read_cr2(void)
{
	return PVOP_CALL0(unsigned long, pv_mmu_ops.read_cr2);
}

static inline void write_cr2(unsigned long x)
{
	PVOP_VCALL1(pv_mmu_ops.write_cr2, x);
}

static inline unsigned long read_cr3(void)
{
	return PVOP_CALL0(unsigned long, pv_mmu_ops.read_cr3);
}

static inline void write_cr3(unsigned long x)
{
	PVOP_VCALL1(pv_mmu_ops.write_cr3, x);
}

static inline void arch_safe_halt(void)
{
	PVOP_VCALL0(pv_irq_ops.safe_halt);
}

static inline void halt(void)
{
	PVOP_VCALL0(pv_irq_ops.halt);
}

#define get_kernel_rpl()  (pv_info.kernel_rpl)

static inline unsigned long long paravirt_sched_clock(void)
{
	return PVOP_CALL0(unsigned long long, pv_time_ops.sched_clock);
}

struct static_key;
extern struct static_key paravirt_steal_enabled;
extern struct static_key paravirt_steal_rq_enabled;

static inline u64 paravirt_steal_clock(int cpu)
{
	return PVOP_CALL1(u64, pv_time_ops.steal_clock, cpu);
}

/* The paravirtualized I/O functions */
static inline void slow_down_io(void)
{
	pv_cpu_ops.io_delay();
#ifdef REALLY_SLOW_IO
	pv_cpu_ops.io_delay();
	pv_cpu_ops.io_delay();
	pv_cpu_ops.io_delay();
#endif
}

static inline void paravirt_activate_mm(struct mm_struct *prev,
					struct mm_struct *next)
{
	PVOP_VCALL2(pv_mmu_ops.activate_mm, prev, next);
}

static inline void paravirt_arch_dup_mmap(struct mm_struct *oldmm,
					  struct mm_struct *mm)
{
	PVOP_VCALL2(pv_mmu_ops.dup_mmap, oldmm, mm);
}

static inline void paravirt_arch_exit_mmap(struct mm_struct *mm)
{
	PVOP_VCALL1(pv_mmu_ops.exit_mmap, mm);
}

static inline void __flush_tlb(void)
{
	PVOP_VCALL0(pv_mmu_ops.flush_tlb_user);
}
static inline void __flush_tlb_global(void)
{
	PVOP_VCALL0(pv_mmu_ops.flush_tlb_kernel);
}
static inline void __flush_tlb_single(unsigned long addr)
{
	PVOP_VCALL1(pv_mmu_ops.flush_tlb_single, addr);
}

static inline void flush_tlb_others(const struct cpumask *cpumask,
				    struct mm_struct *mm,
				    unsigned long start,
				    unsigned long end)
{
	PVOP_VCALL4(pv_mmu_ops.flush_tlb_others, cpumask, mm, start, end);
}

static inline int paravirt_pgd_alloc(struct mm_struct *mm)
{
	return PVOP_CALL1(int, pv_mmu_ops.pgd_alloc, mm);
}

static inline void paravirt_pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
	PVOP_VCALL2(pv_mmu_ops.pgd_free, mm, pgd);
}

static inline void paravirt_alloc_pte(struct mm_struct *mm, unsigned long pfn)
{
	PVOP_VCALL2(pv_mmu_ops.alloc_pte, mm, pfn);
}
static inline void paravirt_release_pte(unsigned long pfn)
{
	PVOP_VCALL1(pv_mmu_ops.release_pte, pfn);
}

static inline void paravirt_alloc_pmd(struct mm_struct *mm, unsigned long pfn)
{
	PVOP_VCALL2(pv_mmu_ops.alloc_pmd, mm, pfn);
}

static inline void paravirt_release_pmd(unsigned long pfn)
{
	PVOP_VCALL1(pv_mmu_ops.release_pmd, pfn);
}

static inline void paravirt_alloc_pud(struct mm_struct *mm, unsigned long pfn)
{
	PVOP_VCALL2(pv_mmu_ops.alloc_pud, mm, pfn);
}
static inline void paravirt_release_pud(unsigned long pfn)
{
	PVOP_VCALL1(pv_mmu_ops.release_pud, pfn);
}

static inline void paravirt_alloc_p4d(struct mm_struct *mm, unsigned long pfn)
{
	PVOP_VCALL2(pv_mmu_ops.alloc_p4d, mm, pfn);
}

static inline void paravirt_release_p4d(unsigned long pfn)
{
	PVOP_VCALL1(pv_mmu_ops.release_p4d, pfn);
}

static inline void pte_update(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep)
{
	PVOP_VCALL3(pv_mmu_ops.pte_update, mm, addr, ptep);
}

static inline pte_t __pte(pteval_t val)
{
	pteval_t ret;

	if (sizeof(pteval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pteval_t,
				   pv_mmu_ops.make_pte,
				   val, (u64)val >> 32);
	else
		ret = PVOP_CALLEE1(pteval_t,
				   pv_mmu_ops.make_pte,
				   val);

	return (pte_t) { .pte = ret };
}

static inline pteval_t pte_val(pte_t pte)
{
	pteval_t ret;

	if (sizeof(pteval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pteval_t, pv_mmu_ops.pte_val,
				   pte.pte, (u64)pte.pte >> 32);
	else
		ret = PVOP_CALLEE1(pteval_t, pv_mmu_ops.pte_val,
				   pte.pte);

	return ret;
}

static inline pgd_t __pgd(pgdval_t val)
{
	pgdval_t ret;

	if (sizeof(pgdval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pgdval_t, pv_mmu_ops.make_pgd,
				   val, (u64)val >> 32);
	else
		ret = PVOP_CALLEE1(pgdval_t, pv_mmu_ops.make_pgd,
				   val);

	return (pgd_t) { ret };
}

static inline pgdval_t pgd_val(pgd_t pgd)
{
	pgdval_t ret;

	if (sizeof(pgdval_t) > sizeof(long))
		ret =  PVOP_CALLEE2(pgdval_t, pv_mmu_ops.pgd_val,
				    pgd.pgd, (u64)pgd.pgd >> 32);
	else
		ret =  PVOP_CALLEE1(pgdval_t, pv_mmu_ops.pgd_val,
				    pgd.pgd);

	return ret;
}

#define  __HAVE_ARCH_PTEP_MODIFY_PROT_TRANSACTION
static inline pte_t ptep_modify_prot_start(struct mm_struct *mm, unsigned long addr,
					   pte_t *ptep)
{
	pteval_t ret;

	ret = PVOP_CALL3(pteval_t, pv_mmu_ops.ptep_modify_prot_start,
			 mm, addr, ptep);

	return (pte_t) { .pte = ret };
}

static inline void ptep_modify_prot_commit(struct mm_struct *mm, unsigned long addr,
					   pte_t *ptep, pte_t pte)
{
	if (sizeof(pteval_t) > sizeof(long))
		/* 5 arg words */
		pv_mmu_ops.ptep_modify_prot_commit(mm, addr, ptep, pte);
	else
		PVOP_VCALL4(pv_mmu_ops.ptep_modify_prot_commit,
			    mm, addr, ptep, pte.pte);
}

static inline void set_pte(pte_t *ptep, pte_t pte)
{
	if (sizeof(pteval_t) > sizeof(long))
		PVOP_VCALL3(pv_mmu_ops.set_pte, ptep,
			    pte.pte, (u64)pte.pte >> 32);
	else
		PVOP_VCALL2(pv_mmu_ops.set_pte, ptep,
			    pte.pte);
}

static inline void set_pte_at(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep, pte_t pte)
{
	if (sizeof(pteval_t) > sizeof(long))
		/* 5 arg words */
		pv_mmu_ops.set_pte_at(mm, addr, ptep, pte);
	else
		PVOP_VCALL4(pv_mmu_ops.set_pte_at, mm, addr, ptep, pte.pte);
}

static inline void set_pmd_at(struct mm_struct *mm, unsigned long addr,
			      pmd_t *pmdp, pmd_t pmd)
{
	if (sizeof(pmdval_t) > sizeof(long))
		/* 5 arg words */
		pv_mmu_ops.set_pmd_at(mm, addr, pmdp, pmd);
	else
		PVOP_VCALL4(pv_mmu_ops.set_pmd_at, mm, addr, pmdp,
			    native_pmd_val(pmd));
}

static inline void set_pud_at(struct mm_struct *mm, unsigned long addr,
			      pud_t *pudp, pud_t pud)
{
	if (sizeof(pudval_t) > sizeof(long))
		/* 5 arg words */
		pv_mmu_ops.set_pud_at(mm, addr, pudp, pud);
	else
		PVOP_VCALL4(pv_mmu_ops.set_pud_at, mm, addr, pudp,
			    native_pud_val(pud));
}

static inline void set_pmd(pmd_t *pmdp, pmd_t pmd)
{
	pmdval_t val = native_pmd_val(pmd);

	if (sizeof(pmdval_t) > sizeof(long))
		PVOP_VCALL3(pv_mmu_ops.set_pmd, pmdp, val, (u64)val >> 32);
	else
		PVOP_VCALL2(pv_mmu_ops.set_pmd, pmdp, val);
}

#if CONFIG_PGTABLE_LEVELS >= 3
static inline pmd_t __pmd(pmdval_t val)
{
	pmdval_t ret;

	if (sizeof(pmdval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pmdval_t, pv_mmu_ops.make_pmd,
				   val, (u64)val >> 32);
	else
		ret = PVOP_CALLEE1(pmdval_t, pv_mmu_ops.make_pmd,
				   val);

	return (pmd_t) { ret };
}

static inline pmdval_t pmd_val(pmd_t pmd)
{
	pmdval_t ret;

	if (sizeof(pmdval_t) > sizeof(long))
		ret =  PVOP_CALLEE2(pmdval_t, pv_mmu_ops.pmd_val,
				    pmd.pmd, (u64)pmd.pmd >> 32);
	else
		ret =  PVOP_CALLEE1(pmdval_t, pv_mmu_ops.pmd_val,
				    pmd.pmd);

	return ret;
}

static inline void set_pud(pud_t *pudp, pud_t pud)
{
	pudval_t val = native_pud_val(pud);

	if (sizeof(pudval_t) > sizeof(long))
		PVOP_VCALL3(pv_mmu_ops.set_pud, pudp,
			    val, (u64)val >> 32);
	else
		PVOP_VCALL2(pv_mmu_ops.set_pud, pudp,
			    val);
}
#if CONFIG_PGTABLE_LEVELS >= 4
static inline pud_t __pud(pudval_t val)
{
	pudval_t ret;

	if (sizeof(pudval_t) > sizeof(long))
		ret = PVOP_CALLEE2(pudval_t, pv_mmu_ops.make_pud,
				   val, (u64)val >> 32);
	else
		ret = PVOP_CALLEE1(pudval_t, pv_mmu_ops.make_pud,
				   val);

	return (pud_t) { ret };
}

static inline pudval_t pud_val(pud_t pud)
{
	pudval_t ret;

	if (sizeof(pudval_t) > sizeof(long))
		ret =  PVOP_CALLEE2(pudval_t, pv_mmu_ops.pud_val,
				    pud.pud, (u64)pud.pud >> 32);
	else
		ret =  PVOP_CALLEE1(pudval_t, pv_mmu_ops.pud_val,
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
		PVOP_VCALL3(pv_mmu_ops.set_p4d, p4dp,
			    val, (u64)val >> 32);
	else
		PVOP_VCALL2(pv_mmu_ops.set_p4d, p4dp,
			    val);
}

#if CONFIG_PGTABLE_LEVELS >= 5

static inline p4d_t __p4d(p4dval_t val)
{
	p4dval_t ret = PVOP_CALLEE1(p4dval_t, pv_mmu_ops.make_p4d, val);

	return (p4d_t) { ret };
}

static inline p4dval_t p4d_val(p4d_t p4d)
{
	return PVOP_CALLEE1(p4dval_t, pv_mmu_ops.p4d_val, p4d.p4d);
}

static inline void set_pgd(pgd_t *pgdp, pgd_t pgd)
{
	pgdval_t val = native_pgd_val(pgd);

	PVOP_VCALL2(pv_mmu_ops.set_pgd, pgdp, val);
}

static inline void pgd_clear(pgd_t *pgdp)
{
	set_pgd(pgdp, __pgd(0));
}

#endif  /* CONFIG_PGTABLE_LEVELS == 5 */

static inline void p4d_clear(p4d_t *p4dp)
{
	set_p4d(p4dp, __p4d(0));
}

#endif	/* CONFIG_PGTABLE_LEVELS == 4 */

#endif	/* CONFIG_PGTABLE_LEVELS >= 3 */

#ifdef CONFIG_X86_PAE
/* Special-case pte-setting operations for PAE, which can't update a
   64-bit pte atomically */
static inline void set_pte_atomic(pte_t *ptep, pte_t pte)
{
	PVOP_VCALL3(pv_mmu_ops.set_pte_atomic, ptep,
		    pte.pte, pte.pte >> 32);
}

static inline void pte_clear(struct mm_struct *mm, unsigned long addr,
			     pte_t *ptep)
{
	PVOP_VCALL3(pv_mmu_ops.pte_clear, mm, addr, ptep);
}

static inline void pmd_clear(pmd_t *pmdp)
{
	PVOP_VCALL1(pv_mmu_ops.pmd_clear, pmdp);
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
#endif	/* CONFIG_X86_PAE */

#define  __HAVE_ARCH_ENTER_LAZY_MMU_MODE
static inline void arch_enter_lazy_mmu_mode(void)
{
	PVOP_VCALL0(pv_mmu_ops.lazy_mode.enter);
}

static inline void arch_leave_lazy_mmu_mode(void)
{
	PVOP_VCALL0(pv_mmu_ops.lazy_mode.leave);
}

static inline void arch_flush_lazy_mmu_mode(void)
{
	PVOP_VCALL0(pv_mmu_ops.lazy_mode.flush);
}

static inline void __set_fixmap(unsigned /* enum fixed_addresses */ idx,
				phys_addr_t phys, pgprot_t flags)
{
	pv_mmu_ops.set_fixmap(idx, phys, flags);
}

#if defined(CONFIG_SMP) && defined(CONFIG_PARAVIRT_SPINLOCKS)

static __always_inline void pv_queued_spin_lock_slowpath(struct qspinlock *lock,
							u32 val)
{
	PVOP_VCALL2(pv_lock_ops.queued_spin_lock_slowpath, lock, val);
}

static __always_inline void pv_queued_spin_unlock(struct qspinlock *lock)
{
	PVOP_VCALLEE1(pv_lock_ops.queued_spin_unlock, lock);
}

static __always_inline void pv_wait(u8 *ptr, u8 val)
{
	PVOP_VCALL2(pv_lock_ops.wait, ptr, val);
}

static __always_inline void pv_kick(int cpu)
{
	PVOP_VCALL1(pv_lock_ops.kick, cpu);
}

static __always_inline bool pv_vcpu_is_preempted(long cpu)
{
	return PVOP_CALLEE1(bool, pv_lock_ops.vcpu_is_preempted, cpu);
}

#endif /* SMP && PARAVIRT_SPINLOCKS */

#ifdef CONFIG_X86_32
#define PV_SAVE_REGS "pushl %ecx; pushl %edx;"
#define PV_RESTORE_REGS "popl %edx; popl %ecx;"

/* save and restore all caller-save registers, except return value */
#define PV_SAVE_ALL_CALLER_REGS		"pushl %ecx;"
#define PV_RESTORE_ALL_CALLER_REGS	"popl  %ecx;"

#define PV_FLAGS_ARG "0"
#define PV_EXTRA_CLOBBERS
#define PV_VEXTRA_CLOBBERS
#else
/* save and restore all caller-save registers, except return value */
#define PV_SAVE_ALL_CALLER_REGS						\
	"push %rcx;"							\
	"push %rdx;"							\
	"push %rsi;"							\
	"push %rdi;"							\
	"push %r8;"							\
	"push %r9;"							\
	"push %r10;"							\
	"push %r11;"
#define PV_RESTORE_ALL_CALLER_REGS					\
	"pop %r11;"							\
	"pop %r10;"							\
	"pop %r9;"							\
	"pop %r8;"							\
	"pop %rdi;"							\
	"pop %rsi;"							\
	"pop %rdx;"							\
	"pop %rcx;"

/* We save some registers, but all of them, that's too much. We clobber all
 * caller saved registers but the argument parameter */
#define PV_SAVE_REGS "pushq %%rdi;"
#define PV_RESTORE_REGS "popq %%rdi;"
#define PV_EXTRA_CLOBBERS EXTRA_CLOBBERS, "rcx" , "rdx", "rsi"
#define PV_VEXTRA_CLOBBERS EXTRA_CLOBBERS, "rdi", "rcx" , "rdx", "rsi"
#define PV_FLAGS_ARG "D"
#endif

/*
 * Generate a thunk around a function which saves all caller-save
 * registers except for the return value.  This allows C functions to
 * be called from assembler code where fewer than normal registers are
 * available.  It may also help code generation around calls from C
 * code if the common case doesn't use many registers.
 *
 * When a callee is wrapped in a thunk, the caller can assume that all
 * arg regs and all scratch registers are preserved across the
 * call. The return value in rax/eax will not be saved, even for void
 * functions.
 */
#define PV_THUNK_NAME(func) "__raw_callee_save_" #func
#define PV_CALLEE_SAVE_REGS_THUNK(func)					\
	extern typeof(func) __raw_callee_save_##func;			\
									\
	asm(".pushsection .text;"					\
	    ".globl " PV_THUNK_NAME(func) ";"				\
	    ".type " PV_THUNK_NAME(func) ", @function;"			\
	    PV_THUNK_NAME(func) ":"					\
	    FRAME_BEGIN							\
	    PV_SAVE_ALL_CALLER_REGS					\
	    "call " #func ";"						\
	    PV_RESTORE_ALL_CALLER_REGS					\
	    FRAME_END							\
	    "ret;"							\
	    ".popsection")

/* Get a reference to a callee-save function */
#define PV_CALLEE_SAVE(func)						\
	((struct paravirt_callee_save) { __raw_callee_save_##func })

/* Promise that "func" already uses the right calling convention */
#define __PV_IS_CALLEE_SAVE(func)			\
	((struct paravirt_callee_save) { func })

static inline notrace unsigned long arch_local_save_flags(void)
{
	return PVOP_CALLEE0(unsigned long, pv_irq_ops.save_fl);
}

static inline notrace void arch_local_irq_restore(unsigned long f)
{
	PVOP_VCALLEE1(pv_irq_ops.restore_fl, f);
}

static inline notrace void arch_local_irq_disable(void)
{
	PVOP_VCALLEE0(pv_irq_ops.irq_disable);
}

static inline notrace void arch_local_irq_enable(void)
{
	PVOP_VCALLEE0(pv_irq_ops.irq_enable);
}

static inline notrace unsigned long arch_local_irq_save(void)
{
	unsigned long f;

	f = arch_local_save_flags();
	arch_local_irq_disable();
	return f;
}


/* Make sure as little as possible of this mess escapes. */
#undef PARAVIRT_CALL
#undef __PVOP_CALL
#undef __PVOP_VCALL
#undef PVOP_VCALL0
#undef PVOP_CALL0
#undef PVOP_VCALL1
#undef PVOP_CALL1
#undef PVOP_VCALL2
#undef PVOP_CALL2
#undef PVOP_VCALL3
#undef PVOP_CALL3
#undef PVOP_VCALL4
#undef PVOP_CALL4

extern void default_banner(void);

#else  /* __ASSEMBLY__ */

#define _PVSITE(ptype, clobbers, ops, word, algn)	\
771:;						\
	ops;					\
772:;						\
	.pushsection .parainstructions,"a";	\
	 .align	algn;				\
	 word 771b;				\
	 .byte ptype;				\
	 .byte 772b-771b;			\
	 .short clobbers;			\
	.popsection


#define COND_PUSH(set, mask, reg)			\
	.if ((~(set)) & mask); push %reg; .endif
#define COND_POP(set, mask, reg)			\
	.if ((~(set)) & mask); pop %reg; .endif

#ifdef CONFIG_X86_64

#define PV_SAVE_REGS(set)			\
	COND_PUSH(set, CLBR_RAX, rax);		\
	COND_PUSH(set, CLBR_RCX, rcx);		\
	COND_PUSH(set, CLBR_RDX, rdx);		\
	COND_PUSH(set, CLBR_RSI, rsi);		\
	COND_PUSH(set, CLBR_RDI, rdi);		\
	COND_PUSH(set, CLBR_R8, r8);		\
	COND_PUSH(set, CLBR_R9, r9);		\
	COND_PUSH(set, CLBR_R10, r10);		\
	COND_PUSH(set, CLBR_R11, r11)
#define PV_RESTORE_REGS(set)			\
	COND_POP(set, CLBR_R11, r11);		\
	COND_POP(set, CLBR_R10, r10);		\
	COND_POP(set, CLBR_R9, r9);		\
	COND_POP(set, CLBR_R8, r8);		\
	COND_POP(set, CLBR_RDI, rdi);		\
	COND_POP(set, CLBR_RSI, rsi);		\
	COND_POP(set, CLBR_RDX, rdx);		\
	COND_POP(set, CLBR_RCX, rcx);		\
	COND_POP(set, CLBR_RAX, rax)

#define PARA_PATCH(struct, off)        ((PARAVIRT_PATCH_##struct + (off)) / 8)
#define PARA_SITE(ptype, clobbers, ops) _PVSITE(ptype, clobbers, ops, .quad, 8)
#define PARA_INDIRECT(addr)	*addr(%rip)
#else
#define PV_SAVE_REGS(set)			\
	COND_PUSH(set, CLBR_EAX, eax);		\
	COND_PUSH(set, CLBR_EDI, edi);		\
	COND_PUSH(set, CLBR_ECX, ecx);		\
	COND_PUSH(set, CLBR_EDX, edx)
#define PV_RESTORE_REGS(set)			\
	COND_POP(set, CLBR_EDX, edx);		\
	COND_POP(set, CLBR_ECX, ecx);		\
	COND_POP(set, CLBR_EDI, edi);		\
	COND_POP(set, CLBR_EAX, eax)

#define PARA_PATCH(struct, off)        ((PARAVIRT_PATCH_##struct + (off)) / 4)
#define PARA_SITE(ptype, clobbers, ops) _PVSITE(ptype, clobbers, ops, .long, 4)
#define PARA_INDIRECT(addr)	*%cs:addr
#endif

#ifdef CONFIG_PARAVIRT_FULL
#include <asm/paravirt_full.h>
#endif

#define DISABLE_INTERRUPTS(clobbers)					\
	PARA_SITE(PARA_PATCH(pv_irq_ops, PV_IRQ_irq_disable), clobbers, \
		  PV_SAVE_REGS(clobbers | CLBR_CALLEE_SAVE);		\
		  call PARA_INDIRECT(pv_irq_ops+PV_IRQ_irq_disable);	\
		  PV_RESTORE_REGS(clobbers | CLBR_CALLEE_SAVE);)

#define ENABLE_INTERRUPTS(clobbers)					\
	PARA_SITE(PARA_PATCH(pv_irq_ops, PV_IRQ_irq_enable), clobbers,	\
		  PV_SAVE_REGS(clobbers | CLBR_CALLEE_SAVE);		\
		  call PARA_INDIRECT(pv_irq_ops+PV_IRQ_irq_enable);	\
		  PV_RESTORE_REGS(clobbers | CLBR_CALLEE_SAVE);)

#ifdef CONFIG_X86_64

#define GET_CR2_INTO_RAX				\
	call PARA_INDIRECT(pv_mmu_ops+PV_MMU_read_cr2)

#define PARAVIRT_ADJUST_EXCEPTION_FRAME					\
	PARA_SITE(PARA_PATCH(pv_irq_ops, PV_IRQ_adjust_exception_frame), \
		  CLBR_NONE,						\
		  call PARA_INDIRECT(pv_irq_ops+PV_IRQ_adjust_exception_frame))

#endif	/* CONFIG_X86_64 */

#endif /* __ASSEMBLY__ */
#else  /* CONFIG_PARAVIRT */
# define default_banner x86_init_noop
#ifndef __ASSEMBLY__
static inline void paravirt_arch_dup_mmap(struct mm_struct *oldmm,
					  struct mm_struct *mm)
{
}

static inline void paravirt_arch_exit_mmap(struct mm_struct *mm)
{
}
#endif /* __ASSEMBLY__ */
#endif /* !CONFIG_PARAVIRT */
#endif /* _ASM_X86_PARAVIRT_H */
