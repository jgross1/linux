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
#else

static inline enum paravirt_lazy_mode paravirt_get_lazy_mode(void)
{
	return PARAVIRT_LAZY_NONE;
}

#endif

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

static inline void paravirt_arch_exit_mmap(struct mm_struct *mm)
{
	PVOP_VCALL1(pv_mmu_ops.exit_mmap, mm);
}

static inline void flush_tlb_others(const struct cpumask *cpumask,
				    struct mm_struct *mm,
				    unsigned long start,
				    unsigned long end)
{
	PVOP_VCALL4(pv_mmu_ops.flush_tlb_others, cpumask, mm, start, end);
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

#endif /* __ASSEMBLY__ */
#else  /* CONFIG_PARAVIRT */
# define default_banner x86_init_noop
#ifndef __ASSEMBLY__
static inline void paravirt_arch_exit_mmap(struct mm_struct *mm)
{
}
#endif /* __ASSEMBLY__ */
#endif /* !CONFIG_PARAVIRT */

#ifndef CONFIG_PARAVIRT_FULL
#ifndef __ASSEMBLY__
static inline void paravirt_arch_dup_mmap(struct mm_struct *oldmm,
					  struct mm_struct *mm)
{
}
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PARAVIRT_FULL */
#endif /* _ASM_X86_PARAVIRT_H */
