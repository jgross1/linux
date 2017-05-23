#ifndef _ASM_X86_PARAVIRT_TYPES_FULL_H
#define _ASM_X86_PARAVIRT_TYPES_FULL_H

struct pvfull_cpu_ops {
	/* hooks for various privileged instructions */
	unsigned long (*get_debugreg)(int regno);
	void (*set_debugreg)(int regno, unsigned long value);

	unsigned long (*read_cr0)(void);
	void (*write_cr0)(unsigned long);

	unsigned long (*read_cr4)(void);
	void (*write_cr4)(unsigned long);

#ifdef CONFIG_X86_64
	unsigned long (*read_cr8)(void);
	void (*write_cr8)(unsigned long);
#endif

	/* Segment descriptor handling */
	void (*load_tr_desc)(void);
	void (*load_gdt)(const struct desc_ptr *);
	void (*load_idt)(const struct desc_ptr *);
	void (*store_idt)(struct desc_ptr *);
	void (*set_ldt)(const void *desc, unsigned entries);
	unsigned long (*store_tr)(void);
	void (*load_tls)(struct thread_struct *t, unsigned int cpu);
#ifdef CONFIG_X86_64
	void (*load_gs_index)(unsigned int idx);
#endif
	void (*write_ldt_entry)(struct desc_struct *ldt, int entrynum,
				const void *desc);
	void (*write_gdt_entry)(struct desc_struct *,
				int entrynum, const void *desc, int size);
	void (*write_idt_entry)(gate_desc *,
				int entrynum, const gate_desc *gate);
	void (*alloc_ldt)(struct desc_struct *ldt, unsigned entries);
	void (*free_ldt)(struct desc_struct *ldt, unsigned entries);

	void (*load_sp0)(struct tss_struct *tss, struct thread_struct *t);

	void (*set_iopl_mask)(unsigned mask);

	void (*wbinvd)(void);

	/* cpuid emulation, mostly so that caps bits can be disabled */
	void (*cpuid)(unsigned int *eax, unsigned int *ebx,
		      unsigned int *ecx, unsigned int *edx);

	/* Unsafe MSR operations.  These will warn or panic on failure. */
	u64 (*read_msr)(unsigned int msr);
	void (*write_msr)(unsigned int msr, unsigned low, unsigned high);

	/*
	 * Safe MSR operations.
	 * read sets err to 0 or -EIO.  write returns 0 or -EIO.
	 */
	u64 (*read_msr_safe)(unsigned int msr, int *err);
	int (*write_msr_safe)(unsigned int msr, unsigned low, unsigned high);

	u64 (*read_pmc)(int counter);

	/*
	 * Switch to usermode gs and return to 64-bit usermode using
	 * sysret.  Only used in 64-bit kernels to return to 64-bit
	 * processes.  Usermode register state, including %rsp, must
	 * already be restored.
	 */
	void (*usergs_sysret64)(void);

	/* Normal iret.  Jump to this with the std iret stack frame set up. */
	void (*iret)(void);

	void (*swapgs)(void);

	void (*start_context_switch)(struct task_struct *prev);
	void (*end_context_switch)(struct task_struct *next);
};

struct pvfull_irq_ops {
	void (*safe_halt)(void);
	void (*halt)(void);

#ifdef CONFIG_X86_64
	void (*adjust_exception_frame)(void);
#endif
};

extern struct pvfull_cpu_ops pvfull_cpu_ops;
extern struct pvfull_irq_ops pvfull_irq_ops;

#endif  /* _ASM_X86_PARAVIRT_TYPES_FULL_H */
