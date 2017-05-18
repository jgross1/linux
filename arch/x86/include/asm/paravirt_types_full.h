#ifndef _ASM_X86_PARAVIRT_TYPES_FULL_H
#define _ASM_X86_PARAVIRT_TYPES_FULL_H

struct pv_lazy_ops {
	/* Set deferred update mode, used for batching operations. */
	void (*enter)(void);
	void (*leave)(void);
	void (*flush)(void);
};

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

struct pvfull_mmu_ops {
	unsigned long (*read_cr2)(void);
	void (*write_cr2)(unsigned long);

	unsigned long (*read_cr3)(void);
	void (*write_cr3)(unsigned long);

	/*
	 * Hooks for intercepting the creation/use/destruction of an
	 * mm_struct.
	 */
	void (*activate_mm)(struct mm_struct *prev,
			    struct mm_struct *next);
	void (*dup_mmap)(struct mm_struct *oldmm,
			 struct mm_struct *mm);

	/* TLB operations */
	void (*flush_tlb_user)(void);
	void (*flush_tlb_kernel)(void);
	void (*flush_tlb_single)(unsigned long addr);

	/* Hooks for allocating and freeing a pagetable top-level */
	int  (*pgd_alloc)(struct mm_struct *mm);
	void (*pgd_free)(struct mm_struct *mm, pgd_t *pgd);

	/*
	 * Hooks for allocating/releasing pagetable pages when they're
	 * attached to a pagetable
	 */
	void (*alloc_pte)(struct mm_struct *mm, unsigned long pfn);
	void (*alloc_pmd)(struct mm_struct *mm, unsigned long pfn);
	void (*alloc_pud)(struct mm_struct *mm, unsigned long pfn);
	void (*alloc_p4d)(struct mm_struct *mm, unsigned long pfn);
	void (*release_pte)(unsigned long pfn);
	void (*release_pmd)(unsigned long pfn);
	void (*release_pud)(unsigned long pfn);
	void (*release_p4d)(unsigned long pfn);

	/* Pagetable manipulation functions */
	void (*set_pte)(pte_t *ptep, pte_t pteval);
	void (*set_pte_at)(struct mm_struct *mm, unsigned long addr,
			   pte_t *ptep, pte_t pteval);
	void (*set_pmd)(pmd_t *pmdp, pmd_t pmdval);
	void (*set_pmd_at)(struct mm_struct *mm, unsigned long addr,
			   pmd_t *pmdp, pmd_t pmdval);
	void (*set_pud_at)(struct mm_struct *mm, unsigned long addr,
			   pud_t *pudp, pud_t pudval);
	void (*pte_update)(struct mm_struct *mm, unsigned long addr,
			   pte_t *ptep);

	pte_t (*ptep_modify_prot_start)(struct mm_struct *mm, unsigned long addr,
					pte_t *ptep);
	void (*ptep_modify_prot_commit)(struct mm_struct *mm, unsigned long addr,
					pte_t *ptep, pte_t pte);

	struct paravirt_callee_save pte_val;
	struct paravirt_callee_save make_pte;

	struct paravirt_callee_save pgd_val;
	struct paravirt_callee_save make_pgd;

#if CONFIG_PGTABLE_LEVELS >= 3
#ifdef CONFIG_X86_PAE
	void (*set_pte_atomic)(pte_t *ptep, pte_t pteval);
	void (*pte_clear)(struct mm_struct *mm, unsigned long addr,
			  pte_t *ptep);
	void (*pmd_clear)(pmd_t *pmdp);

#endif  /* CONFIG_X86_PAE */

	void (*set_pud)(pud_t *pudp, pud_t pudval);

	struct paravirt_callee_save pmd_val;
	struct paravirt_callee_save make_pmd;

#if CONFIG_PGTABLE_LEVELS >= 4
	struct paravirt_callee_save pud_val;
	struct paravirt_callee_save make_pud;

	void (*set_p4d)(p4d_t *p4dp, p4d_t p4dval);

#if CONFIG_PGTABLE_LEVELS >= 5
	struct paravirt_callee_save p4d_val;
	struct paravirt_callee_save make_p4d;

	void (*set_pgd)(pgd_t *pgdp, pgd_t pgdval);
#endif  /* CONFIG_PGTABLE_LEVELS >= 5 */

#endif  /* CONFIG_PGTABLE_LEVELS >= 4 */

#endif  /* CONFIG_PGTABLE_LEVELS >= 3 */

	struct pv_lazy_ops lazy_mode;

	/* Sometimes the physical address is a pfn, and sometimes its
	   an mfn.  We can tell which is which from the index. */
	void (*set_fixmap)(unsigned /* enum fixed_addresses */ idx,
			   phys_addr_t phys, pgprot_t flags);
};

extern struct pvfull_cpu_ops pvfull_cpu_ops;
extern struct pvfull_irq_ops pvfull_irq_ops;
extern struct pvfull_mmu_ops pvfull_mmu_ops;

enum paravirt_lazy_mode paravirt_get_lazy_mode(void);
void paravirt_start_context_switch(struct task_struct *prev);
void paravirt_end_context_switch(struct task_struct *next);

void paravirt_enter_lazy_mmu(void);
void paravirt_leave_lazy_mmu(void);
void paravirt_flush_lazy_mmu(void);

#endif  /* _ASM_X86_PARAVIRT_TYPES_FULL_H */
