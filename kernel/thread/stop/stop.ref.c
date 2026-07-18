/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <c_std.h>
#include <context.h>
#include <hw.h>
#include <thread.h>
#include <dosched.h>
#include <runlist.h>
#include <asid.h>
#include <stop.h>
#include <timer.h>
#include <vm.h>
#include <cpuint.h>
#include <id.h>
#include <alloc.h>

#ifdef SHUTDOWN_AFTER_GUEST_EXIT
/*
 * The root VM (no parent -- H2K_id_to_context() on vmblock->parent returns
 * NULL only for the boot VM, since vmblocks[0] is never assigned) has just
 * drained its last CPU: nobody is left to hand control back to, so bring
 * the physical machine down instead of idling forever.  The other hw
 * threads are parked in wait(); wake them all with an NMI so each executes
 * stop() via the NMI_STOP handler (which this build option pulls in), then
 * stop this hw thread too.  Never returns.
 *
 * Deliberately targets MAX_HTHREADS_MASK (every hw thread this build can
 * possibly have started) rather than H2K_gp->hthreads_mask: the latter is
 * a one-shot modectl readback taken right after hwthreads_mask() issues
 * async start requests to the other threads, so it only reflects whichever
 * threads had already flipped their own enable bit by that point -- a
 * race, not a complete picture.  NMI-ing a thread that was never started
 * is a harmless no-op.
 */
static void H2K_hw_shutdown(s32_t status) __attribute__((noreturn));
static void H2K_hw_shutdown(s32_t status)
{
	u32_t others_mask = MAX_HTHREADS_MASK & ~(1u << get_hwtnum());

	asm volatile
		(
		 " nmi(%0)\n"
		 :
		 : "r"(others_mask)
		 );

	for (;;) {
		asm volatile
			(
			 " stop(%0)\n"
			 :
			 : "r"(status)
			 );
	}
}
#endif

void H2K_thread_stop(s32_t status, H2K_thread_context *me)
{
	H2K_vmblock_t *vmblock = me->vmblock;
	H2K_thread_context *parent_context;
	H2K_vmblock_t *parent_vmblock;

	BKL_LOCK(&H2K_bkl);
	H2K_timer_cancel_withlock(me);
	H2K_runlist_remove(me);
	H2K_asid_table_dec(me->ssr_asid);
	H2K_thread_context_clear(me);
	me->next = vmblock->free_threads;
	vmblock->free_threads = me;
	vmblock->num_cpus--;
	vmblock->status = status;

	if (status != 0 || vmblock->num_cpus == 0) { // signal parent
		parent_context = H2K_id_to_context(vmblock->parent);
		if (parent_context != NULL
				&& parent_context->status != H2K_STATUS_DEAD) { // parent exists
			parent_vmblock = parent_context->vmblock;
			H2K_vm_cpuint_post_locked(parent_vmblock, parent_context, H2K_VM_CHILDINT, parent_vmblock->intinfo);
		} else if (vmblock->num_cpus == 0) { // no parent; just deallocate.
			/* Can't free immediately because H2K_switch reads from *me */
			/* EJP: I think this is OK now if we dosched(NULL,htnum)? */
			H2K_mem_alloc_free(vmblock);
#ifdef SHUTDOWN_AFTER_GUEST_EXIT
			BKL_UNLOCK();
			H2K_hw_shutdown(status);
			/* NOTREACHED */
#endif
		}
	}
	/* If we dosched(NULL,get_hwtnum()) I think we can remove special cases in free() */
	H2K_dosched(NULL,get_hwtnum());
}

