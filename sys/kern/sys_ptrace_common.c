/*	$NetBSD: sys_ptrace_common.c,v 1.12 2017/01/26 03:54:01 christos Exp $	*/

/*-
 * Copyright (c) 2008, 2009 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Andrew Doran.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*-
 * Copyright (c) 1982, 1986, 1989, 1993
 *	The Regents of the University of California.  All rights reserved.
 * (c) UNIX System Laboratories, Inc.
 * All or some portions of this file are derived from material licensed
 * to the University of California by American Telephone and Telegraph
 * Co. or Unix System Laboratories, Inc. and are reproduced herein with
 * the permission of UNIX System Laboratories, Inc.
 *
 * This code is derived from software contributed to Berkeley by
 * Jan-Simon Pendry.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	from: @(#)sys_process.c	8.1 (Berkeley) 6/10/93
 */

/*-
 * Copyright (c) 1993 Jan-Simon Pendry.
 * Copyright (c) 1994 Christopher G. Demetriou.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Jan-Simon Pendry.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	from: @(#)sys_process.c	8.1 (Berkeley) 6/10/93
 */

/*
 * References:
 *	(1) Bach's "The Design of the UNIX Operating System",
 *	(2) sys/miscfs/procfs from UCB's 4.4BSD-Lite distribution,
 *	(3) the "4.4BSD Programmer's Reference Manual" published
 *		by USENIX and O'Reilly & Associates.
 * The 4.4BSD PRM does a reasonably good job of documenting what the various
 * ptrace() requests should actually do, and its text is quoted several times
 * in this file.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: sys_ptrace_common.c,v 1.12 2017/01/26 03:54:01 christos Exp $");

#ifdef _KERNEL_OPT
#include "opt_ptrace.h"
#include "opt_ktrace.h"
#include "opt_pax.h"
#endif

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/errno.h>
#include <sys/exec.h>
#include <sys/pax.h>
#include <sys/ptrace.h>
#include <sys/uio.h>
#include <sys/ras.h>
#include <sys/kmem.h>
#include <sys/kauth.h>
#include <sys/mount.h>
#include <sys/syscallargs.h>
#include <sys/module.h>
#include <sys/condvar.h>
#include <sys/mutex.h>

#include <uvm/uvm_extern.h>

#include <machine/reg.h>

#ifdef PTRACE

# ifdef DEBUG
#  define DPRINTF(a) uprintf a
# else
#  define DPRINTF(a)
# endif

static kauth_listener_t ptrace_listener;
static int process_auxv_offset(struct proc *, struct uio *);

#if 0
static int ptrace_cbref;
static kmutex_t ptrace_mtx;
static kcondvar_t ptrace_cv;
#endif

static int
ptrace_listener_cb(kauth_cred_t cred, kauth_action_t action, void *cookie,
    void *arg0, void *arg1, void *arg2, void *arg3)
{
	struct proc *p;
	int result;

	result = KAUTH_RESULT_DEFER;
	p = arg0;

#if 0
	mutex_enter(&ptrace_mtx);
	ptrace_cbref++;
	mutex_exit(&ptrace_mtx);
#endif
	if (action != KAUTH_PROCESS_PTRACE)
		goto out;

	switch ((u_long)arg1) {
	case PT_TRACE_ME:
	case PT_ATTACH:
	case PT_WRITE_I:
	case PT_WRITE_D:
	case PT_READ_I:
	case PT_READ_D:
	case PT_IO:
#ifdef PT_GETREGS
	case PT_GETREGS:
#endif
#ifdef PT_SETREGS
	case PT_SETREGS:
#endif
#ifdef PT_GETFPREGS
	case PT_GETFPREGS:
#endif
#ifdef PT_SETFPREGS
	case PT_SETFPREGS:
#endif
#ifdef __HAVE_PTRACE_WATCHPOINTS
	case PT_READ_WATCHPOINT:
	case PT_WRITE_WATCHPOINT:
	case PT_COUNT_WATCHPOINTS:
#endif
	case PT_SET_EVENT_MASK:
	case PT_GET_EVENT_MASK:
	case PT_GET_PROCESS_STATE:
	case PT_SET_SIGINFO:
	case PT_GET_SIGINFO:
#ifdef __HAVE_PTRACE_MACHDEP
	PTRACE_MACHDEP_REQUEST_CASES
#endif
		if (kauth_cred_getuid(cred) != kauth_cred_getuid(p->p_cred) ||
		    ISSET(p->p_flag, PK_SUGID)) {
			break;
		}

		result = KAUTH_RESULT_ALLOW;

	break;

#ifdef PT_STEP
	case PT_STEP:
#endif
	case PT_CONTINUE:
	case PT_KILL:
	case PT_DETACH:
	case PT_LWPINFO:
	case PT_SYSCALL:
	case PT_SYSCALLEMU:
	case PT_DUMPCORE:
		result = KAUTH_RESULT_ALLOW;
		break;

	default:
		break;
	}

 out:
#if 0
	mutex_enter(&ptrace_mtx);
	if (--ptrace_cbref == 0)
		cv_broadcast(&ptrace_cv);
	mutex_exit(&ptrace_mtx);
#endif

	return result;
}

int
ptrace_init(void)
{

#if 0
	mutex_init(&ptrace_mtx, MUTEX_DEFAULT, IPL_NONE);
	cv_init(&ptrace_cv, "ptracecb");
	ptrace_cbref = 0;
#endif
	ptrace_listener = kauth_listen_scope(KAUTH_SCOPE_PROCESS,
	    ptrace_listener_cb, NULL);
	return 0;
}

int
ptrace_fini(void)
{

	kauth_unlisten_scope(ptrace_listener);

#if 0
	/* Make sure no-one is executing our kauth listener */

	mutex_enter(&ptrace_mtx);
	while (ptrace_cbref != 0)
		cv_wait(&ptrace_cv, &ptrace_mtx);
	mutex_exit(&ptrace_mtx);
	mutex_destroy(&ptrace_mtx);
	cv_destroy(&ptrace_cv);
#endif

	return 0;
}

int
do_ptrace(struct ptrace_methods *ptm, struct lwp *l, int req, pid_t pid,
    void *addr, int data, register_t *retval)
{
	struct proc *p = l->l_proc;
	struct lwp *lt;
#ifdef PT_STEP
	struct lwp *lt2;
#endif
	struct proc *t;				/* target process */
	struct uio uio;
	struct iovec iov;
	struct ptrace_io_desc piod;
	struct ptrace_event pe;
	struct ptrace_state ps;
	struct ptrace_lwpinfo pl;
	struct ptrace_siginfo psi;
#ifdef __HAVE_PTRACE_WATCHPOINTS
	struct ptrace_watchpoint pw;
#endif
	struct vmspace *vm;
	int error, write, tmp, pheld;
	int signo = 0;
	int resume_all;
	ksiginfo_t ksi;
	char *path;
	int len = 0;
	error = 0;

	/*
	 * If attaching or detaching, we need to get a write hold on the
	 * proclist lock so that we can re-parent the target process.
	 */
	mutex_enter(proc_lock);

	/* "A foolish consistency..." XXX */
	if (req == PT_TRACE_ME) {
		t = p;
		mutex_enter(t->p_lock);
	} else {
		/* Find the process we're supposed to be operating on. */
		t = proc_find(pid);
		if (t == NULL) {
			mutex_exit(proc_lock);
			return ESRCH;
		}

		/* XXX-elad */
		mutex_enter(t->p_lock);
		error = kauth_authorize_process(l->l_cred, KAUTH_PROCESS_CANSEE,
		    t, KAUTH_ARG(KAUTH_REQ_PROCESS_CANSEE_ENTRY), NULL, NULL);
		if (error) {
			mutex_exit(proc_lock);
			mutex_exit(t->p_lock);
			return ESRCH;
		}
	}

	/*
	 * Grab a reference on the process to prevent it from execing or
	 * exiting.
	 */
	if (!rw_tryenter(&t->p_reflock, RW_READER)) {
		mutex_exit(proc_lock);
		mutex_exit(t->p_lock);
		return EBUSY;
	}

	/* Make sure we can operate on it. */
	switch (req) {
	case  PT_TRACE_ME:
		/* Saying that you're being traced is always legal. */
		break;

	case  PT_ATTACH:
		/*
		 * You can't attach to a process if:
		 *	(1) it's the process that's doing the attaching,
		 */
		if (t->p_pid == p->p_pid) {
			error = EINVAL;
			break;
		}

		/*
		 *  (2) it's a system process
		 */
		if (t->p_flag & PK_SYSTEM) {
			error = EPERM;
			break;
		}

		/*
		 *	(3) it's already being traced, or
		 */
		if (ISSET(t->p_slflag, PSL_TRACED)) {
			error = EBUSY;
			break;
		}

		/*
		 * 	(4) the tracer is chrooted, and its root directory is
		 * 	    not at or above the root directory of the tracee
		 */
		mutex_exit(t->p_lock);	/* XXXSMP */
		tmp = proc_isunder(t, l);
		mutex_enter(t->p_lock);	/* XXXSMP */
		if (!tmp) {
			error = EPERM;
			break;
		}
		break;

	case  PT_READ_I:
	case  PT_READ_D:
	case  PT_WRITE_I:
	case  PT_WRITE_D:
	case  PT_IO:
#ifdef PT_GETREGS
	case  PT_GETREGS:
#endif
#ifdef PT_SETREGS
	case  PT_SETREGS:
#endif
#ifdef PT_GETFPREGS
	case  PT_GETFPREGS:
#endif
#ifdef PT_SETFPREGS
	case  PT_SETFPREGS:
#endif
#ifdef __HAVE_PTRACE_WATCHPOINTS
	case  PT_READ_WATCHPOINT:
	case  PT_WRITE_WATCHPOINT:
	case  PT_COUNT_WATCHPOINTS:
#endif
#ifdef __HAVE_PTRACE_MACHDEP
	PTRACE_MACHDEP_REQUEST_CASES
#endif
		/*
		 * You can't read/write the memory or registers of a process
		 * if the tracer is chrooted, and its root directory is not at
		 * or above the root directory of the tracee.
		 */
		mutex_exit(t->p_lock);	/* XXXSMP */
		tmp = proc_isunder(t, l);
		mutex_enter(t->p_lock);	/* XXXSMP */
		if (!tmp) {
			error = EPERM;
			break;
		}
		/*FALLTHROUGH*/

	case  PT_CONTINUE:
	case  PT_KILL:
	case  PT_DETACH:
	case  PT_LWPINFO:
	case  PT_SYSCALL:
	case  PT_SYSCALLEMU:
	case  PT_DUMPCORE:
#ifdef PT_STEP
	case  PT_STEP:
#endif
	case  PT_SET_EVENT_MASK:
	case  PT_GET_EVENT_MASK:
	case  PT_GET_PROCESS_STATE:
	case  PT_SET_SIGINFO:
	case  PT_GET_SIGINFO:
		/*
		 * You can't do what you want to the process if:
		 *	(1) It's not being traced at all,
		 */
		if (!ISSET(t->p_slflag, PSL_TRACED)) {
			error = EPERM;
			break;
		}

		/*
		 *	(2) it's being traced by procfs (which has
		 *	    different signal delivery semantics),
		 */
		if (ISSET(t->p_slflag, PSL_FSTRACE)) {
			DPRINTF(("file system traced\n"));
			error = EBUSY;
			break;
		}

		/*
		 *	(3) it's not being traced by _you_, or
		 */
		if (t->p_pptr != p) {
			DPRINTF(("parent %d != %d\n", t->p_pptr->p_pid,
			    p->p_pid));
			error = EBUSY;
			break;
		}

		/*
		 *	(4) it's not currently stopped.
		 */
		if (t->p_stat != SSTOP || !t->p_waited /* XXXSMP */) {
			DPRINTF(("stat %d flag %d\n", t->p_stat,
			    !t->p_waited));
			error = EBUSY;
			break;
		}
		break;

	default:			/* It was not a legal request. */
		error = EINVAL;
		break;
	}

	if (error == 0) {
		error = kauth_authorize_process(l->l_cred,
		    KAUTH_PROCESS_PTRACE, t, KAUTH_ARG(req),
		    NULL, NULL);
	}
	if (error == 0) {
		lt = lwp_find_first(t);
		if (lt == NULL)
			error = ESRCH;
	}

	if (error != 0) {
		mutex_exit(proc_lock);
		mutex_exit(t->p_lock);
		rw_exit(&t->p_reflock);
		return error;
	}

	/* Do single-step fixup if needed. */
	FIX_SSTEP(t);
	KASSERT(lt != NULL);
	lwp_addref(lt);

	/*
	 * Which locks do we need held? XXX Ugly.
	 */
	switch (req) {
#ifdef PT_STEP
	case PT_STEP:
#endif
	case PT_CONTINUE:
	case PT_DETACH:
	case PT_KILL:
	case PT_SYSCALL:
	case PT_SYSCALLEMU:
	case PT_ATTACH:
	case PT_TRACE_ME:
		pheld = 1;
		break;
	default:
		mutex_exit(proc_lock);
		mutex_exit(t->p_lock);
		pheld = 0;
		break;
	}

	/* Now do the operation. */
	write = 0;
	*retval = 0;
	tmp = 0;
	resume_all = 1;

	switch (req) {
	case  PT_TRACE_ME:
		/* Just set the trace flag. */
		SET(t->p_slflag, PSL_TRACED);
		t->p_opptr = t->p_pptr;
		break;

	case  PT_WRITE_I:		/* XXX no separate I and D spaces */
	case  PT_WRITE_D:
#if defined(__HAVE_RAS)
		/*
		 * Can't write to a RAS
		 */
		if (ras_lookup(t, addr) != (void *)-1) {
			error = EACCES;
			break;
		}
#endif
		write = 1;
		tmp = data;
		/* FALLTHROUGH */

	case  PT_READ_I:		/* XXX no separate I and D spaces */
	case  PT_READ_D:
		/* write = 0 done above. */
		iov.iov_base = (void *)&tmp;
		iov.iov_len = sizeof(tmp);
		uio.uio_iov = &iov;
		uio.uio_iovcnt = 1;
		uio.uio_offset = (off_t)(unsigned long)addr;
		uio.uio_resid = sizeof(tmp);
		uio.uio_rw = write ? UIO_WRITE : UIO_READ;
		UIO_SETUP_SYSSPACE(&uio);

		error = process_domem(l, lt, &uio);
		if (!write)
			*retval = tmp;
		break;

	case  PT_IO:
		error = ptm->ptm_copyinpiod(&piod, addr);
		if (error)
			break;

		iov.iov_base = piod.piod_addr;
		iov.iov_len = piod.piod_len;
		uio.uio_iov = &iov;
		uio.uio_iovcnt = 1;
		uio.uio_offset = (off_t)(unsigned long)piod.piod_offs;
		uio.uio_resid = piod.piod_len;

		switch (piod.piod_op) {
		case PIOD_READ_D:
		case PIOD_READ_I:
			uio.uio_rw = UIO_READ;
			break;
		case PIOD_WRITE_D:
		case PIOD_WRITE_I:
			/*
			 * Can't write to a RAS
			 */
			if (ras_lookup(t, addr) != (void *)-1) {
				return EACCES;
			}
			uio.uio_rw = UIO_WRITE;
			break;
		case PIOD_READ_AUXV:
			req = PT_READ_D;
			uio.uio_rw = UIO_READ;
			tmp = t->p_execsw->es_arglen;
			if (uio.uio_offset > tmp)
				return EIO;
			if (uio.uio_resid > tmp - uio.uio_offset)
				uio.uio_resid = tmp - uio.uio_offset;
			piod.piod_len = iov.iov_len = uio.uio_resid;
			error = process_auxv_offset(t, &uio);
			break;
		default:
			error = EINVAL;
			break;
		}
		if (error)
			break;
		error = proc_vmspace_getref(l->l_proc, &vm);
		if (error)
			break;
		uio.uio_vmspace = vm;

		error = process_domem(l, lt, &uio);
		piod.piod_len -= uio.uio_resid;
		(void) ptm->ptm_copyoutpiod(&piod, addr);

		uvmspace_free(vm);
		break;

	case  PT_DUMPCORE:
		if ((path = addr) != NULL) {
			char *dst;
			len = data;

			if (len < 0 || len >= MAXPATHLEN) {
				error = EINVAL;
				break;
			}
			dst = kmem_alloc(len + 1, KM_SLEEP);
			if ((error = copyin(path, dst, len)) != 0) {
				kmem_free(dst, len + 1);
				break;
			}
			path = dst;
			path[len] = '\0';
		}
		error = (*coredump_vec)(lt, path);
		if (path)
			kmem_free(path, len + 1);
		break;

#ifdef PT_STEP
	case  PT_STEP:
		/*
		 * From the 4.4BSD PRM:
		 * "Execution continues as in request PT_CONTINUE; however
		 * as soon as possible after execution of at least one
		 * instruction, execution stops again. [ ... ]"
		 */
#endif
	case  PT_CONTINUE:
	case  PT_SYSCALL:
	case  PT_DETACH:
		if (req == PT_SYSCALL) {
			if (!ISSET(t->p_slflag, PSL_SYSCALL)) {
				SET(t->p_slflag, PSL_SYSCALL);
#ifdef __HAVE_SYSCALL_INTERN
				(*t->p_emul->e_syscall_intern)(t);
#endif
			}
		} else {
			if (ISSET(t->p_slflag, PSL_SYSCALL)) {
				CLR(t->p_slflag, PSL_SYSCALL);
#ifdef __HAVE_SYSCALL_INTERN
				(*t->p_emul->e_syscall_intern)(t);
#endif
			}
		}
		t->p_trace_enabled = trace_is_enabled(t);

		/*
		 * Pick up the LWPID, if supplied.  There are two cases:
		 * data < 0 : step or continue single thread, lwp = -data
		 * data > 0 in PT_STEP : step this thread, continue others
		 * For operations other than PT_STEP, data > 0 means
		 * data is the signo to deliver to the process.
		 */
		tmp = data;
		if (tmp >= 0) {
#ifdef PT_STEP
			if (req == PT_STEP)
				signo = 0;
			else
#endif
			{
				signo = tmp;
				tmp = 0;	/* don't search for LWP */
			}
		} else
			tmp = -tmp;

		if (tmp > 0) {
			if (req == PT_DETACH) {
				error = EINVAL;
				break;
			}
			lwp_delref2 (lt);
			lt = lwp_find(t, tmp);
			if (lt == NULL) {
				error = ESRCH;
				break;
			}
			lwp_addref(lt);
			resume_all = 0;
			signo = 0;
		}

		/*
		 * From the 4.4BSD PRM:
		 * "The data argument is taken as a signal number and the
		 * child's execution continues at location addr as if it
		 * incurred that signal.  Normally the signal number will
		 * be either 0 to indicate that the signal that caused the
		 * stop should be ignored, or that value fetched out of
		 * the process's image indicating which signal caused
		 * the stop.  If addr is (int *)1 then execution continues
		 * from where it stopped."
		 */

		/* Check that the data is a valid signal number or zero. */
		if (signo < 0 || signo >= NSIG) {
			error = EINVAL;
			break;
		}

		/* If the address parameter is not (int *)1, set the pc. */
		if ((int *)addr != (int *)1) {
			error = process_set_pc(lt, addr);
			if (error != 0)
				break;
		}
#ifdef PT_STEP
		/*
		 * Arrange for a single-step, if that's requested and possible.
		 * More precisely, set the single step status as requested for
		 * the requested thread, and clear it for other threads.
		 */
		LIST_FOREACH(lt2, &t->p_lwps, l_sibling) {
			if (lt != lt2) {
				lwp_lock(lt2);
				process_sstep(lt2, 0);
				lwp_unlock(lt2);
			}
		}
		error = process_sstep(lt, req == PT_STEP);
		if (error)
			break;
#endif
		if (req == PT_DETACH) {
			CLR(t->p_slflag, PSL_TRACED|PSL_FSTRACE|PSL_SYSCALL);

			/* give process back to original parent or init */
			if (t->p_opptr != t->p_pptr) {
				struct proc *pp = t->p_opptr;
				proc_reparent(t, pp ? pp : initproc);
			}

			/* not being traced any more */
			t->p_opptr = NULL;
		}
	sendsig:
		t->p_fpid = 0;
		t->p_vfpid = 0;
		t->p_vfpid_done = 0;
		t->p_lwp_created = 0;
		t->p_lwp_exited = 0;
		/* Finally, deliver the requested signal (or none). */
		if (t->p_stat == SSTOP) {
			/*
			 * Unstop the process.  If it needs to take a
			 * signal, make all efforts to ensure that at
			 * an LWP runs to see it.
			 */
			t->p_xsig = signo;
			if (resume_all)
				proc_unstop(t);
			else
				lwp_unstop(lt);
		} else if (t->p_sigctx.ps_faked) {
			if (signo != t->p_sigctx.ps_info._signo) {
				error = EINVAL;
				break;
			}
			t->p_sigctx.ps_faked = false;
			KSI_INIT_EMPTY(&ksi);
			ksi.ksi_info = t->p_sigctx.ps_info;
			ksi.ksi_lid = t->p_sigctx.ps_lwp;
			kpsignal2(t, &ksi);
		} else if (signo != 0) {
			KSI_INIT_EMPTY(&ksi);
			ksi.ksi_signo = signo;
			kpsignal2(t, &ksi);
		}
		break;

	case  PT_SYSCALLEMU:
		if (!ISSET(t->p_slflag, PSL_SYSCALL) || t->p_stat != SSTOP) {
			error = EINVAL;
			break;
		}
		SET(t->p_slflag, PSL_SYSCALLEMU);
		break;

	case  PT_KILL:
		/* just send the process a KILL signal. */
		signo = SIGKILL;
		goto sendsig;	/* in PT_CONTINUE, above. */

	case  PT_ATTACH:
		/*
		 * Go ahead and set the trace flag.
		 * Save the old parent (it's reset in
		 *   _DETACH, and also in kern_exit.c:wait4()
		 * Reparent the process so that the tracing
		 *   proc gets to see all the action.
		 * Stop the target.
		 */
		proc_changeparent(t, p);
		signo = SIGSTOP;
		goto sendsig;

	case  PT_GET_EVENT_MASK:
		if (data != sizeof(pe)) {
			DPRINTF(("ptrace(%d): %d != %zu\n", req,
			    data, sizeof(pe)));
			error = EINVAL;
			break;
		}
		memset(&pe, 0, sizeof(pe));
		pe.pe_set_event = ISSET(t->p_slflag, PSL_TRACEFORK) ?
		    PTRACE_FORK : 0;
		pe.pe_set_event |= ISSET(t->p_slflag, PSL_TRACEVFORK) ?
		    PTRACE_VFORK : 0;
		pe.pe_set_event |= ISSET(t->p_slflag, PSL_TRACEVFORK_DONE) ?
		    PTRACE_VFORK_DONE : 0;
		pe.pe_set_event |= ISSET(t->p_slflag, PSL_TRACELWP_CREATE) ?
		    PTRACE_LWP_CREATE : 0;
		pe.pe_set_event |= ISSET(t->p_slflag, PSL_TRACELWP_EXIT) ?
		    PTRACE_LWP_EXIT : 0;
		error = copyout(&pe, addr, sizeof(pe));
		break;

	case  PT_SET_EVENT_MASK:
		if (data != sizeof(pe)) {
			DPRINTF(("ptrace(%d): %d != %zu\n", req, data,
			    sizeof(pe)));
			error = EINVAL;
			break;
		}
		if ((error = copyin(addr, &pe, sizeof(pe))) != 0)
			return error;
		if (pe.pe_set_event & PTRACE_FORK)
			SET(t->p_slflag, PSL_TRACEFORK);
		else
			CLR(t->p_slflag, PSL_TRACEFORK);
#if notyet
		if (pe.pe_set_event & PTRACE_VFORK)
			SET(t->p_slflag, PSL_TRACEVFORK);
		else
			CLR(t->p_slflag, PSL_TRACEVFORK);
#else
		if (pe.pe_set_event & PTRACE_VFORK) {
			error = ENOTSUP;
			break;
		}
#endif
		if (pe.pe_set_event & PTRACE_VFORK_DONE)
			SET(t->p_slflag, PSL_TRACEVFORK_DONE);
		else
			CLR(t->p_slflag, PSL_TRACEVFORK_DONE);
		if (pe.pe_set_event & PTRACE_LWP_CREATE)
			SET(t->p_slflag, PSL_TRACELWP_CREATE);
		else
			CLR(t->p_slflag, PSL_TRACELWP_CREATE);
		if (pe.pe_set_event & PTRACE_LWP_EXIT)
			SET(t->p_slflag, PSL_TRACELWP_EXIT);
		else
			CLR(t->p_slflag, PSL_TRACELWP_EXIT);
		break;

	case  PT_GET_PROCESS_STATE:
		if (data != sizeof(ps)) {
			DPRINTF(("ptrace(%d): %d != %zu\n", req, data,
			    sizeof(ps)));
			error = EINVAL;
			break;
		}
		memset(&ps, 0, sizeof(ps));
		if (t->p_fpid) {
			ps.pe_report_event = PTRACE_FORK;
			ps.pe_other_pid = t->p_fpid;
		} else if (t->p_vfpid) {
			ps.pe_report_event = PTRACE_VFORK;
			ps.pe_other_pid = t->p_vfpid;
		} else if (t->p_vfpid_done) {
			ps.pe_report_event = PTRACE_VFORK_DONE;
			ps.pe_other_pid = t->p_vfpid_done;
		} else if (t->p_lwp_created) {
			ps.pe_report_event = PTRACE_LWP_CREATE;
			ps.pe_lwp = t->p_lwp_created;
		} else if (t->p_lwp_exited) {
			ps.pe_report_event = PTRACE_LWP_EXIT;
			ps.pe_lwp = t->p_lwp_exited;
		}
		error = copyout(&ps, addr, sizeof(ps));
		break;

	case PT_LWPINFO:
		if (data != sizeof(pl)) {
			DPRINTF(("ptrace(%d): %d != %zu\n", req, data,
			    sizeof(pl)));
			error = EINVAL;
			break;
		}
		error = copyin(addr, &pl, sizeof(pl));
		if (error)
			break;
		tmp = pl.pl_lwpid;
		lwp_delref(lt);
		mutex_enter(t->p_lock);
		if (tmp == 0)
			lt = lwp_find_first(t);
		else {
			lt = lwp_find(t, tmp);
			if (lt == NULL) {
				mutex_exit(t->p_lock);
				error = ESRCH;
				break;
			}
			lt = LIST_NEXT(lt, l_sibling);
		}
		while (lt != NULL && !lwp_alive(lt))
			lt = LIST_NEXT(lt, l_sibling);
		pl.pl_lwpid = 0;
		pl.pl_event = 0;
		if (lt) {
			lwp_addref(lt);
			pl.pl_lwpid = lt->l_lid;
			/*
			 * If we match the lwp, or it was sent to every lwp,
			 * we set PL_EVENT_SIGNAL.
			 * XXX: ps_lwp == 0 means everyone and noone, so
			 * check ps_signo too.
			 */
			if (lt->l_lid == t->p_sigctx.ps_lwp
			    || (t->p_sigctx.ps_lwp == 0 &&
			        t->p_sigctx.ps_info._signo))
				pl.pl_event = PL_EVENT_SIGNAL;
		}
		mutex_exit(t->p_lock);

		error = copyout(&pl, addr, sizeof(pl));
		break;

	case  PT_SET_SIGINFO:
		if (data != sizeof(psi)) {
			DPRINTF(("ptrace(%d): %d != %zu\n", req, data,
			    sizeof(psi)));
			error = EINVAL;
			break;
		}

		error = copyin(addr, &psi, sizeof(psi));
		if (error)
			break;

		/* Check that the data is a valid signal number or zero. */
		if (psi.psi_siginfo.si_signo < 0 ||
		    psi.psi_siginfo.si_signo >= NSIG) {
			error = EINVAL;
			break;
		}

		tmp = psi.psi_lwpid;
		if (tmp != 0)
			lwp_delref(lt);

		mutex_enter(t->p_lock);

		if (tmp != 0) {
			lt = lwp_find(t, tmp);
			if (lt == NULL) {
				mutex_exit(t->p_lock);
				error = ESRCH;
				break;
			}
			lwp_addref(lt);
		}

		t->p_sigctx.ps_faked = true;
		t->p_sigctx.ps_info = psi.psi_siginfo._info;
		t->p_sigctx.ps_lwp = psi.psi_lwpid;
		mutex_exit(t->p_lock);
		break;

	case  PT_GET_SIGINFO:
		if (data != sizeof(psi)) {
			DPRINTF(("ptrace(%d): %d != %zu\n", req, data,
			    sizeof(psi)));
			error = EINVAL;
			break;
		}
		mutex_enter(t->p_lock);
		psi.psi_siginfo._info = t->p_sigctx.ps_info;
		psi.psi_lwpid = t->p_sigctx.ps_lwp;
		mutex_exit(t->p_lock);

		error = copyout(&psi, addr, sizeof(psi));
		if (error)
			break;

		break;

#ifdef PT_SETREGS
	case  PT_SETREGS:
		write = 1;
#endif
#ifdef PT_GETREGS
	case  PT_GETREGS:
		/* write = 0 done above. */
#endif
#if defined(PT_SETREGS) || defined(PT_GETREGS)
		tmp = data;
		if (tmp != 0 && t->p_nlwps > 1) {
			lwp_delref(lt);
			mutex_enter(t->p_lock);
			lt = lwp_find(t, tmp);
			if (lt == NULL) {
				mutex_exit(t->p_lock);
				error = ESRCH;
				break;
			}
			lwp_addref(lt);
			mutex_exit(t->p_lock);
		}
		if (!process_validregs(lt))
			error = EINVAL;
		else {
			error = proc_vmspace_getref(p, &vm);
			if (error)
				break;
			iov.iov_base = addr;
			iov.iov_len = PROC_REGSZ(p);
			uio.uio_iov = &iov;
			uio.uio_iovcnt = 1;
			uio.uio_offset = 0;
			uio.uio_resid = iov.iov_len;
			uio.uio_rw = write ? UIO_WRITE : UIO_READ;
			uio.uio_vmspace = vm;

			error = ptm->ptm_doregs(l, lt, &uio);
			uvmspace_free(vm);
		}
		break;
#endif

#ifdef PT_SETFPREGS
	case  PT_SETFPREGS:
		write = 1;
		/*FALLTHROUGH*/
#endif
#ifdef PT_GETFPREGS
	case  PT_GETFPREGS:
		/* write = 0 done above. */
#endif
#if defined(PT_SETFPREGS) || defined(PT_GETFPREGS)
		tmp = data;
		if (tmp != 0 && t->p_nlwps > 1) {
			lwp_delref(lt);
			mutex_enter(t->p_lock);
			lt = lwp_find(t, tmp);
			if (lt == NULL) {
				mutex_exit(t->p_lock);
				error = ESRCH;
				break;
			}
			lwp_addref(lt);
			mutex_exit(t->p_lock);
		}
		if (!process_validfpregs(lt))
			error = EINVAL;
		else {
			error = proc_vmspace_getref(p, &vm);
			if (error)
				break;
			iov.iov_base = addr;
			iov.iov_len = PROC_FPREGSZ(p);
			uio.uio_iov = &iov;
			uio.uio_iovcnt = 1;
			uio.uio_offset = 0;
			uio.uio_resid = iov.iov_len;
			uio.uio_rw = write ? UIO_WRITE : UIO_READ;
			uio.uio_vmspace = vm;

			error = ptm->ptm_dofpregs(l, lt, &uio);
			uvmspace_free(vm);
		}
		break;
#endif

#ifdef __HAVE_PTRACE_WATCHPOINTS
		/*
		 * The "write" variable is used as type of operation.
		 * Possible values:
		 *	0 - return the number of supported hardware watchpoints
		 *	1 - set new watchpoint value
		 *	2 - get existing watchpoint image
		 */
	case  PT_WRITE_WATCHPOINT:
		write = 1;
	case  PT_READ_WATCHPOINT:
		/* write = 0 done above */

		if (data != sizeof(pw)) {
			DPRINTF(("ptrace(%d): %d != %zu\n", req,
			    data, sizeof(pe)));
			error = EINVAL;
			break;
		}
		error = copyin(addr, &pw, sizeof(pw));
		if (error)
			break;
		tmp = pw.pw_lwpid;
		if (tmp != 0 && t->p_nlwps > 1) {
			lwp_delref(lt);
			mutex_enter(t->p_lock);
			lt = lwp_find(t, tmp);
			if (lt == NULL) {
				mutex_exit(t->p_lock);
				error = ESRCH;
				break;
			}
			lwp_addref(lt);
			mutex_exit(t->p_lock);
		}
		++write;
	case  PT_COUNT_WATCHPOINTS:
		if (!process_validwatchpoint(lt))
			error = EINVAL;
		else {
			lwp_lock(lt);
			error = ptm->ptm_dowatchpoint(l, lt, write, &pw, addr,
			    retval);
			lwp_unlock(lt);
		}
		break;
#endif

#ifdef __HAVE_PTRACE_MACHDEP
	PTRACE_MACHDEP_REQUEST_CASES
		error = ptrace_machdep_dorequest(l, lt, req, addr, data);
		break;
#endif
	}

	if (pheld) {
		mutex_exit(t->p_lock);
		mutex_exit(proc_lock);
	}
	if (lt != NULL)
		lwp_delref(lt);
	rw_exit(&t->p_reflock);

	return error;
}

int
process_doregs(struct lwp *curl /*tracer*/,
    struct lwp *l /*traced*/,
    struct uio *uio)
{
#if defined(PT_GETREGS) || defined(PT_SETREGS)
	int error;
	struct reg r;
	char *kv;
	int kl;

	if (uio->uio_offset < 0 || uio->uio_offset > (off_t)sizeof(r))
		return EINVAL;

	kl = sizeof(r);
	kv = (char *)&r;

	kv += uio->uio_offset;
	kl -= uio->uio_offset;
	if ((size_t)kl > uio->uio_resid)
		kl = uio->uio_resid;

	error = process_read_regs(l, &r);
	if (error == 0)
		error = uiomove(kv, kl, uio);
	if (error == 0 && uio->uio_rw == UIO_WRITE) {
		if (l->l_stat != LSSTOP)
			error = EBUSY;
		else
			error = process_write_regs(l, &r);
	}

	uio->uio_offset = 0;
	return error;
#else
	return EINVAL;
#endif
}

int
process_validregs(struct lwp *l)
{

#if defined(PT_SETREGS) || defined(PT_GETREGS)
	return (l->l_flag & LW_SYSTEM) == 0;
#else
	return 0;
#endif
}

int
process_dofpregs(struct lwp *curl /*tracer*/,
    struct lwp *l /*traced*/,
    struct uio *uio)
{
#if defined(PT_GETFPREGS) || defined(PT_SETFPREGS)
	int error;
	struct fpreg r;
	char *kv;
	size_t kl;

	if (uio->uio_offset < 0 || uio->uio_offset > (off_t)sizeof(r))
		return EINVAL;

	kl = sizeof(r);
	kv = (char *)&r;

	kv += uio->uio_offset;
	kl -= uio->uio_offset;
	if (kl > uio->uio_resid)
		kl = uio->uio_resid;

	error = process_read_fpregs(l, &r, &kl);
	if (error == 0)
		error = uiomove(kv, kl, uio);
	if (error == 0 && uio->uio_rw == UIO_WRITE) {
		if (l->l_stat != LSSTOP)
			error = EBUSY;
		else
			error = process_write_fpregs(l, &r, kl);
	}
	uio->uio_offset = 0;
	return error;
#else
	return EINVAL;
#endif
}

int
process_validfpregs(struct lwp *l)
{

#if defined(PT_SETFPREGS) || defined(PT_GETFPREGS)
	return (l->l_flag & LW_SYSTEM) == 0;
#else
	return 0;
#endif
}

static int
process_auxv_offset(struct proc *p, struct uio *uio)
{
	struct ps_strings pss;
	int error;
	off_t off = (off_t)p->p_psstrp;

	if ((error = copyin_psstrings(p, &pss)) != 0)
		return error;

	if (pss.ps_envstr == NULL)
		return EIO;

	uio->uio_offset += (off_t)(vaddr_t)(pss.ps_envstr + pss.ps_nenvstr + 1);
#ifdef __MACHINE_STACK_GROWS_UP
	if (uio->uio_offset < off)
		return EIO;
#else
	if (uio->uio_offset > off)
		return EIO;
	if ((uio->uio_offset + uio->uio_resid) > off)
		uio->uio_resid = off - uio->uio_offset;
#endif
	return 0;
}

int
process_dowatchpoint(struct lwp *curl /*tracer*/, struct lwp *l /*traced*/,
    int operation, struct ptrace_watchpoint *pw, void *addr,
    register_t *retval)
{

#ifdef __HAVE_PTRACE_WATCHPOINTS
	int error;

	KASSERT(operation >= 0);
	KASSERT(operation <= 2);

	switch (operation) {
	case 0:
		return process_count_watchpoints(l, retval);
	case 1:
		error = process_read_watchpoint(l, pw);
		if (error)
			return error;
		return copyout(pw, addr, sizeof(*pw));
	default:
		return process_write_watchpoint(l, pw);
	}
#else
	return EINVAL;
#endif
}

int
process_validwatchpoint(struct lwp *l)
{

#ifdef __HAVE_PTRACE_WATCHPOINTS
	return (l->l_flag & LW_SYSTEM) == 0;
#else
	return 0;
#endif
}
#endif /* PTRACE */

MODULE(MODULE_CLASS_EXEC, ptrace_common, "");
 
static int
ptrace_common_modcmd(modcmd_t cmd, void *arg)
{
        int error;
 
        switch (cmd) {
        case MODULE_CMD_INIT:
                error = ptrace_init();
                break;
        case MODULE_CMD_FINI:
                error = ptrace_fini();
                break;
        default:
		ptrace_hooks();
                error = ENOTTY;
                break;
        }
        return error;
}
