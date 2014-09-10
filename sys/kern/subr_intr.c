/*	$NetBSD$	*/

/*
 * Copyright (c) 2014 Internet Initiative Japan Inc.
 * All rights reserved.
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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/errno.h>
#include <sys/cpu.h>
#include <sys/intr.h>
#include <sys/kcpuset.h>
#include <sys/proc.h>
#include <sys/xcall.h>

#include <sys/conf.h>
#include <sys/intrio.h>
#include <sys/kauth.h>

#include <machine/limits.h>


void	intrctlattach(int);

dev_type_ioctl(intrctl_ioctl);

const struct cdevsw intrctl_cdevsw = {
	.d_open = nullopen,
	.d_close = nullclose,
	.d_read = nullread,
	.d_write = nullwrite,
	.d_ioctl = intrctl_ioctl,
	.d_stop = nullstop,
	.d_tty = notty,
	.d_poll = nopoll,
	.d_mmap = nommap,
	.d_kqfilter = nokqfilter,
	.d_discard = nodiscard,
	.d_flag = D_OTHER | D_MPSAFE
};

void
intrctlattach(int dummy)
{
}

static int
intrctl_list(void *data)
{
	return intrctl_list_md(data);
}

static int
intrctl_affinity(void *data)
{
	kcpuset_t *intr_cpuset;
	struct intr_set *iset;
	u_int cpu_idx;
	void *ich;
	int error;

	iset = data;
	cpu_idx = iset->cpu_index;
	if (!kcpuset_isset(kcpuset_running, cpu_idx)) {
		printf("cpu index %u is not running\n", cpu_idx);
		return EINVAL;
	}

	kcpuset_create(&intr_cpuset, true);
	kcpuset_set(intr_cpuset, cpu_idx);

	mutex_enter(&cpu_lock);

	ich = intr_get_handler(iset->intrid);
	if (ich == NULL) {
		error = EINVAL;
		goto out;
	}
	error = intr_distribute(ich, intr_cpuset, NULL);

out:
	mutex_exit(&cpu_lock);
	kcpuset_destroy(intr_cpuset);

	return error;
}

#define UNSET_NOINTR_SHIELD	0
#define SET_NOINTR_SHIELD	1

static void
intr_shield_xcall(void *arg1, void *arg2)
{
	struct cpu_info *ci;
	struct schedstate_percpu *spc;
	int shield;
	int s;

	ci = arg1;
	shield = (int)(intptr_t)arg2;

	spc = &ci->ci_schedstate;
	s = splsched();
	if (shield == UNSET_NOINTR_SHIELD)
		spc->spc_flags &= ~SPCF_NOINTR;
	else if (shield == SET_NOINTR_SHIELD)
		spc->spc_flags |= SPCF_NOINTR;
	splx(s);

	return;
}

static int
intr_shield(u_int cpu_idx, int shield)
{
	struct cpu_info *ci;
	struct schedstate_percpu *spc;

	ci = cpu_lookup(cpu_idx);
	if (ci == NULL) {
		return EINVAL;
	}
	spc = &ci->ci_schedstate;

	if (shield == UNSET_NOINTR_SHIELD) {
		if ((spc->spc_flags & SPCF_NOINTR) == 0)
			return 0;
	}
	else if (shield == SET_NOINTR_SHIELD) {
		if ((spc->spc_flags & SPCF_NOINTR) != 0)
			return 0;
	}

	if (ci == curcpu() || !mp_online) {
		intr_shield_xcall(ci, (void *)(intptr_t)shield);
	}
	else {
		uint64_t where;
		where = xc_unicast(0, intr_shield_xcall, ci,
			(void *)(intptr_t)shield, ci);
		xc_wait(where);
	}

	spc->spc_lastmod = time_second;
	return 0;
}

static int
intr_avert_intr(u_int cpu_idx)
{
	kcpuset_t *cpuset;
	void *ich;
	int error;
	int i;
	int nids;
	char **ids;

	KASSERT(mutex_owned(&cpu_lock));

	kcpuset_create(&cpuset, true);
	kcpuset_set(cpuset, cpu_idx);

	error = intr_construct_intrids(cpuset, &ids, &nids);
	if (error)
		return error;
	if (nids == 0)
		return 0; /* nothing to do */

	intr_get_available(cpuset);
	kcpuset_clear(cpuset, cpu_idx);
	if (kcpuset_iszero(cpuset)) {
		printf("no available cpu\n");
		return ENOENT;
	}

	for (i = 0; i < nids; i++) {
		ich = intr_get_handler(ids[i]);
		if (ich == NULL) {
			error = ENOENT;
			break;
		}
		error = intr_distribute(ich, cpuset, NULL);
		if (error)
			break;
	}

	intr_destruct_intrids(ids, nids);
	kcpuset_destroy(cpuset);
	return error;
}

static int
intrctl_intr(void *data)
{
	int error;
	u_int cpu_idx;

	cpu_idx = *(u_int *)data;

	mutex_enter(&cpu_lock);
	error = intr_shield(cpu_idx, UNSET_NOINTR_SHIELD);
	mutex_exit(&cpu_lock);

	return error;
}

static int
intrctl_nointr(void *data)
{
	int error;
	u_int cpu_idx;

	cpu_idx = *(u_int *)data;

	mutex_enter(&cpu_lock);
	error = intr_shield(cpu_idx, SET_NOINTR_SHIELD);
	if (error) {
		mutex_exit(&cpu_lock);
		return error;
	}
	error = intr_avert_intr(cpu_idx);
	mutex_exit(&cpu_lock);

	return error;
}

int
intrctl_ioctl(dev_t dev, u_long cmd, void *data, int flag, lwp_t *l)
{
	int error;

	switch (cmd) {
	case IOC_INTR_LIST:
		error = intrctl_list(data);
		break;
	case IOC_INTR_AFFINITY:
		error = kauth_authorize_system(l->l_cred, KAUTH_SYSTEM_INTR,
		    KAUTH_REQ_SYSTEM_INTR_AFFINITY, NULL, NULL, NULL);
		if (error)
			break;
		error = intrctl_affinity(data);
		break;
	case IOC_INTR_INTR:
		error = kauth_authorize_system(l->l_cred, KAUTH_SYSTEM_CPU,
		    KAUTH_REQ_SYSTEM_CPU_SETSTATE, NULL, NULL, NULL);
		if (error)
			break;
		error = intrctl_intr(data);
		break;
	case IOC_INTR_NOINTR:
		error = kauth_authorize_system(l->l_cred, KAUTH_SYSTEM_CPU,
		    KAUTH_REQ_SYSTEM_CPU_SETSTATE, NULL, NULL, NULL);
		if (error)
			break;
		error = intrctl_nointr(data);
		break;
	default:
		error = ENOTTY;
		break;
	}

	return error;
}
