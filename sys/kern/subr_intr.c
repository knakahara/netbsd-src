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

/* XXXX temporary */
static u_int
cpuid2cpuindex(cpuid_t cpuid)
{
	CPU_INFO_ITERATOR cii;
	struct cpu_info *ci;

	for (CPU_INFO_FOREACH(cii, ci)) {
		if (ci->ci_cpuid == cpuid) {
			return ci->ci_index;
		}
	}
	return UINT_MAX;
}

static int
intrctl_affinity(void *data)
{
	struct kcpuset *intr_cpuset;
	struct intr_set *iset;
	cpuid_t cpuid;
	u_int cpu_index;
	void *ich;
	int error;

	iset = data;
	cpuid = iset->cpuid;
	cpu_index = cpuid2cpuindex(cpuid);
	if (cpu_index == UINT_MAX) {
		printf("cpu id %ld is invalid\n", cpuid);
		return EINVAL;
	}

	if (!kcpuset_isset(kcpuset_running, cpu_index)) {
		printf("cpu index %u is not running\n", cpu_index);
		return EINVAL;
	}

	kcpuset_create(&intr_cpuset, true);
	kcpuset_set(intr_cpuset, cpu_index);

	mutex_enter(&cpu_lock);

	ich = intr_intrctl_handler(iset->intrid);
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

static int
intrctl_intr(void *data)
{
	return intrctl_intr_md(data);
}

static int
intrctl_nointr(void *data)
{
	return intrctl_nointr_md(data);
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
