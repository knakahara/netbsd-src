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
#include <sys/kmem.h>
#include <sys/proc.h>
#include <sys/xcall.h>
#include <sys/sysctl.h>

#include <sys/conf.h>
#include <sys/intrio.h>
#include <sys/kauth.h>

#include <machine/limits.h>

#include <dev/pci/pcivar.h>

#ifdef INTR_DEBUG
#define DPRINTF(msg) printf msg
#else
#define DPRINTF(msg)
#endif

static struct intr_set kintr_set = { "\0", NULL, 0 };

#define UNSET_NOINTR_SHIELD	0
#define SET_NOINTR_SHIELD	1

static void
intr_shield_xcall(void *arg1, void *arg2)
{
	struct cpu_info *ci;
	struct schedstate_percpu *spc;
	int s, shield;

	ci = arg1;
	shield = (int)(intptr_t)arg2;
	spc = &ci->ci_schedstate;

	s = splsched();
	if (shield == UNSET_NOINTR_SHIELD)
		spc->spc_flags &= ~SPCF_NOINTR;
	else if (shield == SET_NOINTR_SHIELD)
		spc->spc_flags |= SPCF_NOINTR;
	splx(s);
}

/*
 * Change SPCF_NOINTR flag of schedstate_percpu->spc_flags
 */
static int
intr_shield(u_int cpu_idx, int shield)
{
	struct cpu_info *ci;
	struct schedstate_percpu *spc;

	ci = cpu_lookup(cpu_idx);
	if (ci == NULL)
		return EINVAL;

	spc = &ci->ci_schedstate;
	if (shield == UNSET_NOINTR_SHIELD) {
		if ((spc->spc_flags & SPCF_NOINTR) == 0)
			return 0;
	} else if (shield == SET_NOINTR_SHIELD) {
		if ((spc->spc_flags & SPCF_NOINTR) != 0)
			return 0;
	}

	if (ci == curcpu() || !mp_online) {
		intr_shield_xcall(ci, (void *)(intptr_t)shield);
	} else {
		uint64_t where;
		where = xc_unicast(0, intr_shield_xcall, ci,
			(void *)(intptr_t)shield, ci);
		xc_wait(where);
	}

	spc->spc_lastmod = time_second;
	return 0;
}

/*
 * Move all assigned interrupts from "cpu_idx" to the other cpu as possible.
 * The destination cpu is lowest cpuid of available cpus.
 * If there are no available cpus, give up to move interrupts.
 */
static int
intr_avert_intr(u_int cpu_idx)
{
	kcpuset_t *cpuset;
	void *ich;
	char **ids;
	int error, i, nids;

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
		DPRINTF(("%s: no available cpu\n", __func__));
		return ENOENT;
	}

	for (i = 0; i < nids; i++) {
		ich = intr_get_handler(ids[i]);
		KASSERT(ich != NULL);
		error = pci_intr_distribute(ich, cpuset, NULL);
		if (error)
			break;
	}

	intr_destruct_intrids(ids, nids);
	kcpuset_destroy(cpuset);
	return error;
}

/*
 * Return actual intr_list_line size. intr_list_line size is variable by ncpu.
 */
static size_t
intr_list_line_size(void)
{

	return sizeof(struct intr_list_line) +
		sizeof(struct intr_list_line_cpu) * (ncpu - 1);
}

static size_t
intr_list_size(void)
{
	size_t ilsize;
	int error, nids;
	char **ids;

	ilsize = 0;

	/* buffer header */
	ilsize += sizeof(struct intr_list);

	/* il_line body */
	mutex_enter(&cpu_lock);
	error = intr_construct_intrids(kcpuset_running, &ids, &nids);
	mutex_exit(&cpu_lock);
	if (error)
		return 0;
	ilsize += intr_list_line_size() * nids;

	intr_destruct_intrids(ids, nids);
	return ilsize;
}

/*
 * Set intrctl list to "data", and return list structure bytes.
 * If error occured, return <0.
 * If "data" == NULL, simply return list structure bytes.
 */
static int
intr_list(void *data, int length)
{
	struct intr_list *il;
	struct intr_list_line *illine;
	kcpuset_t *assigned, *avail;
	size_t ilsize;
	u_int cpu_idx;
	int nids, intr_idx, ret, line_size;
	char **ids;

	ilsize = intr_list_size();
	if (ilsize == 0)
		return -ENOMEM;

	if (data == NULL)
		return ilsize;

	if (length < ilsize)
		return -ENOMEM;

	il = (struct intr_list *)data;

	illine = (struct intr_list_line *)((char *)il + sizeof(struct intr_list));
	il->il_lineoffset = (off_t)illine - (off_t)il;

	kcpuset_create(&avail, true);
	intr_get_available(avail);
	kcpuset_create(&assigned, true);

	mutex_enter(&cpu_lock);

	ret = intr_construct_intrids(kcpuset_running, &ids, &nids);
	if (ret != 0) {
		mutex_exit(&cpu_lock);
		DPRINTF(("%s: intr_construct_intrids() failed\n", __func__));
		ret = -ret;
		goto out;
	}

	line_size = intr_list_line_size();
	/* ensure interrupts are not added after above intr_list_size(). */
	if (ilsize < sizeof(struct intr_list) + line_size * nids) {
		mutex_exit(&cpu_lock);
		intr_destruct_intrids(ids, nids);
		DPRINTF(("%s: interrupts are added during execution.\n", __func__));
		ret = -ENOMEM;
		goto out;
	}

	for (intr_idx = 0; intr_idx < nids; intr_idx++) {
		void *ich;

		ich = intr_get_handler(ids[intr_idx]);
		KASSERT(ich != NULL);
		strncpy(illine->ill_intrid, ids[intr_idx], INTRIDBUF);
		strncpy(illine->ill_xname, intr_get_devname(ich), INTRDEVNAMEBUF);

		intr_get_assigned(ich, assigned);
		for (cpu_idx = 0; cpu_idx < ncpu; cpu_idx++) {
			struct intr_list_line_cpu *illcpu =
				&illine->ill_cpu[cpu_idx];

			illcpu->illc_assigned =
				kcpuset_isset(assigned, cpu_idx) ? true : false;
			illcpu->illc_count = intr_get_count(ich, cpu_idx);
		}

		illine = (struct intr_list_line *)((char *)illine + line_size);
	}

	mutex_exit(&cpu_lock);

	ret = ilsize;
	il->il_version = INTR_LIST_VERSION;
	il->il_ncpus = ncpu;
	il->il_nintrs = nids;
	il->il_linesize = line_size;
	il->il_bufsize = ilsize;

	intr_destruct_intrids(ids, nids);
 out:
	kcpuset_destroy(assigned);
	kcpuset_destroy(avail);

	return ret;
}

/*
 * "intrctl list" entry
 */
static int
intr_list_sysctl(SYSCTLFN_ARGS)
{
	int ret;
	void *buf;

	if (oldlenp == NULL)
		return EINVAL;

	if (oldp == NULL) {
		ret = intr_list(NULL, 0);
		if (ret < 0)
			return -ret;

		*oldlenp = ret;
		return 0;
	}

	if (*oldlenp == 0)
		return ENOMEM;

	buf = kmem_zalloc(*oldlenp, KM_SLEEP);
	if (buf == NULL)
		return ENOMEM;

	ret = intr_list(buf, *oldlenp);
	if (ret < 0)
		return -ret;

	return copyout(buf, oldp, *oldlenp);
}

/*
 * "intrctl affinity" entry
 */
static int
intr_set_affinity_sysctl(SYSCTLFN_ARGS)
{
	struct sysctlnode node;
	struct intr_set *iset;
	cpuset_t *ucpuset;
	kcpuset_t *kcpuset;
	int error;
	void *ich;

	error = kauth_authorize_system(l->l_cred, KAUTH_SYSTEM_INTR,
	    KAUTH_REQ_SYSTEM_INTR_AFFINITY, NULL, NULL, NULL);
	if (error)
		return EPERM;

	node = *rnode;
	iset = (struct intr_set *)node.sysctl_data;

	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error != 0 || newp == NULL)
		return error;

	ucpuset = iset->cpuset;
	kcpuset_create(&kcpuset, true);
	kcpuset_copyin(ucpuset, kcpuset, iset->cpuset_size);
	if (kcpuset_iszero(kcpuset)) {
		kcpuset_destroy(kcpuset);
		return EINVAL;
	}

	mutex_enter(&cpu_lock);
	ich = intr_get_handler(iset->intrid);
	if (ich == NULL) {
		mutex_exit(&cpu_lock);
		kcpuset_destroy(kcpuset);
		return EINVAL;
	}
	error = pci_intr_distribute(ich, kcpuset, NULL);
	mutex_exit(&cpu_lock);

	kcpuset_destroy(kcpuset);
	return error;
}

/*
 * "intrctl intr" entry
 */
static int
intr_intr_sysctl(SYSCTLFN_ARGS)
{
	struct sysctlnode node;
	struct intr_set *iset;
	cpuset_t *ucpuset;
	kcpuset_t *kcpuset;
	int error;
	u_int cpu_idx;

	error = kauth_authorize_system(l->l_cred, KAUTH_SYSTEM_CPU,
	    KAUTH_REQ_SYSTEM_CPU_SETSTATE, NULL, NULL, NULL);
	if (error)
		return EPERM;

	node = *rnode;
	iset = (struct intr_set *)node.sysctl_data;

	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error != 0 || newp == NULL)
		return error;

	ucpuset = iset->cpuset;
	kcpuset_create(&kcpuset, true);
	kcpuset_copyin(ucpuset, kcpuset, iset->cpuset_size);
	if (kcpuset_iszero(kcpuset)) {
		kcpuset_destroy(kcpuset);
		return EINVAL;
	}

	cpu_idx = kcpuset_ffs(kcpuset) - 1; /* support one CPU only */

	mutex_enter(&cpu_lock);
	error = intr_shield(cpu_idx, UNSET_NOINTR_SHIELD);
	mutex_exit(&cpu_lock);

	kcpuset_destroy(kcpuset);
	return error;
}

/*
 * "intrctl nointr" entry
 */
static int
intr_nointr_sysctl(SYSCTLFN_ARGS)
{
	struct sysctlnode node;
	struct intr_set *iset;
	cpuset_t *ucpuset;
	kcpuset_t *kcpuset;
	int error;
	u_int cpu_idx;

	error = kauth_authorize_system(l->l_cred, KAUTH_SYSTEM_CPU,
	    KAUTH_REQ_SYSTEM_CPU_SETSTATE, NULL, NULL, NULL);
	if (error)
		return EPERM;

	node = *rnode;
	iset = (struct intr_set *)node.sysctl_data;

	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error != 0 || newp == NULL)
		return error;

	ucpuset = iset->cpuset;
	kcpuset_create(&kcpuset, true);
	kcpuset_copyin(ucpuset, kcpuset, iset->cpuset_size);
	if (kcpuset_iszero(kcpuset)) {
		kcpuset_destroy(kcpuset);
		return EINVAL;
	}

	cpu_idx = kcpuset_ffs(kcpuset) - 1; /* support one CPU only */

	mutex_enter(&cpu_lock);
	error = intr_shield(cpu_idx, SET_NOINTR_SHIELD);
	if (error) {
		mutex_exit(&cpu_lock);
		return error;
	}
	error = intr_avert_intr(cpu_idx);
	mutex_exit(&cpu_lock);

	kcpuset_destroy(kcpuset);
	return error;
}

SYSCTL_SETUP(sysctl_intr_setup, "sysctl intr setup")
{
	const struct sysctlnode *node = NULL;

	sysctl_createv(clog, 0, NULL, &node,
		       CTLFLAG_PERMANENT, CTLTYPE_NODE,
		       "intr", SYSCTL_DESCR("Interrupt options"),
		       NULL, 0, NULL, 0,
		       CTL_KERN, CTL_CREATE, CTL_EOL);

	sysctl_createv(clog, 0, &node, NULL,
		       CTLFLAG_PERMANENT, CTLTYPE_STRUCT,
 		       "list", SYSCTL_DESCR("intrctl list"),
		       intr_list_sysctl, 0, NULL, 0,
		       CTL_CREATE, CTL_EOL);

	sysctl_createv(clog, 0, &node, NULL,
		       CTLFLAG_PERMANENT|CTLFLAG_READWRITE, CTLTYPE_STRUCT,
		       "affinity", SYSCTL_DESCR("set affinity"),
		       intr_set_affinity_sysctl, 0, &kintr_set, sizeof(kintr_set),
		       CTL_CREATE, CTL_EOL);

	sysctl_createv(clog, 0, &node, NULL,
 		       CTLFLAG_PERMANENT|CTLFLAG_READWRITE, CTLTYPE_STRUCT,
 		       "intr", SYSCTL_DESCR("set intr"),
		       intr_intr_sysctl, 0, &kintr_set, sizeof(kintr_set),
		       CTL_CREATE, CTL_EOL);

	sysctl_createv(clog, 0, &node, NULL,
		       CTLFLAG_PERMANENT|CTLFLAG_READWRITE, CTLTYPE_STRUCT,
		       "nointr", SYSCTL_DESCR("set nointr"),
		       intr_nointr_sysctl, 0, &kintr_set, sizeof(kintr_set),
		       CTL_CREATE, CTL_EOL);
}
