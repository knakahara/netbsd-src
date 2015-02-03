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
	int nids;
	int error;
	int i;

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

/*
 * Set intrctl list to "data", and return list bytes (include '\0').
 * If error occured, return <0.
 * If "data" == NULL, simply return list bytes.
 */
#define INTR_LIST_LINE 1024
static int
intr_list(char *data, int length)
{
	CPU_INFO_ITERATOR cii;
	struct cpu_info *ci;
	void *ich;
	kcpuset_t *assigned, *avail;
	char *buf, *cur, *end;
	char **ids;
	uint64_t intr_count;
	u_int cpu_idx;
	int nids, intr_idx, ret;
	int bufsize = 0;
	bool sizeonly;

	if (data == NULL) {
		buf = kmem_zalloc(INTR_LIST_LINE, KM_SLEEP);
		if (buf == NULL)
			return -ENOMEM;
		length = INTR_LIST_LINE;
		sizeonly = true;
	} else {
		if (length <= 0)
			return -EINVAL;
		buf = data;
		sizeonly = false;
	}

	cur = buf;
	end = buf + length;

#define FILL_BUF(_label, _fmt, ...) do{				\
		ret = snprintf(cur, end - cur, _fmt, ## __VA_ARGS__);    \
		if (ret < 0) {                                          \
			ret = -EIO;					\
			goto _label;					\
		}                                                       \
		bufsize += ret;						\
		if (!sizeonly) {					\
			if (cur + ret >= end) {				\
				ret =  -ENOBUFS;			\
				goto _label;				\
			}						\
			cur += ret;					\
		}							\
	}while(0)


	FILL_BUF(out0, " interrupt name");

	kcpuset_create(&avail, true);
	intr_get_available(avail);
	for (CPU_INFO_FOREACH(cii, ci)) {
		char intr_enable;
		if (kcpuset_isset(avail, cpu_index(ci)))
			intr_enable = '+';
		else
			intr_enable = '-';

		FILL_BUF(out1, "\tCPU#%02u(%c)", cpu_index(ci), intr_enable);
	}
	FILL_BUF(out1, "\n");

	mutex_enter(&cpu_lock);
	ret = intr_construct_intrids(kcpuset_running, &ids, &nids);
	if (ret != 0) {
		DPRINTF(("%s: intr_construct_intrids() failed\n", __func__));
		ret = -ret;
		goto out2;
	}

	kcpuset_create(&assigned, true);
	for (intr_idx = 0; intr_idx < nids; intr_idx++) {
		FILL_BUF(out3, " %s", ids[intr_idx]);

		ich = intr_get_handler(ids[intr_idx]);
		intr_get_assigned(ich, assigned);
		for (cpu_idx = 0; cpu_idx < ncpuonline; cpu_idx++) {
			intr_count = intr_get_count(ich, cpu_idx);
			if (kcpuset_isset(assigned, cpu_idx))
				FILL_BUF(out3, "\t%8" PRIu64 "*", intr_count);
			else
				FILL_BUF(out3, "\t%8" PRIu64, intr_count);
		}

		FILL_BUF(out3, "\t%s\n", intr_get_devname(ich));
	}

	ret = bufsize + 1; /* add '\0' */

 out3:
	kcpuset_destroy(assigned);
	intr_destruct_intrids(ids, nids);
 out2:
	mutex_exit(&cpu_lock);
 out1:
	kcpuset_destroy(avail);
	if (sizeonly) {
		kmem_free(buf, INTR_LIST_LINE);
	}
 out0:
	return ret;

#undef FULL_BUF
}

/*
 * "intrctl list" entry
 */
static int
intr_list_sysctl(SYSCTLFN_ARGS)
{
	int listsize;
	int ret;
	char *buf;

	if (oldp == NULL) {
		listsize = intr_list(NULL, 0);
		if (listsize < 0)
			return EINVAL;

		if (oldlenp == NULL)
			return EINVAL;

		*oldlenp = listsize;
		return 0;
	}

	if (*oldlenp == 0)
		return EINVAL;

	buf = kmem_zalloc(*oldlenp, KM_SLEEP);
	if (buf == NULL)
		return ENOMEM;

	ret = intr_list(buf, *oldlenp);
	if (ret < 0)
		return -ret;

	return copyoutstr(buf, oldp, *oldlenp, NULL);
}

/*
 * "intrctl affinity" entry
 */
static int
intr_set_affinity_sysctl(SYSCTLFN_ARGS)
{
	struct sysctlnode node;
	int error;
	struct intr_set *iset;
	cpuset_t *ucpuset;
	kcpuset_t *kcpuset;
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
	error = intr_distribute(ich, kcpuset, NULL);
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
	int error;
	u_int cpu_idx;
	struct intr_set *iset;
	cpuset_t *ucpuset;
	kcpuset_t *kcpuset;

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
	int error;
	u_int cpu_idx;
	struct intr_set *iset;
	cpuset_t *ucpuset;
	kcpuset_t *kcpuset;

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
