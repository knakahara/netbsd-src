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

#ifndef _SYS_INTRIO_H_
#define _SYS_INTRIO_H_

#include <sys/ioccom.h>
#include <sys/types.h>

#ifndef _KERNEL
#include <sched.h>
#endif

#define INTR_LIST_BUFSIZE 4096

#define INTRID_LEN 47 /* should use max size of interrupt name of supporting
		       * architectures.
		       *     - x86
		       *       sizeof(struct device.dv_xname) +
		       *           sizeof(struct intrsource.is_evname) - 1
		       *       see intr_string()
		       */

struct intr_set {
	char intrid[INTRID_LEN + 1];
	cpuset_t *cpuset;
	size_t cpuset_size;
};

#define	IOC_INTR_LIST		_IOR('c', 0, char[INTR_LIST_BUFSIZE])
#define	IOC_INTR_AFFINITY	_IOW('c', 1, struct intr_set)
#define	IOC_INTR_INTR		_IOW('c', 2, struct intr_set)
#define	IOC_INTR_NOINTR		_IOW('c', 3, struct intr_set)

#endif /* !_SYS_INTRIO_H_ */
