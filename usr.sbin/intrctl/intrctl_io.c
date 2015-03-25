/*	$NetBSD$	*/

/*
 * Copyright (c) 2015 Internet Initiative Japan Inc.
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
__RCSID("$NetBSD$");

#include <sys/sysctl.h>
#include <sys/intrio.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include "intrctl_io.h"

/*
 * To support increasing the number of interrupts (devices are dynamically
 * attached), retry sysctl(3) "retry" times.
 */
void *
intrctl_io_alloc(int retry)
{
	int i, error;
	size_t buf_size;
	void *buf;

	error = sysctlbyname("kern.intr.list", NULL, &buf_size, NULL, 0);
	if (error < 0) {
		fprintf(stderr, "sysctl kern.intr.list listsize");
		return NULL;
	}

	buf = malloc(buf_size);
	if (buf == NULL) {
		fprintf(stderr, "malloc(buf_size)");
		return NULL;
	}

	for (i = 0; i < retry; i++) {
		error = sysctlbyname("kern.intr.list", buf, &buf_size, NULL, 0);
		if (error >= 0)
			return buf;
		else if (error == -ENOMEM) {
			void *temp;

			temp = realloc(buf, buf_size);
			if (temp == NULL) {
				free(buf);
				return NULL;
			}
			buf = temp;
		}
		else {
			free(buf);
			fprintf(stderr, "sysctl kern.intr.list list");
			return NULL;
		}
	}
	return NULL;
}

void
intrctl_io_free(void *handle)
{

	free(handle);
}

int
intrctl_io_ncpus(void *handle)
{
	struct intr_list *list = handle;

	return list->il_ncpus;
}

int
intrctl_io_nintrs(void *handle)
{
	struct intr_list *list = handle;

	return list->il_nintrs;
}

struct intr_list_line *
intrctl_io_firstline(void *handle)
{
	struct intr_list *list = handle;

	return (struct intr_list_line *)((char *)list + list->il_lineoffset);
}

struct intr_list_line *
intrctl_io_nextline(void *handle, struct intr_list_line *cur)
{
	size_t line_size;
	char *buf_end;
	struct intr_list_line *next;
	struct intr_list *list = handle;

	buf_end = (char *)list + list->il_bufsize;

	line_size = list->il_linesize;
	next = (struct intr_list_line *)((char *)cur + line_size);
	if ((char *)next >= buf_end)
		return NULL;

	return next;
}
