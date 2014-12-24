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
__RCSID("$NetBSD$");

#include <sys/ioctl.h>
#include <sys/intrio.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <paths.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

__dead static void	usage(void);

static int	fd;
int		verbose;

static void	intr_list(int, char **);
static void	intr_affinity(int, char **);
static void	intr_intr(int, char **);
static void	intr_nointr(int, char **);

static struct cmdtab {
	const char	*label;
	void	(*func)(int, char **);
} const intr_cmdtab[] = {
	{ "list", intr_list },
	{ "affinity", intr_affinity },
	{ "intr", intr_intr },
	{ "nointr", intr_nointr },
	{ NULL, NULL },
};

int
main(int argc, char **argv)
{
	const struct cmdtab *ct;
	char *cmdname;

	if (argc < 2)
		usage();

	cmdname = argv[1];
	argv += 1;
	argc -= 1;

	for (ct = intr_cmdtab; ct->label != NULL; ct++) {
		if (strcmp(cmdname, ct->label) == 0) {
			break;
		}
	}

	if (ct->label == NULL)
		errx(EXIT_FAILURE, "unknown command ``%s''", cmdname);

	if ((fd = open(_PATH_INTRCTL, O_RDWR)) < 0)
		err(EXIT_FAILURE, _PATH_INTRCTL);
	(*ct->func)(argc, argv);
	close(fd);
	exit(EXIT_SUCCESS);
	/* NOTREACHED */
}

static void
usage(void)
{
	const char *progname = getprogname();

	fprintf(stderr, "usage: %s list\n", progname);
	fprintf(stderr, "       %s affinity -i interrupt_name -c cpu_index\n", progname);
	fprintf(stderr, "       %s intr -c cpu_index\n", progname);
	fprintf(stderr, "       %s nointr -c cpu_index\n", progname);
	exit(EXIT_FAILURE);
	/* NOTREACHED */
}

static void
intr_list(int argc, char **argv)
{
	char *buf, *line, *cur, *end;
	int error;

	buf = malloc(INTR_LIST_BUFSIZE);
	if (buf == NULL)
		err(EXIT_FAILURE, "malloc(buf)");

	error = ioctl(fd, IOC_INTR_LIST_HEADER, buf);
	if (error < 0)
		err(EXIT_FAILURE, "IOC_INTR_LIST_HEADER");
	printf("%s", buf);

	memset(buf, '\0', INTR_LIST_BUFSIZE);
	error = ioctl(fd, IOC_INTR_INTRIDS, buf);
	if (error < 0)
		err(EXIT_FAILURE, "IOC_INTR_INTRIDS");

	line = malloc(INTR_LIST_BUFSIZE);
	if (line == NULL)
		err(EXIT_FAILURE, "malloc(line)");

	cur = buf;
	for (;;) {
		end = strchr(cur, '\n');
		if (end == NULL)
			break;
		*(end++) = '\0';

		memset(line, '\0', INTR_LIST_BUFSIZE);
		strncpy(line, cur, INTRID_LEN);
		error = ioctl(fd, IOC_INTR_LIST_DATA, line);
		if (error < 0)
			err(EXIT_FAILURE, "IOC_INTR_LIST_DATA");

		printf("%s", line);
		cur = end;
	}

	free(line);
	free(buf);
}

static void
intr_affinity(int argc, char **argv)
{
	struct intr_set iset;
	cpuset_t *cpuset;
	unsigned long index;
	int ch;
	int error;

	index = ULONG_MAX;
	memset(&iset.intrid, 0, sizeof(iset.intrid));

	while ((ch = getopt(argc, argv, "c:i:")) != -1) {
		switch (ch) {
		case 'c':
			index = strtoul(optarg, NULL, 10);
			break;
		case 'i':
			if (strnlen(optarg, ARG_MAX) > INTRID_LEN)
				usage();
			strncpy(iset.intrid, optarg, INTRID_LEN);
			break;
		default:
			usage();
		}
	}

	if (iset.intrid[0] == '\0' || index == ULONG_MAX)
		usage();

	if (index >= (u_long)sysconf(_SC_NPROCESSORS_CONF))
		err(EXIT_FAILURE, "invalid cpu index");

	cpuset = cpuset_create();
	if (cpuset == NULL)
		err(EXIT_FAILURE, "create_cpuset()");

	cpuset_zero(cpuset);
	cpuset_set(index, cpuset);
	iset.cpuset = cpuset;
	iset.cpuset_size = cpuset_size(cpuset);
	error = ioctl(fd, IOC_INTR_AFFINITY, &iset);
	cpuset_destroy(cpuset);
	if (error < 0)
		err(EXIT_FAILURE, "IOC_INTR_AFFINITY");
}

static void
intr_intr(int argc, char **argv)
{
	struct intr_set iset;
	cpuset_t *cpuset;
	unsigned long index;
	int ch;
	int error;

	index = ULONG_MAX;

	while ((ch = getopt(argc, argv, "c:")) != -1) {
		switch (ch) {
		case 'c':
			index = strtoul(optarg, NULL, 10);
			break;
		default:
			usage();
		}
	}

	if (index == ULONG_MAX)
		usage();

	if (index >= (u_long)sysconf(_SC_NPROCESSORS_CONF))
		err(EXIT_FAILURE, "invalid cpu index");

	cpuset = cpuset_create();
	if (cpuset == NULL)
		err(EXIT_FAILURE, "create_cpuset()");

	cpuset_zero(cpuset);
	cpuset_set(index, cpuset);
	iset.cpuset = cpuset;
	iset.cpuset_size = cpuset_size(cpuset);
	error = ioctl(fd, IOC_INTR_INTR, &iset);
	cpuset_destroy(cpuset);
	if (error < 0)
		err(EXIT_FAILURE, "IOC_INTR_INTR");
}

static void
intr_nointr(int argc, char **argv)
{
	struct intr_set iset;
	cpuset_t *cpuset;
	unsigned long index;
	int ch;
	int error;

	index = ULONG_MAX;

	while ((ch = getopt(argc, argv, "c:")) != -1) {
		switch (ch) {
		case 'c':
			index = strtoul(optarg, NULL, 10);
			break;
		default:
			usage();
		}
	}

	if (index == ULONG_MAX)
		usage();

	if (index >= (u_long)sysconf(_SC_NPROCESSORS_CONF))
		err(EXIT_FAILURE, "invalid cpu index");

	cpuset = cpuset_create();
	if (cpuset == NULL)
		err(EXIT_FAILURE, "create_cpuset()");

	cpuset_zero(cpuset);
	cpuset_set(index, cpuset);
	iset.cpuset = cpuset;
	iset.cpuset_size = cpuset_size(cpuset);
	error = ioctl(fd, IOC_INTR_NOINTR, &iset);
	cpuset_destroy(cpuset);
	if (error < 0) {
		err(EXIT_FAILURE, "IOC_INTR_NOINTR");
	}
}
