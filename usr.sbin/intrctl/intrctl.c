
#include <sys/ioctl.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

__dead static void	usage(void);

static int	fd;
int		verbose;

static void	intr_list(int, char **);
static void	intr_set(int, char **);
static void	intr_intr(int, char **);
static void	intr_nointr(int, char **);

static struct cmdtab {
	const char	*label;
	void	(*func)(int, char **);
} const intr_cmdtab[] = {
	{ "list", intr_list },
	{ "set", intr_set },
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
	fprintf(stderr, "       %s set -i irq -c cpuno\n", progname);
	fprintf(stderr, "       %s intr -c cpuno\n", progname);
	fprintf(stderr, "       %s nointr -c cpuno\n", progname);
	exit(EXIT_FAILURE);
	/* NOTREACHED */
}

static void
intr_list(int argc, char **argv)
{
}

static void
intr_set(int argc, char **argv)
{
}

static void
intr_intr(int argc, char **argv)
{
}

static void
intr_nointr(int argc, char **argv)
{
}

