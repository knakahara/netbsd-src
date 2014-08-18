
#ifndef _SYS_INTRIO_H_
#define _SYS_INTRIO_H_

#include <sys/ioccom.h>
#include <sys/types.h>

#define INTR_LIST_BUFSIZE 4096

struct intr_set {
	int irq;
	cpuid_t cpuid;
};

#define	IOC_INTR_LIST	_IOR('c', 0, char[INTR_LIST_BUFSIZE])
#define	IOC_INTR_SET	_IOW('c', 1, struct intr_set)
#define	IOC_INTR_INTR	_IOW('c', 2, cpuid_t)
#define	IOC_INTR_NOINTR	_IOW('c', 3, cpuid_t)

#endif /* !_SYS_INTRIO_H_ */
