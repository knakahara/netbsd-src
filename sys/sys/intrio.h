
#ifndef _SYS_INTRIO_H_
#define _SYS_INTRIO_H_

#include <sys/ioccom.h>

#define INTR_LIST_BUFSIZE 4096

#define	IOC_INTR_LIST	_IOR('c', 0, char[INTR_LIST_BUFSIZE])

#endif /* !_SYS_INTRIO_H_ */
