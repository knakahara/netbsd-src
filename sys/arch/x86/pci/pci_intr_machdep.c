/*	$NetBSD: pci_intr_machdep.c,v 1.27 2014/03/29 19:28:30 christos Exp $	*/

/*-
 * Copyright (c) 1997, 1998, 2009 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe of the Numerical Aerospace Simulation Facility,
 * NASA Ames Research Center.
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

/*
 * Copyright (c) 1996 Christopher G. Demetriou.  All rights reserved.
 * Copyright (c) 1994 Charles M. Hannum.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Charles M. Hannum.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Machine-specific functions for PCI autoconfiguration.
 *
 * On PCs, there are two methods of generating PCI configuration cycles.
 * We try to detect the appropriate mechanism for this machine and set
 * up a few function pointers to access the correct method directly.
 *
 * The configuration method can be hard-coded in the config file by
 * using `options PCI_CONF_MODE=N', where `N' is the configuration mode
 * as defined section 3.6.4.1, `Generating Configuration Cycles'.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: pci_intr_machdep.c,v 1.27 2014/03/29 19:28:30 christos Exp $");

#include <sys/types.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/malloc.h>

#include <dev/pci/pcivar.h>

#include "ioapic.h"
#include "eisa.h"
#include "acpica.h"
#include "opt_mpbios.h"
#include "opt_acpi.h"

#include <machine/i82489reg.h>

#if NIOAPIC > 0 || NACPICA > 0
#include <machine/i82093reg.h>
#include <machine/i82093var.h>
#include <machine/mpconfig.h>
#include <machine/mpbiosvar.h>
#include <machine/pic.h>
#endif

#ifdef MPBIOS
#include <machine/mpbiosvar.h>
#endif

#if NACPICA > 0
#include <machine/mpacpi.h>
#endif

#define	MPSAFE_MASK	0x80000000

int
pci_intr_map(const struct pci_attach_args *pa, pci_intr_handle_t *ihp)
{
	int pin = pa->pa_intrpin;
	int line = pa->pa_intrline;
	pci_chipset_tag_t ipc, pc = pa->pa_pc;
#if NIOAPIC > 0 || NACPICA > 0
	int rawpin = pa->pa_rawintrpin;
	int bus, dev, func;
#endif

	for (ipc = pc; ipc != NULL; ipc = ipc->pc_super) {
		if ((ipc->pc_present & PCI_OVERRIDE_INTR_MAP) == 0)
			continue;
		return (*ipc->pc_ov->ov_intr_map)(ipc->pc_ctx, pa, ihp);
	}

	if (pin == 0) {
		/* No IRQ used. */
		goto bad;
	}

	*ihp = 0;

	if (pin > PCI_INTERRUPT_PIN_MAX) {
		aprint_normal("pci_intr_map: bad interrupt pin %d\n", pin);
		goto bad;
	}

#if NIOAPIC > 0 || NACPICA > 0
	KASSERT(rawpin >= PCI_INTERRUPT_PIN_A);
	KASSERT(rawpin <= PCI_INTERRUPT_PIN_D);
	pci_decompose_tag(pc, pa->pa_tag, &bus, &dev, &func);
	if (mp_busses != NULL) {
		/*
		 * Note: PCI_INTERRUPT_PIN_A == 1 where intr_find_mpmapping
		 * wants pci bus_pin encoding which uses INT_A == 0.
		 */
		if (intr_find_mpmapping(bus,
		    (dev << 2) | (rawpin - PCI_INTERRUPT_PIN_A), ihp) == 0) {
			if (APIC_IRQ_LEGACY_IRQ(*ihp) == 0)
				*ihp |= line;
			return 0;
		}
		/*
		 * No explicit PCI mapping found. This is not fatal,
		 * we'll try the ISA (or possibly EISA) mappings next.
		 */
	}
#endif

	/*
	 * Section 6.2.4, `Miscellaneous Functions', says that 255 means
	 * `unknown' or `no connection' on a PC.  We assume that a device with
	 * `no connection' either doesn't have an interrupt (in which case the
	 * pin number should be 0, and would have been noticed above), or
	 * wasn't configured by the BIOS (in which case we punt, since there's
	 * no real way we can know how the interrupt lines are mapped in the
	 * hardware).
	 *
	 * XXX
	 * Since IRQ 0 is only used by the clock, and we can't actually be sure
	 * that the BIOS did its job, we also recognize that as meaning that
	 * the BIOS has not configured the device.
	 */
	if (line == 0 || line == X86_PCI_INTERRUPT_LINE_NO_CONNECTION) {
		aprint_normal("pci_intr_map: no mapping for pin %c (line=%02x)\n",
		       '@' + pin, line);
		goto bad;
	} else {
		if (line >= NUM_LEGACY_IRQS) {
			aprint_normal("pci_intr_map: bad interrupt line %d\n", line);
			goto bad;
		}
		if (line == 2) {
			aprint_normal("pci_intr_map: changed line 2 to line 9\n");
			line = 9;
		}
	}
#if NIOAPIC > 0 || NACPICA > 0
	if (mp_busses != NULL) {
		if (intr_find_mpmapping(mp_isa_bus, line, ihp) == 0) {
			if ((*ihp & 0xff) == 0)
				*ihp |= line;
			return 0;
		}
#if NEISA > 0
		if (intr_find_mpmapping(mp_eisa_bus, line, ihp) == 0) {
			if ((*ihp & 0xff) == 0)
				*ihp |= line;
			return 0;
		}
#endif
		aprint_normal("pci_intr_map: bus %d dev %d func %d pin %d; line %d\n",
		    bus, dev, func, pin, line);
		aprint_normal("pci_intr_map: no MP mapping found\n");
	}
#endif

	*ihp = line;
	return 0;

bad:
	*ihp = -1;
	return 1;
}

const char *
pci_intr_string(pci_chipset_tag_t pc, pci_intr_handle_t ih, char *buf,
    size_t len)
{
	pci_chipset_tag_t ipc;

	for (ipc = pc; ipc != NULL; ipc = ipc->pc_super) {
		if ((ipc->pc_present & PCI_OVERRIDE_INTR_STRING) == 0)
			continue;
		return (*ipc->pc_ov->ov_intr_string)(ipc->pc_ctx, pc, ih,
		    buf, len);
	}

	return intr_string(ih & ~MPSAFE_MASK, buf, len);
}


const struct evcnt *
pci_intr_evcnt(pci_chipset_tag_t pc, pci_intr_handle_t ih)
{
	pci_chipset_tag_t ipc;

	for (ipc = pc; ipc != NULL; ipc = ipc->pc_super) {
		if ((ipc->pc_present & PCI_OVERRIDE_INTR_EVCNT) == 0)
			continue;
		return (*ipc->pc_ov->ov_intr_evcnt)(ipc->pc_ctx, pc, ih);
	}

	/* XXX for now, no evcnt parent reported */
	return NULL;
}

int
pci_intr_setattr(pci_chipset_tag_t pc, pci_intr_handle_t *ih,
		 int attr, uint64_t data)
{

	switch (attr) {
	case PCI_INTR_MPSAFE:
		if (data) {
			 *ih |= MPSAFE_MASK;
		} else {
			 *ih &= ~MPSAFE_MASK;
		}
		/* XXX Set live if already mapped. */
		return 0;
	default:
		return ENODEV;
	}
}

void *
pci_intr_establish(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg)
{
	int pin, irq;
	struct pic *pic;
#if NIOAPIC > 0
	struct ioapic_softc *ioapic;
#endif
	bool mpsafe;
	pci_chipset_tag_t ipc;

	for (ipc = pc; ipc != NULL; ipc = ipc->pc_super) {
		if ((ipc->pc_present & PCI_OVERRIDE_INTR_ESTABLISH) == 0)
			continue;
		return (*ipc->pc_ov->ov_intr_establish)(ipc->pc_ctx,
		    pc, ih, level, func, arg);
	}

	pic = &i8259_pic;
	pin = irq = (ih & ~MPSAFE_MASK);
	mpsafe = ((ih & MPSAFE_MASK) != 0);

#if NIOAPIC > 0
	if (ih & APIC_INT_VIA_APIC) {
		ioapic = ioapic_find(APIC_IRQ_APIC(ih));
		if (ioapic == NULL) {
			aprint_normal("pci_intr_establish: bad ioapic %d\n",
			    APIC_IRQ_APIC(ih));
			return NULL;
		}
		pic = &ioapic->sc_pic;
		pin = APIC_IRQ_PIN(ih);
		irq = APIC_IRQ_LEGACY_IRQ(ih);
		if (irq < 0 || irq >= NUM_LEGACY_IRQS)
			irq = -1;
	}
#endif

	return intr_establish(irq, pic, pin, IST_LEVEL, level, func, arg,
	    mpsafe);
}

void
pci_intr_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	pci_chipset_tag_t ipc;

	for (ipc = pc; ipc != NULL; ipc = ipc->pc_super) {
		if ((ipc->pc_present & PCI_OVERRIDE_INTR_DISESTABLISH) == 0)
			continue;
		(*ipc->pc_ov->ov_intr_disestablish)(ipc->pc_ctx, pc, cookie);
		return;
	}

	intr_disestablish(cookie);
}

#if NIOAPIC > 0
/*
 * experimental support for MSI, does support a single vector,
 * no MSI-X, 8-bit APIC IDs
 * (while it doesn't need the ioapic technically, it borrows
 * from its kernel support)
 */

#if 0
/* dummies, needed by common intr_establish code */
static void
msipic_hwmask(struct pic *pic, int pin)
{
}
static void
msipic_addroute(struct pic *pic, struct cpu_info *ci,
		int pin, int vec, int type)
{
}

static struct pic msi_pic = {
	.pic_name = "msi",
	.pic_type = PIC_SOFT,
	.pic_vecbase = 0,
	.pic_apicid = 0,
	.pic_lock = __SIMPLELOCK_UNLOCKED,
	.pic_hwmask = msipic_hwmask,
	.pic_hwunmask = msipic_hwmask,
	.pic_addroute = msipic_addroute,
	.pic_delroute = msipic_addroute,
	.pic_edge_stubs = ioapic_edge_stubs,
};

struct msi_hdl {
	struct intrhand *ih;
	pci_chipset_tag_t pc;
	pcitag_t tag;
	int co;
};
#endif

/*
 * XXXX should separate source file about MSI
 */
static pcitag_t msi_pcitags[NUM_MSI_INTS];
static void
set_msi_pcitag(int vector, pcitag_t tag)
{
	msi_pcitags[vector - FIRST_MSI_INT] = tag;
}
static pcitag_t
get_msi_pcitag(int vector)
{
	return msi_pcitags[vector - FIRST_MSI_INT];
}

static void
msi_hwmask(struct pic *pic, int pin)
{
}

static void
msi_hwunmask(struct pic *pic, int pin)
{
}

static void
msi_addroute(struct pic *pic, struct cpu_info *ci,
	     int pin, int vec, int type)
{
	pci_chipset_tag_t pc = NULL;
	pcitag_t tag = get_msi_pcitag(pin);
	pcireg_t reg, addr, data, ctl;
	int off;

	if (pci_get_capability(pc, tag, PCI_CAP_MSI, &off, &reg) == 0)
		panic("%s: no msi capability", __func__);

	/*
	 * XXXX
	 * see OpenBSD's cpu_attach().
	 * OpenBSD's ci->ci_apicid is equal to NetBSD's ci_cpuid.
	 * What's mean OpenBSD's ci->ci_cpuid? the value is sc->sc_dev.dv_unit.
	 */
	addr = LAPIC_MSIADDR_BASE | __SHIFTIN(ci->ci_cpuid, LAPIC_MSIADDR_DSTID_MASK);
	/*if triger mode is edge, it don't care level for trigger mode. */
	data = __SHIFTIN(vec, LAPIC_MSIDATA_VECTOR_MASK) |
		LAPIC_MSIDATA_TRGMODE_EDGE | LAPIC_MSIDATA_DM_FIXED;

	if (reg & PCI_MSI_CTL_64BIT_ADDR) {
		pci_conf_write(pc, tag, off + PCI_MSI_MADDR64_LO, addr);
		pci_conf_write(pc, tag, off + PCI_MSI_MADDR64_HI, 0);
		pci_conf_write(pc, tag, off + PCI_MSI_MDATA64, data);
	} else {
		pci_conf_write(pc, tag, off + PCI_MSI_MADDR, addr);
		pci_conf_write(pc, tag, off + PCI_MSI_MDATA, data);
	}
	ctl = reg | PCI_MSI_CTL_MSI_ENABLE;
	pci_conf_write(pc, tag, off + PCI_MSI_CTL, ctl);
}

static void
msi_delroute(struct pic *pic, struct cpu_info *ci,
	     int pin, int vec, int type)
{
	pci_chipset_tag_t pc = NULL;
	pcitag_t tag = get_msi_pcitag(pin);
	pcireg_t reg;
	int off;

	if (pci_get_capability(pc, tag, PCI_CAP_MSI, &off, &reg) == 0)
		panic("%s: no msi capability", __func__);
	pci_conf_write(pc, tag, off, reg & ~PCI_MSI_CTL_MSI_ENABLE);
}

static struct pic msi_pic = {
	.pic_name = "msi",
	.pic_type = PIC_MSI,
	.pic_vecbase = 0,
	.pic_apicid = 0,
	.pic_lock = __SIMPLELOCK_UNLOCKED,
	.pic_hwmask = msi_hwmask,
	.pic_hwunmask = msi_hwunmask,
	.pic_addroute = msi_addroute,
	.pic_delroute = msi_delroute,
	.pic_edge_stubs = ioapic_edge_stubs,
};

/*
 * return number of the devices's MSI vectors
 * return 0 if the device does not support MSI
 */
int
pci_msi_count(struct pci_attach_args *pa)
{
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;

	int offset;
	pcireg_t reg;
	int mmc;

	if (pci_get_capability(pc, tag, PCI_CAP_MSI, &offset, NULL) == 0)
		return 0;

	reg = pci_conf_read(pc, tag, offset + PCI_MSI_CTL);
	mmc = PCI_MSI_CTL_MMC(reg);
	switch (mmc) {
	case 0x6:
	case 0x7:
		aprint_normal("the device use reserved MMC values.\n");
		return 0;
	default:
		return 1 << mmc;
	}
}

/* XXXX tentative function name */
/* XXXX define other file? */
static int
pci_msi_alloc_md(pci_intr_handle_t **ihps, int *count, struct pci_attach_args *pa)
{
	int *vectors;
	int i;

	vectors = intr_allocate_msi_vectors(count);
	if (vectors == NULL) {
		aprint_normal("cannot allocate MSI vectors.\n");
		return 1;
	}

	for (i = 0; i < *count; i++) {
		set_msi_pcitag(vectors[i], pa->pa_tag);
		vectors[i] |= APIC_INT_VIA_MSG;
	}

	*ihps = vectors;
	return 0;
}

/*
 * This function should be MI API.
 * This function is used by device drivers like pci_intr_map().
 *
 * "ihps" is the array  of vector numbers which MSI used instead of IRQ number.
 * "count" must be powr of 2.
 * "count" can decrease if sturct intrsource cannot be allocated.
 * if count == 0, return non-zero value.
 */
int
pci_msi_alloc(struct pci_attach_args *pa, pci_intr_handle_t **ihps, int *count)
{
	int hw_max;

	if (*count < 1) {
		aprint_normal("invalid count: %d\n", *count);
		return 1;
	}
	if (((*count - 1) & *count) != 0) {
		aprint_normal("count %d must be power of 2.\n", *count);
		return 1;
	}

	hw_max = pci_msi_count(pa);
	if (hw_max == 0)
		return 1;

	if (*count > hw_max) {
		aprint_normal("cut off MSI count to %d\n", hw_max);
		*count = hw_max; /* cut off hw_max */
	}

	return pci_msi_alloc_md(ihps, count, pa);
}

static void
pci_msi_release_md(void **cookies, int count)
{
	int *vectors;

	vectors = *cookies;
	return intr_free_msi_vectors(vectors, count);
}

/* XXXX tentative function name */
/* XXXX define other file? */
void
pci_msi_release(void **cookie, int count)
{
	if (count < 1) {
		aprint_normal("invalid count: %d\n", count);
		return;
	}

	return pci_msi_release_md(cookie, count);
}

void *
pci_msi_establish(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg)
{
	int pin, irq;
	struct pic *pic;
	bool mpsafe;

	irq = -1;
	pic = &msi_pic;
	pin = ih & (~MPSAFE_MASK); /* hint to search pcitag_t */
	mpsafe = ((ih & MPSAFE_MASK) != 0);

	if ((ih & APIC_INT_VIA_MSG) == 0) {
		aprint_normal("invalid MSI ih: 0x%x\n", ih);
		return NULL;
	}

	return intr_establish(irq, pic, pin, IST_EDGE, level, func, arg,
	    mpsafe);
}

void
pci_msi_disestablish(pci_chipset_tag_t pc, void *cookie)
{
}

#if 0
void *
pci_msi_establish(struct pci_attach_args *pa, int level,
		  int (*func)(void *), void *arg)
{
	int co;
	struct intrhand *ih;
	struct msi_hdl *msih;
	struct cpu_info *ci;
	struct intrsource *is;
	pcireg_t reg;

	if (!pci_get_capability(pa->pa_pc, pa->pa_tag, PCI_CAP_MSI, &co, 0))
		return NULL;

	ih = intr_establish(-1, &msi_pic, -1, IST_EDGE, level, func, arg, 0);
	if (ih == NULL)
		return NULL;

	msih = malloc(sizeof(*msih), M_DEVBUF, M_WAITOK);
	msih->ih = ih;
	msih->pc = pa->pa_pc;
	msih->tag = pa->pa_tag;
	msih->co = co;

	ci = ih->ih_cpu;
	is = ci->ci_isources[ih->ih_slot];
	reg = pci_conf_read(pa->pa_pc, pa->pa_tag, co + PCI_MSI_CTL);
	pci_conf_write(pa->pa_pc, pa->pa_tag, co + PCI_MSI_MADDR64_LO,
		       LAPIC_MSIADDR_BASE |
		       __SHIFTIN(ci->ci_cpuid, LAPIC_MSIADDR_DSTID_MASK));
	if (reg & PCI_MSI_CTL_64BIT_ADDR) {
		pci_conf_write(pa->pa_pc, pa->pa_tag, co + PCI_MSI_MADDR64_HI,
		    0);
		/* XXX according to the manual, ASSERT is unnecessary if
		 * EDGE
		 */
		pci_conf_write(pa->pa_pc, pa->pa_tag, co + PCI_MSI_MDATA64,
		    __SHIFTIN(is->is_idtvec, LAPIC_MSIDATA_VECTOR_MASK) |
		    LAPIC_MSIDATA_TRGMODE_EDGE | LAPIC_MSIDATA_LEVEL_ASSERT |
		    LAPIC_MSIDATA_DM_FIXED);
	} else {
		/* XXX according to the manual, ASSERT is unnecessary if
		 * EDGE
		 */
		pci_conf_write(pa->pa_pc, pa->pa_tag, co + PCI_MSI_MDATA,
		    __SHIFTIN(is->is_idtvec, LAPIC_MSIDATA_VECTOR_MASK) |
		    LAPIC_MSIDATA_TRGMODE_EDGE | LAPIC_MSIDATA_LEVEL_ASSERT |
		    LAPIC_MSIDATA_DM_FIXED);
	}
	pci_conf_write(pa->pa_pc, pa->pa_tag, co + PCI_MSI_CTL,
	    PCI_MSI_CTL_MSI_ENABLE);
	return msih;
}

void
pci_msi_disestablish(void *ih)
{
	struct msi_hdl *msih = ih;

	pci_conf_write(msih->pc, msih->tag, msih->co + PCI_MSI_CTL, 0);
	intr_disestablish(msih->ih);
	free(msih, M_DEVBUF);
}
#endif

#endif
