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

#include <sys/types.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/kmem.h>
#include <sys/malloc.h>

#include <dev/pci/pcivar.h>

#include <machine/i82093var.h>
#include <machine/pic.h>

#include <x86/pci/msipic.h>

const char *
pci_msi_string(uint64_t ih, char *buf, size_t len)
{
	int dev, vec;

	KASSERT(INT_VIA_MSI(ih));

	dev = MSI_INT_DEV(ih);
	vec = MSI_INT_VEC(ih);
	if (MSI_INT_IS_MSIX(ih))
		snprintf(buf, len, "msix%d vec %d", dev, vec);
	else
		snprintf(buf, len, "msi%d vec %d", dev, vec);

	return buf;
}

/* XXXX tentative function name */
static int
pci_msi_alloc_md(pci_intr_handle_t **ihps, int *count, struct pci_attach_args *pa)
{
	struct pic *msi_pic;
	uint64_t *vectors;

	msi_pic = construct_msi_pic(pa);
	if (msi_pic == NULL) {
		aprint_normal("cannot allocate MSI pic.\n");
		return 1;
	}

	vectors = intr_allocate_msi_vectors(msi_pic, count);
	if (vectors == NULL) {
		aprint_normal("cannot allocate MSI vectors.\n");
		return 1;
	}

	set_msi_vectors(msi_pic, *count, NULL);

	*ihps = vectors;
	return 0;
}

static void
pci_msi_release_md(void **cookies, int count)
{
	struct pic *pic;
	uint64_t *vectors;

	vectors = *cookies;
	pic = find_msi_pic(MSI_INT_DEV(vectors[0]));
	if (pic == NULL)
		return;

	intr_free_msi_vectors(pic, count);
	delete_common_msi_pic(pic);
}

static void *
pci_msi_common_establish(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg, struct pic *pic)
{
	int pin, irq;
	bool mpsafe;

	KASSERT(INT_VIA_MSI(ih));

	irq = -1;
	pin = MSI_INT_VEC(ih);
	mpsafe = ((ih & MPSAFE_MASK) != 0);

	return intr_establish(irq, pic, pin, IST_EDGE, level, func, arg,
	    mpsafe);
}

static void
pci_msi_common_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	intr_disestablish(cookie);
}

/* XXXX tentative function name */
static int
pci_msix_alloc_md(pci_intr_handle_t **ihps, int *count, struct pci_attach_args *pa)
{
	int i;
	struct pic *msix_pic;
	uint64_t *vectors;
	int *vecs;

	msix_pic = construct_msix_pic(pa);
	if (msix_pic == NULL)
		return 1;

	vectors = intr_allocate_msix_vectors(msix_pic, count);
	if (vectors == NULL) {
		aprint_normal("cannot allocate MSI-X vectors.\n");
		return 1;
	}

	vecs = kmem_zalloc(sizeof(int) * (*count), KM_SLEEP);
	if (vecs == NULL) {
		aprint_normal("cannot allocate MSI-X vector table.\n");
		return 1;
	}

	for (i = 0; i < *count; i++) {
		MSI_INT_MAKE_MSIX(vectors[i]);
		vecs[i] = i;
	}

	set_msi_vectors(msix_pic, *count, vecs);

	*ihps = vectors;
	return 0;
}

/*****************************************************************************/
/*
 * XXXX below APIs are tentative.
 * XXXX these APIS should be MI code.
 */

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

/*
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
	struct pic *pic;

	pic = find_msi_pic(MSI_INT_DEV(ih));
	if (pic == NULL) {
		aprint_normal("pci_intr_handler has no msi_pic\n");
		return NULL;
	}

	return pci_msi_common_establish(pc, ih, level, func, arg, pic);
}

void
pci_msi_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	pci_msi_common_disestablish(pc, cookie);
}

/*
 * return number of the devices's MSI-X vectors
 * return 0 if the device does not support MSI-X
 */
int
pci_msix_count(struct pci_attach_args *pa)
{
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	pcireg_t reg;
	int offset;

	if (pci_get_capability(pc, tag, PCI_CAP_MSIX, &offset, NULL) == 0)
		return 0;

	reg = pci_conf_read(pc, tag, offset + PCI_MSIX_CTL);

	return PCI_MSIX_CTL_TBLSIZE(reg);
}

/*
 * This function is used by device drivers like pci_intr_map().
 *
 * "ihps" is the array  of vector numbers which MSI used instead of IRQ number.
 * "count" can decrease if sturct intrsource cannot be allocated.
 * if count == 0, return non-zero value.
 */
int
pci_msix_alloc(struct pci_attach_args *pa, pci_intr_handle_t **ihps, int *count)
{
	int hw_max;

	if (*count < 1) {
		aprint_normal("invalid count: %d\n", *count);
		return 1;
	}

	hw_max = pci_msix_count(pa);
	if (hw_max == 0)
		return 1;

	if (*count > hw_max) {
		aprint_normal("cut off MSI-X count to %d\n", hw_max);
		*count = hw_max; /* cut off hw_max */
	}

	return pci_msix_alloc_md(ihps, count, pa);
}

/* XXXX currently same as MSI. should be support sparse vectors. */
void
pci_msix_release(void **cookie, int count)
{
	return pci_msi_release(cookie, count);
}

void *
pci_msix_establish(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg)
{
	struct pic *pic;

	pic = find_msi_pic(MSI_INT_DEV(ih));
	if (pic == NULL) {
		aprint_normal("pci_intr_handler has no msi_pic\n");
		return NULL;
	}

	return pci_msi_common_establish(pc, ih, level, func, arg, pic);
}

void
pci_msix_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	pci_msi_common_disestablish(pc, cookie);
}

/* XXXX not yet implement MSI-X remap */
