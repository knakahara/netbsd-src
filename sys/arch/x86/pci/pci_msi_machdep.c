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

/*
 * TODO
 *
 *     - PBA (Pending Bit Array) support
 *     - HyperTransport mapping support
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/types.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/systm.h>
#include <sys/cpu.h>
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
pci_msi_string(pci_intr_handle_t ih, char *buf, size_t len)
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

static pci_intr_handle_t
pci_msi_calculate_handle(struct pic *msi_pic, int vector)
{
	pci_intr_handle_t handle;

	KASSERT(is_msi_pic(msi_pic));

	handle = __SHIFTIN((uint64_t)msi_get_devid(msi_pic), MSI_INT_DEV_MASK) |
		__SHIFTIN((uint64_t)vector, MSI_INT_VEC_MASK) | APIC_INT_VIA_MSI;
	if (msi_pic->pic_type == PIC_MSI)
		MSI_INT_MAKE_MSI(handle);
	else if (msi_pic->pic_type == PIC_MSIX)
		MSI_INT_MAKE_MSIX(handle);

	return handle;
}

static pci_intr_handle_t *
pci_msi_alloc_vectors(struct pic *msi_pic, int *count)
{
	struct intrsource *isp;
	const char *intrstr;
	char intrstr_buf[INTRID_LEN + 1];
	pci_intr_handle_t *vectors;
	pci_intr_handle_t pih;
	int i;

	vectors = kmem_zalloc(sizeof(vectors[0]) * (*count), KM_SLEEP);
	if (vectors == NULL) {
		aprint_normal("cannot allocate vectors\n");
		return NULL;
	}

	mutex_enter(&cpu_lock);
	for (i = 0; i < *count; i++) {
		pih = pci_msi_calculate_handle(msi_pic, i);

		intrstr = pci_msi_string(pih, intrstr_buf, sizeof(intrstr_buf));
		isp = intr_allocate_io_intrsource(intrstr);
		if (isp == NULL) {
			aprint_normal("can't allocate io_intersource\n");
			return NULL;
		}

		vectors[i] = pih;
	}
	mutex_exit(&cpu_lock);

	return vectors;
}

static void
pci_msi_free_vectors(struct pic *msi_pic, int count)
{
	const char *intrstr;
	char intrstr_buf[INTRID_LEN + 1];
	pci_intr_handle_t pih;
	int i;

	mutex_enter(&cpu_lock);
	for (i = 0; i < count; i++) {
		pih = pci_msi_calculate_handle(msi_pic, i);
		intrstr = pci_msi_string(pih, intrstr_buf, sizeof(intrstr_buf));
		intr_free_io_intrsource(intrstr);
	}
	mutex_exit(&cpu_lock);
}

static int
pci_msi_alloc_md_common(pci_intr_handle_t **ihps, int *count,
    struct pci_attach_args *pa, bool exact)
{
	int i, error;
	struct pic *msi_pic;
	pci_intr_handle_t *vectors = NULL;

	if ((pa->pa_flags & PCI_FLAGS_MSI_OKAY) == 0) {
		aprint_normal("PCI host bridge does not support MSI.\n");
		return 1;
	}

	msi_pic = construct_msi_pic(pa);
	if (msi_pic == NULL) {
		aprint_normal("cannot allocate MSI pic.\n");
		return 1;
	}

	while (*count > 0) {
		vectors = pci_msi_alloc_vectors(msi_pic, count);
		if (vectors == NULL) {
			if (exact) {
				aprint_normal("cannot allocate MSI vectors.\n");
				return 1;
			} else {
				(*count) >>= 1; /* MSI must be power of 2. */
				continue;
			}
		} else
			break;
	}
	if (vectors == NULL) {
		aprint_normal("cannot allocate MSI vectors.\n");
		return 1;
	}

	for (i = 0; i < *count; i++) {
		MSI_INT_MAKE_MSI(vectors[i]);
	}

	error = set_msi_vectors(msi_pic, NULL, *count);
	if (error) {
		pci_msi_free_vectors(msi_pic, *count);
		destruct_msi_pic(msi_pic);
		return 1;
	}

	*ihps = vectors;
	return 0;
}

static int
pci_msi_alloc_md(pci_intr_handle_t **ihps, int *count, struct pci_attach_args *pa)
{
	return pci_msi_alloc_md_common(ihps, count, pa, false);
}

static int
pci_msi_alloc_exact_md(pci_intr_handle_t **ihps, int count, struct pci_attach_args *pa)
{
	return pci_msi_alloc_md_common(ihps, &count, pa, true);

}

static void
pci_msi_release_md(pci_intr_handle_t **pihs, int count)
{
	struct pic *pic;
	pci_intr_handle_t *vectors;

	vectors = *pihs;
	pic = find_msi_pic(MSI_INT_DEV(vectors[0]));
	if (pic == NULL)
		return;

	pci_msi_free_vectors(pic, count);
	destruct_msi_pic(pic);
}

static void *
pci_msi_common_establish_xname(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg, struct pic *pic,
    const char *xname)
{
	int pin, irq;
	bool mpsafe;

	KASSERT(INT_VIA_MSI(ih));

	irq = -1;
	pin = MSI_INT_VEC(ih);
	mpsafe = ((ih & MPSAFE_MASK) != 0);

	return intr_establish_xname(irq, pic, pin, IST_EDGE, level, func, arg,
	    mpsafe, xname);
}

static void
pci_msi_common_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	intr_disestablish(cookie);
}

static int
pci_msix_alloc_md_common(pci_intr_handle_t **ihps, int *count,
    struct pci_attach_args *pa, bool exact)
{
	int i, error;
	struct pic *msix_pic;
	pci_intr_handle_t *vectors = NULL;

	if ((pa->pa_flags & PCI_FLAGS_MSI_OKAY) == 0 ||
	    (pa->pa_flags & PCI_FLAGS_MSIX_OKAY) == 0) {
		aprint_normal("PCI host bridge does not support MSI-X.\n");
		return 1;
	}

	msix_pic = construct_msix_pic(pa);
	if (msix_pic == NULL)
		return 1;

	while (*count > 0) {
		vectors = pci_msi_alloc_vectors(msix_pic, count);
		if (vectors == NULL) {
			if (exact) {
				aprint_normal("cannot allocate MSI-X vectors.\n");
				return 1;
			} else {
				(*count)--;
				continue;
			}
		} else
			break;
	}
	if (vectors == NULL) {
		aprint_normal("cannot allocate MSI-X vectors.\n");
		return 1;
	}

	for (i = 0; i < *count; i++) {
		MSI_INT_MAKE_MSIX(vectors[i]);
	}

	error = set_msi_vectors(msix_pic, vectors, *count);
	if (error) {
		pci_msi_free_vectors(msix_pic, *count);
		destruct_msix_pic(msix_pic);
		return 1;
	}

	*ihps = vectors;
	return 0;
}

static int
pci_msix_alloc_md(pci_intr_handle_t **ihps, int *count, struct pci_attach_args *pa)
{
	return pci_msix_alloc_md_common(ihps, count, pa, false);
}

static int
pci_msix_alloc_exact_md(pci_intr_handle_t **ihps, int count, struct pci_attach_args *pa)
{
	return pci_msix_alloc_md_common(ihps, &count, pa, true);
}

static void
pci_msix_release_md(pci_intr_handle_t **pihs, int count)
{
	struct pic *pic;
	pci_intr_handle_t *vectors;

	vectors = *pihs;
	pic = find_msi_pic(MSI_INT_DEV(vectors[0]));
	if (pic == NULL)
		return;

	pci_msi_free_vectors(pic, count);
	destruct_msix_pic(pic);
}

static int
pci_msix_remap_md(pci_intr_handle_t *pihs, int count)
{
	int i, devid;
	struct pic *msi_pic;

	/* pihs must be the same devid */
	devid = MSI_INT_DEV(pihs[0]);
	for (i = 1; i < count; i++) {
		if (pihs[i] == MSI_INT_MSIX_INVALID)
			continue;

		if (MSI_INT_DEV(pihs[i]) != devid) {
			aprint_normal("first devid(%d),but pihs[%d]'s devid(%d).\n",
			    devid, i, (int)MSI_INT_DEV(pihs[i]));
			return EINVAL;
		}
	}

	msi_pic = find_msi_pic(devid);
	if (msi_pic == NULL) {
		aprint_normal("invalid devid: %d\n", devid);
		return ENOENT;
	}

	return remap_msix_vectors(msi_pic, pihs, count);
}

/*****************************************************************************/
/*
 * these APIs should be MI code.
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

int
pci_msi_alloc_exact(struct pci_attach_args *pa, pci_intr_handle_t **ihps, int count)
{
	int hw_max;

	if (count < 1) {
		aprint_normal("invalid count: %d\n", count);
		return 1;
	}
	if (((count - 1) & count) != 0) {
		aprint_normal("count %d must be power of 2.\n", count);
		return 1;
	}

	hw_max = pci_msi_count(pa);
	if (hw_max == 0)
		return 1;

	if (count > hw_max) {
		aprint_normal("over hardware max MSI count %d\n", hw_max);
		return 1;
	}

	return pci_msi_alloc_exact_md(ihps, count, pa);
}

void
pci_msi_release(pci_intr_handle_t **pihs, int count)
{
	if (count < 1) {
		aprint_normal("invalid count: %d\n", count);
		return;
	}

	return pci_msi_release_md(pihs, count);
}

void *
pci_msi_establish_xname(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg, const char *xname)
{
	struct pic *pic;

	pic = find_msi_pic(MSI_INT_DEV(ih));
	if (pic == NULL) {
		aprint_normal("pci_intr_handler has no msi_pic\n");
		return NULL;
	}

	return pci_msi_common_establish_xname(pc, ih, level, func, arg, pic,
	    xname);
}

void *
pci_msi_establish(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg)
{
	return pci_msi_establish_xname(pc, ih, level, func, arg, "unknown");
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

int
pci_msix_alloc_exact(struct pci_attach_args *pa, pci_intr_handle_t **ihps, int count)
{
	int hw_max;

	if (count < 1) {
		aprint_normal("invalid count: %d\n", count);
		return 1;
	}

	hw_max = pci_msix_count(pa);
	if (hw_max == 0)
		return 1;

	if (count > hw_max) {
		aprint_normal("over hardware max MSI-X count %d\n", hw_max);
		return 1;
	}

	return pci_msix_alloc_exact_md(ihps, count, pa);
}

void
pci_msix_release(pci_intr_handle_t **pihs, int count)
{
	if (count < 1) {
		aprint_normal("invalid count: %d\n", count);
		return;
	}

	return pci_msix_release_md(pihs, count);
}

void *
pci_msix_establish_xname(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg, const char *xname)
{
	struct pic *pic;

	pic = find_msi_pic(MSI_INT_DEV(ih));
	if (pic == NULL) {
		aprint_normal("pci_intr_handler has no msi_pic\n");
		return NULL;
	}

	return pci_msi_common_establish_xname(pc, ih, level, func, arg, pic,
	    xname);
}

void *
pci_msix_establish(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg)
{
	return pci_msix_establish_xname(pc, ih, level, func, arg, "unknown");
}

void
pci_msix_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	pci_msi_common_disestablish(pc, cookie);
}

int
pci_msix_remap(pci_intr_handle_t *pihs, int count)
{
	if (pihs == NULL)
		return EINVAL;

	if (count < 0 || MAX_MSIX_COUNT < count) {
		aprint_normal("invalid count: %d", count);
		return EINVAL;
	}

	return pci_msix_remap_md(pihs, count);
}

void
pci_any_intr_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	/* XXXX ov_intr_disestablish() care? */
	intr_disestablish(cookie);
}

void
pci_any_intr_release(pci_intr_handle_t **pihs, int count)
{
	if (!INT_VIA_MSI(*pihs[0]))
		pci_intr_release(pihs[0]);
	else if (!MSI_INT_IS_MSIX(*pihs[0]))
		pci_msi_release(pihs, count);
	else
		pci_msix_release(pihs, count);
}
