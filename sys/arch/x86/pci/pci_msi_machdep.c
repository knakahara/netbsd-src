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
#include <sys/malloc.h>

#include <dev/pci/pcivar.h>

#include <machine/i82489reg.h>

#include <machine/i82093reg.h>
#include <machine/i82093var.h>
#include <machine/pic.h>

#define BUS_SPACE_WRITE_FLUSH(pc, tag) (void)bus_space_read_4(pc, tag, 0)

static struct pci_attach_args msi_pci_attach_args[NUM_MSI_INTS];
static void
set_msi_pci_attach_args(int vector, struct pci_attach_args *pa)
{
	memcpy(&msi_pci_attach_args[vector - FIRST_MSI_INT], pa, sizeof(*pa));
}
static struct pci_attach_args *
get_msi_pci_attach_args(int vector)
{
	return &msi_pci_attach_args[vector - FIRST_MSI_INT];
}

struct msix_table_handler {
	int mth_table_index;
	bus_space_tag_t mth_tag;
	bus_space_handle_t mth_handle;
};
static struct msix_table_handler msix_table_handlers[NUM_MSI_INTS];

static void
set_msix_table_handler(int vector, struct msix_table_handler *handler)
{
	struct msix_table_handler *mth;

	mth = &msix_table_handlers[vector - FIRST_MSI_INT];
	mth->mth_table_index = handler->mth_table_index;
	mth->mth_tag = handler->mth_tag;
	mth->mth_handle = handler->mth_handle;
}

static struct msix_table_handler *
get_msix_table_handler(int vector)
{
	return &msix_table_handlers[vector - FIRST_MSI_INT];
}

const char *
msi_string(uint64_t ih, char *buf, size_t len)
{
	char *type;
	int dev, vec;

	KASSERT(INT_VIA_MSG(ih));

	if (MSI_INT_IS_MSIX(ih))
		type = "msix";
	else
		type = "msi";

	dev = MSI_INT_DEV(ih);
	vec = MSI_INT_VEC(ih);
	snprintf(buf, len "%s%d vec %d", type, dev, vec);
}

struct msipic {
	int mp_devid;
	int mp_vecid;
};

int
msi_get_devid(struct pic *pic)
{
	KASSERT(pic->pic_msipic != NULL);

	return pic->pic_msipic->mp_devid;
}

int
msi_get_vecid(struct pic *pic)
{
	KASSERT(pic->pic_msipic != NULL);

	return pic->pic_msipic->mp_vecid;
}

#define MSI_MSICTL_ENABLE 1
#define MSI_MSICTL_DISABLE 0
static void
msi_set_msictl_enablebit(struct pic *pic, int pin, int flag)
{
	pci_chipset_tag_t pc = NULL;
	struct pci_attach_args *pa = get_msi_pci_attach_args(pin);
	pcitag_t tag = pa->pa_tag;
	pcireg_t ctl;
	int off;

	/* should use Mask Bits? */
	if (pci_get_capability(pc, tag, PCI_CAP_MSI, &off, NULL) == 0)
		panic("%s: no msi capability", __func__);

	ctl = pci_conf_read(pc, tag, off + PCI_MSI_CTL);
	if (flag == MSI_MSICTL_ENABLE)
		ctl |= PCI_MSI_CTL_MSI_ENABLE;
	else
		ctl &= ~PCI_MSI_CTL_MSI_ENABLE;

	pci_conf_write(pc, tag, off, ctl);
}

static void
msi_hwmask(struct pic *pic, int pin)
{
	msi_set_msictl_enablebit(pic, pin, MSI_MSICTL_DISABLE);
}

static void
msi_hwunmask(struct pic *pic, int pin)
{
	msi_set_msictl_enablebit(pic, pin, MSI_MSICTL_ENABLE);
}

static void
msi_addroute(struct pic *pic, struct cpu_info *ci,
	     int pin, int vec, int type)
{
	pci_chipset_tag_t pc = NULL;
	struct pci_attach_args *pa = get_msi_pci_attach_args(pin);
	pcitag_t tag = pa->pa_tag;
	pcireg_t addr, data, ctl;
	int off;

	if (pci_get_capability(pc, tag, PCI_CAP_MSI, &off, NULL) == 0)
		panic("%s: no msi capability", __func__);

	/*
	 * see OpenBSD's cpu_attach().
	 * OpenBSD's ci->ci_apicid is equal to NetBSD's ci_cpuid.
	 * What's mean OpenBSD's ci->ci_cpuid? the value is sc->sc_dev.dv_unit.
	 */
	addr = LAPIC_MSIADDR_BASE | __SHIFTIN(ci->ci_cpuid, LAPIC_MSIADDR_DSTID_MASK);
	/*if triger mode is edge, it don't care level for trigger mode. */
	data = __SHIFTIN(vec, LAPIC_MSIDATA_VECTOR_MASK) |
		LAPIC_MSIDATA_TRGMODE_EDGE | LAPIC_MSIDATA_DM_FIXED;

	ctl = pci_conf_read(pc, tag, off + PCI_MSI_CTL);

	if (ctl & PCI_MSI_CTL_64BIT_ADDR) {
		pci_conf_write(pc, tag, off + PCI_MSI_MADDR64_LO, addr);
		pci_conf_write(pc, tag, off + PCI_MSI_MADDR64_HI, 0);
		pci_conf_write(pc, tag, off + PCI_MSI_MDATA64, data);
	} else {
		pci_conf_write(pc, tag, off + PCI_MSI_MADDR, addr);
		pci_conf_write(pc, tag, off + PCI_MSI_MDATA, data);
	}
	ctl |= PCI_MSI_CTL_MSI_ENABLE;
	pci_conf_write(pc, tag, off + PCI_MSI_CTL, ctl);
}

static void
msi_delroute(struct pic *pic, struct cpu_info *ci,
	     int pin, int vec, int type)
{
	msi_hwmask(pic, pin);
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

#define MSIX_VECCTL_HWMASK 1
#define MSIX_VECCTL_HWUNMASK 0
static void
msix_set_vecctl_mask(struct pic *pic, int pin, int flag)
{
	pci_chipset_tag_t pc = NULL;
	struct pci_attach_args *pa = get_msi_pci_attach_args(pin);
	pcitag_t tag = pa->pa_tag;
	pcireg_t reg;
	int off;
	uint64_t table_off;

	struct msix_table_handler *mth;

	uint64_t entry_base;
	uint32_t vecctl;
	pcireg_t tbl;

	mth = get_msix_table_handler(pin);

	if (pci_get_capability(pc, tag, PCI_CAP_MSIX, &off, &reg) == 0)
		panic("%s: no msix capability", __func__);
	tbl = pci_conf_read(pc, tag, off + PCI_MSIX_TBLOFFSET);
	table_off = tbl & PCI_MSIX_TBLOFFSET_MASK;

	entry_base = table_off +
		PCI_MSIX_TABLE_ENTRY_SIZE * mth->mth_table_index;

	vecctl = bus_space_read_4(mth->mth_tag, mth->mth_handle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_VECTCTL);
	if (flag == MSIX_VECCTL_HWMASK)
		vecctl |= PCI_MSIX_VECTCTL_HWMASK_MASK;
	else
		vecctl &= ~PCI_MSIX_VECTCTL_HWMASK_MASK;

	bus_space_write_4(mth->mth_tag, mth->mth_handle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_VECTCTL, vecctl);
	BUS_SPACE_WRITE_FLUSH(mth->mth_tag, mth->mth_handle);
}

static void
msix_hwmask(struct pic *pic, int pin)
{
	msix_set_vecctl_mask(pic, pin, MSIX_VECCTL_HWMASK);
}

static void
msix_hwunmask(struct pic *pic, int pin)
{
	msix_set_vecctl_mask(pic, pin, MSIX_VECCTL_HWUNMASK);
}

static void
msix_addroute(struct pic *pic, struct cpu_info *ci,
	     int pin, int vec, int type)
{
	struct pci_attach_args *pa = get_msi_pci_attach_args(pin);
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	int off;
	uint64_t table_off;

	struct msix_table_handler *mth;

	uint64_t entry_base;
	pcireg_t tbl, addr, data, ctl;

	mth = get_msix_table_handler(pin);

	if (pci_get_capability(pc, tag, PCI_CAP_MSIX, &off, NULL) == 0)
		panic("%s: no msix capability", __func__);
	tbl = pci_conf_read(pc, tag, off + PCI_MSIX_TBLOFFSET);
	table_off = tbl & PCI_MSIX_TBLOFFSET_MASK;

	entry_base = table_off +
		PCI_MSIX_TABLE_ENTRY_SIZE * mth->mth_table_index;

	/*
	 * see OpenBSD's cpu_attach().
	 * OpenBSD's ci->ci_apicid is equal to NetBSD's ci_cpuid.
	 * What's mean OpenBSD's ci->ci_cpuid? the value is sc->sc_dev.dv_unit.
	 */
	addr = LAPIC_MSIADDR_BASE | __SHIFTIN(ci->ci_cpuid, LAPIC_MSIADDR_DSTID_MASK);
	/*if triger mode is edge, it don't care level for trigger mode. */
	data = __SHIFTIN(vec, LAPIC_MSIDATA_VECTOR_MASK) |
		LAPIC_MSIDATA_TRGMODE_EDGE | LAPIC_MSIDATA_DM_FIXED;

	bus_space_write_4(mth->mth_tag, mth->mth_handle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_ADDR_LO, addr);
	bus_space_write_4(mth->mth_tag, mth->mth_handle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_ADDR_HI, 0);
	bus_space_write_4(mth->mth_tag, mth->mth_handle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_DATA, data);
	bus_space_write_4(mth->mth_tag, mth->mth_handle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_VECTCTL, 0);
	BUS_SPACE_WRITE_FLUSH(mth->mth_tag, mth->mth_handle);

	ctl = pci_conf_read(pc, tag, off + PCI_MSIX_CTL);

	ctl |= PCI_MSIX_CTL_ENABLE;
	pci_conf_write(pc, tag, off + PCI_MSI_CTL, ctl);
}

static void
msix_delroute(struct pic *pic, struct cpu_info *ci,
	     int pin, int vec, int type)
{
	msix_hwmask(pic, pin);
}

static struct pic msix_pic = {
	.pic_name = "msix",
	.pic_type = PIC_MSIX,
	.pic_vecbase = 0,
	.pic_apicid = 0,
	.pic_lock = __SIMPLELOCK_UNLOCKED,
	.pic_hwmask = msix_hwmask,
	.pic_hwunmask = msix_hwunmask,
	.pic_addroute = msix_addroute,
	.pic_delroute = msix_delroute,
	.pic_edge_stubs = ioapic_edge_stubs,
};

/* XXXX tentative function name */
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
		set_msi_pci_attach_args(vectors[i], pa);
	}

	*ihps = vectors;
	return 0;
}

static void
pci_msi_release_md(void **cookies, int count)
{
	int *vectors;

	vectors = *cookies;
	return intr_free_msi_vectors(vectors, count);
}

static void *
pci_msi_common_establish(pci_chipset_tag_t pc, pci_intr_handle_t ih,
    int level, int (*func)(void *), void *arg, struct pic *pic)
{
	int pin, irq;
	bool mpsafe;

	irq = -1;
	pin = ih & (~MPSAFE_MASK); /* hint to search pcitag_t */
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
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	int off, bir, bar;
	pcireg_t tbl;

	int *vectors;
	int i, err;
	u_int memtype;
	struct msix_table_handler mh;

	if (pci_get_capability(pc, tag, PCI_CAP_MSIX, &off, NULL) == 0) {
		aprint_normal("%s: no msix capability", __func__);
		return 1;
	}
	tbl = pci_conf_read(pc, tag, off + PCI_MSIX_TBLOFFSET);
	bir = tbl & PCI_MSIX_PBABIR_MASK;
	switch(bir) {
	case 0:
		bar = PCI_BAR0;
		break;
	case 1:
		bar = PCI_BAR1;
		break;
	case 2:
		bar = PCI_BAR2;
		break;
	case 3:
		bar = PCI_BAR3;
		break;
	case 4:
		bar = PCI_BAR4;
		break;
	case 5:
		bar = PCI_BAR5;
		break;
	default:
		aprint_normal("the device use reserved BIR values.\n");
		return 1;
	}
	memtype = pci_mapreg_type(pc, tag, bar);
	err = pci_mapreg_map(pa, bar, memtype, BUS_SPACE_MAP_LINEAR,
	    &mh.mth_tag, &mh.mth_handle, NULL, NULL);
	if (err) {
		aprint_normal("cannot map msix table.\n");
		return 1;
	}

	vectors = intr_allocate_msix_vectors(count);
	if (vectors == NULL) {
		aprint_normal("cannot allocate MSI-X vectors.\n");
		return 1;
	}


	for (i = 0; i < *count; i++) {
		mh.mth_table_index = i;
		set_msix_table_handler(vectors[i], &mh);
		set_msi_pci_attach_args(vectors[i], pa);
	}

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
	return pci_msi_common_establish(pc, ih, level, func, arg, &msi_pic);
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

	int offset;
	pcireg_t reg;

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
	return pci_msi_common_establish(pc, ih, level, func, arg, &msix_pic);
}

void
pci_msix_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	pci_msi_common_disestablish(pc, cookie);
}

/* XXXX not yet implement MSI-X remap */
