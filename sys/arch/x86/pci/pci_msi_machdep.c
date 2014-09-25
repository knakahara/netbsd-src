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

#include <machine/i82489reg.h>

#include <machine/i82093reg.h>
#include <machine/i82093var.h>
#include <machine/pic.h>

#define MSI_FIRST_DEVID 0

#define BUS_SPACE_WRITE_FLUSH(pc, tag) (void)bus_space_read_4(pc, tag, 0)

struct msipic {
	u_int mp_bus;
	u_int mp_device;
	u_int mp_function;

	int mp_devid;
	int *mp_vecids;
	int mp_veccnt;

	struct pci_attach_args mp_pa;
	bus_space_tag_t mp_bstag;
	bus_space_handle_t mp_bshandle;
	struct pic *mp_pic;
	LIST_ENTRY(msipic) mp_list;
};

static LIST_HEAD(, msipic) msipic_list =
	LIST_HEAD_INITIALIZER(msipic_list);

static struct pic *
find_msi_pic(int devid)
{
	struct msipic *mpp;

	LIST_FOREACH(mpp, &msipic_list, mp_list) {
		if(mpp->mp_devid == devid)
			return mpp->mp_pic;
	}
	return NULL;
}

static struct pic *
create_common_msi_pic(struct pci_attach_args *pa, struct pic *pic_tmpl)
{
	struct pic *pic;
	struct msipic *msipic;

	static int dev_seq = MSI_FIRST_DEVID;

	pic = kmem_zalloc(sizeof(*pic), KM_SLEEP);
	if (pic == NULL) {
		return NULL;
	}
	memcpy(pic, pic_tmpl, sizeof(*pic));

	msipic = kmem_zalloc(sizeof(*msipic), KM_SLEEP);
	if (msipic == NULL) {
		kmem_free(pic, sizeof(*msipic));
		return NULL;
	}

	pic->pic_msipic = msipic;
	msipic->mp_pic = pic;
	memcpy(&msipic->mp_pa, pa, sizeof(msipic->mp_pa));
	msipic->mp_devid = dev_seq;
	/*
	 * pci_msi_alloc() must be called only ont time in the device driver.
	 */
	KASSERT(find_msi_pic(msipic->mp_devid) == NULL);
	LIST_INSERT_HEAD(&msipic_list, msipic, mp_list);

	KASSERT(dev_seq != INT_MAX);
	dev_seq++;

	return pic;
}

const char *
msi_string(uint64_t ih, char *buf, size_t len)
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

bool
is_msi_pic(struct pic *pic)
{
	return (pic->pic_msipic != NULL);
}

int
msi_get_devid(struct pic *pic)
{
	KASSERT(pic->pic_msipic != NULL);

	return pic->pic_msipic->mp_devid;
}

static int
msi_get_vecid(struct pic *pic, int seq)
{
	KASSERT(pic->pic_msipic != NULL);

	return pic->pic_msipic->mp_vecids[seq];
}

static struct pci_attach_args *
get_msi_pci_attach_args(struct pic *pic)
{
	KASSERT(is_msi_pic(pic));

	return &pic->pic_msipic->mp_pa;
}

#define MSI_MSICTL_ENABLE 1
#define MSI_MSICTL_DISABLE 0
static void
msi_set_msictl_enablebit(struct pic *pic, int pin, int flag)
{
	pci_chipset_tag_t pc = NULL;
	struct pci_attach_args *pa = get_msi_pci_attach_args(pic);
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
	struct pci_attach_args *pa = get_msi_pci_attach_args(pic);
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

static struct pic msi_pic_tmpl = {
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
	.pic_ioapic = NULL,
};

static struct pic *
create_msi_pic(struct pci_attach_args *pa)
{
	return create_common_msi_pic(pa, &msi_pic_tmpl);
}

#define MSIX_VECCTL_HWMASK 1
#define MSIX_VECCTL_HWUNMASK 0
static void
msix_set_vecctl_mask(struct pic *pic, int pin, int flag)
{
	pci_chipset_tag_t pc = NULL;
	struct pci_attach_args *pa = get_msi_pci_attach_args(pic);
	pcitag_t tag = pa->pa_tag;
	pcireg_t reg;
	uint64_t table_off;
	uint64_t entry_base;
	uint32_t vecctl;
	pcireg_t tbl;
	int off;

	bus_space_tag_t bstag = pic->pic_msipic->mp_bstag;
	bus_space_handle_t bshandle = pic->pic_msipic->mp_bshandle;
	int table_idx = msi_get_vecid(pic, pin);

	if (pci_get_capability(pc, tag, PCI_CAP_MSIX, &off, &reg) == 0)
		panic("%s: no msix capability", __func__);
	tbl = pci_conf_read(pc, tag, off + PCI_MSIX_TBLOFFSET);
	table_off = tbl & PCI_MSIX_TBLOFFSET_MASK;

	entry_base = table_off +
		PCI_MSIX_TABLE_ENTRY_SIZE * table_idx;

	vecctl = bus_space_read_4(bstag, bshandle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_VECTCTL);
	if (flag == MSIX_VECCTL_HWMASK)
		vecctl |= PCI_MSIX_VECTCTL_HWMASK_MASK;
	else
		vecctl &= ~PCI_MSIX_VECTCTL_HWMASK_MASK;

	bus_space_write_4(bstag, bshandle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_VECTCTL, vecctl);
	BUS_SPACE_WRITE_FLUSH(bstag, bshandle);
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
	struct pci_attach_args *pa = get_msi_pci_attach_args(pic);
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	uint64_t table_off;
	uint64_t entry_base;
	pcireg_t tbl, addr, data, ctl;
	int off;

	bus_space_tag_t bstag = pic->pic_msipic->mp_bstag;
	bus_space_handle_t bshandle = pic->pic_msipic->mp_bshandle;
	int table_idx = msi_get_vecid(pic, pin);

	if (pci_get_capability(pc, tag, PCI_CAP_MSIX, &off, NULL) == 0)
		panic("%s: no msix capability", __func__);
	tbl = pci_conf_read(pc, tag, off + PCI_MSIX_TBLOFFSET);
	table_off = tbl & PCI_MSIX_TBLOFFSET_MASK;

	entry_base = table_off +
		PCI_MSIX_TABLE_ENTRY_SIZE * table_idx;

	/*
	 * see OpenBSD's cpu_attach().
	 * OpenBSD's ci->ci_apicid is equal to NetBSD's ci_cpuid.
	 * What's mean OpenBSD's ci->ci_cpuid? the value is sc->sc_dev.dv_unit.
	 */
	addr = LAPIC_MSIADDR_BASE | __SHIFTIN(ci->ci_cpuid, LAPIC_MSIADDR_DSTID_MASK);
	/*if triger mode is edge, it don't care level for trigger mode. */
	data = __SHIFTIN(vec, LAPIC_MSIDATA_VECTOR_MASK) |
		LAPIC_MSIDATA_TRGMODE_EDGE | LAPIC_MSIDATA_DM_FIXED;

	bus_space_write_4(bstag, bshandle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_ADDR_LO, addr);
	bus_space_write_4(bstag, bshandle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_ADDR_HI, 0);
	bus_space_write_4(bstag, bshandle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_DATA, data);
	bus_space_write_4(bstag, bshandle,
	    entry_base + PCI_MSIX_TABLE_ENTRY_VECTCTL, 0);
	BUS_SPACE_WRITE_FLUSH(bstag, bshandle);

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

static struct pic msix_pic_tmpl = {
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

static struct pic *
create_msix_pic(struct pci_attach_args *pa)
{
	return create_common_msi_pic(pa, &msix_pic_tmpl);
}

/* XXXX tentative function name */
static int
pci_msi_alloc_md(pci_intr_handle_t **ihps, int *count, struct pci_attach_args *pa)
{
	struct pic *msi_pic;
	uint64_t *vectors;

	msi_pic = create_msi_pic(pa);
	if (msi_pic == NULL) {
		aprint_normal("cannot allocate MSI pic.\n");
		return 1;
	}

	vectors = intr_allocate_msi_vectors(msi_pic, count);
	if (vectors == NULL) {
		aprint_normal("cannot allocate MSI vectors.\n");
		return 1;
	}

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
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	pcireg_t tbl;
	bus_space_tag_t bstag;
	bus_space_handle_t bshandle;
	u_int memtype;
	int off, bir, bar, i, err;
	struct pic *msix_pic;
	uint64_t *vectors;
	int *vecs;

	msix_pic = create_msix_pic(pa);
	if (msix_pic == NULL) {
		aprint_normal("cannot allocate MSI-X pic.\n");
		return 1;
	}

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
	    &bstag, &bshandle, NULL, NULL);
	if (err) {
		aprint_normal("cannot map msix table.\n");
		return 1;
	}
	msix_pic->pic_msipic->mp_bstag = bstag;
	msix_pic->pic_msipic->mp_bshandle = bshandle;

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
	msix_pic->pic_msipic->mp_vecids = vecs;
	msix_pic->pic_msipic->mp_veccnt = *count;

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
