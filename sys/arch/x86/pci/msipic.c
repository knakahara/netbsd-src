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
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/kmem.h>
#include <sys/malloc.h>
#include <sys/mutex.h>

#include <dev/pci/pcivar.h>

#include <machine/i82489reg.h>
#include <machine/i82093reg.h>
#include <machine/i82093var.h>
#include <machine/pic.h>
#include <machine/lock.h>

#include <x86/pci/msipic.h>

#define BUS_SPACE_WRITE_FLUSH(pc, tag) (void)bus_space_read_4(pc, tag, 0)

struct msipic {
	int mp_bus;
	int mp_dev;
	int mp_fun;

	int mp_devid;
	int mp_veccnt;
	/*
	 * MSI vectors are simple sequential number 0 to mp_veccnt - 1.
	 * So, MSI does *not* use "mp_msixtable". it is always NULL.
	 *
	 * The other hand, MSI-X vectors is complex because MSI-X can use
	 * "remap". So, MSI-X use "mp_msixtable" which mange MSI-X table
	 * index and MSI-X handle.
	 * mp_msixtablei's index means MSI-X handle's "vecid"(=MSI_INT_VEC(pih))
	 * mp_msixtablei[i] value means MSI-X table index
	 * If device driver does "remap" msix, the index and the value
	 * combination change. Furthermore, it may change mp_msixtablei size,
	 * if the device driver "remap" to sparse index.
	 */
	int *mp_msixtablei;

	struct pci_attach_args mp_pa;
	bus_space_tag_t mp_bstag;
	bus_space_handle_t mp_bshandle;
	bus_size_t mp_bssize;
	struct pic *mp_pic;

	kmutex_t mp_lock;
	LIST_ENTRY(msipic) mp_list;
};

static kmutex_t msipic_list_lock;
static LIST_HEAD(, msipic) msipic_list =
	LIST_HEAD_INITIALIZER(msipic_list);

struct dev_seq {
	bool using;
	int last_used_bus;
	int last_used_dev;
	int last_used_fun;
};
#define NUM_MSI_DEVS 256
static struct dev_seq dev_seq[NUM_MSI_DEVS];

static int
allocate_devid(struct pci_attach_args *pa)
{
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	int bus, dev, fun, i;

	pci_decompose_tag(pc, tag, &bus, &dev, &fun);

	/* if the device was once attached, use same devid */
	for (i = 0; i < NUM_MSI_DEVS; i++) {
		/* skip never used dev_seq[i] */
		if (dev_seq[i].last_used_bus == 0 &&
		    dev_seq[i].last_used_dev == 0 &&
		    dev_seq[i].last_used_fun == 0)
			break;

		if (dev_seq[i].last_used_bus == bus &&
		    dev_seq[i].last_used_dev == dev &&
		    dev_seq[i].last_used_fun == fun) {
			dev_seq[i].using = true;
			return i;
		}
	}

	for (i = 0; i < NUM_MSI_DEVS; i++) {
		if (dev_seq[i].using == 0) {
			dev_seq[i].using = true;
			dev_seq[i].last_used_bus = bus;
			dev_seq[i].last_used_dev = dev;
			dev_seq[i].last_used_fun = fun;
			return i;
		}
	}

	aprint_normal("too many MSI devices.\n");
	return -1;
}

static void
release_devid(int devid)
{
	if (devid < 0 || NUM_MSI_DEVS <= devid) {
		aprint_normal("%s: invalid device.\n", __func__);
		return;
	}

	dev_seq[devid].using = false;
	/* keep last_used_* */
}

struct pic *
find_msi_pic(int devid)
{
	struct msipic *mpp;

	mutex_enter(&msipic_list_lock);
	LIST_FOREACH(mpp, &msipic_list, mp_list) {
		if(mpp->mp_devid == devid) {
			mutex_exit(&msipic_list_lock);
			return mpp->mp_pic;
		}
	}
	mutex_exit(&msipic_list_lock);
	return NULL;
}

static struct pic *
construct_common_msi_pic(struct pci_attach_args *pa, struct pic *pic_tmpl)
{
	struct pic *pic;
	struct msipic *msipic;
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	int devid;

	devid = allocate_devid(pa);
	if (devid == -1)
		return NULL;

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
	pci_decompose_tag(pc, tag,
	    &msipic->mp_bus, &msipic->mp_dev, &msipic->mp_fun);
	memcpy(&msipic->mp_pa, pa, sizeof(msipic->mp_pa));
	msipic->mp_devid = devid;
	mutex_init(&msipic->mp_lock, MUTEX_DEFAULT, IPL_NONE);
	/*
	 * pci_msi_alloc() must be called only ont time in the device driver.
	 */
	KASSERT(find_msi_pic(msipic->mp_devid) == NULL);

	mutex_enter(&msipic_list_lock);
	LIST_INSERT_HEAD(&msipic_list, msipic, mp_list);
	mutex_exit(&msipic_list_lock);

	return pic;
}

static void
destruct_common_msi_pic(struct pic *msi_pic)
{
	struct msipic *msipic;

	if (msi_pic == NULL)
		return;

	msipic = msi_pic->pic_msipic;
	mutex_enter(&msipic_list_lock);
	LIST_REMOVE(msipic, mp_list);
	mutex_exit(&msipic_list_lock);

	release_devid(msipic->mp_devid);
	kmem_free(msipic, sizeof(*msipic));
	kmem_free(msi_pic, sizeof(*msi_pic));
}

bool
is_msi_pic(struct pic *pic)
{
	return (pic->pic_msipic != NULL);
}

int
msi_get_devid(struct pic *pic)
{
	KASSERT(is_msi_pic(pic));

	return pic->pic_msipic->mp_devid;
}

static int
msix_get_table_index(struct pic *pic, int vecid)
{
	KASSERT(pic->pic_msipic != NULL);
	KASSERT(pic->pic_type == PIC_MSIX);

	return pic->pic_msipic->mp_msixtablei[vecid];
}

static struct pci_attach_args *
get_msi_pci_attach_args(struct pic *pic)
{
	KASSERT(pic->pic_msipic != NULL);

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

struct pic *
construct_msi_pic(struct pci_attach_args *pa)
{
	return construct_common_msi_pic(pa, &msi_pic_tmpl);
}

void
destruct_msi_pic(struct pic *msi_pic)
{
	destruct_common_msi_pic(msi_pic);
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
	int table_idx;

	table_idx = msix_get_table_index(pic, pin);
	if (table_idx < 0)
		panic("%s: invalid MSI-X table index, devid=%d vecid=%d",
		    __func__, msi_get_devid(pic), pin);

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
	int table_idx;

	table_idx = msix_get_table_index(pic, pin);
	if (table_idx < 0)
		panic("%s: invalid MSI-X table index, devid=%d vecid=%d",
		    __func__, msi_get_devid(pic), pin);

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
	pci_conf_write(pc, tag, off + PCI_MSIX_CTL, ctl);
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

struct pic *
construct_msix_pic(struct pci_attach_args *pa)
{
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	pcireg_t tbl;
	bus_space_tag_t bstag;
	bus_space_handle_t bshandle;
	bus_size_t bssize;
	u_int memtype;
	int off, bir, bar, err;
	struct pic *msix_pic;

	msix_pic = construct_common_msi_pic(pa, &msix_pic_tmpl);
	if (msix_pic == NULL) {
		aprint_normal("cannot allocate MSI-X pic.\n");
		return NULL;
	}

	if (pci_get_capability(pc, tag, PCI_CAP_MSIX, &off, NULL) == 0) {
		aprint_normal("%s: no msix capability", __func__);
		return NULL;
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
		return NULL;
	}
	memtype = pci_mapreg_type(pc, tag, bar);
	err = pci_mapreg_map(pa, bar, memtype, BUS_SPACE_MAP_LINEAR,
	    &bstag, &bshandle, NULL, &bssize);
	if (err) {
		aprint_normal("cannot map msix table.\n");
		return NULL;
	}
	msix_pic->pic_msipic->mp_bstag = bstag;
	msix_pic->pic_msipic->mp_bshandle = bshandle;
	msix_pic->pic_msipic->mp_bssize = bssize;

	return msix_pic;
}

void
destruct_msix_pic(struct pic *msix_pic)
{
	struct msipic *msipic;

	KASSERT(is_msi_pic(msix_pic));
	KASSERT(msix_pic->pic_type == PIC_MSIX);

	msipic = msix_pic->pic_msipic;
	bus_space_unmap(msipic->mp_bstag, msipic->mp_bshandle,
	    msipic->mp_bssize);

	if (msipic->mp_msixtablei != NULL)
		kmem_free(msipic->mp_msixtablei,
		    sizeof(msipic->mp_msixtablei[0]) * msipic->mp_veccnt);
	msipic->mp_msixtablei = NULL;

	destruct_common_msi_pic(msix_pic);
}

int
set_msi_vectors(struct pic *msi_pic, pci_intr_handle_t *pihs, int count)
{
	int i;
	int *vecs;

	KASSERT(is_msi_pic(msi_pic));

	if (msi_pic->pic_type == PIC_MSI) {
		aprint_normal("MSI ignore vecs parameter.\n");
		vecs = NULL;
	}
	else if (msi_pic->pic_type == PIC_MSIX) {
		vecs = kmem_zalloc(sizeof(int) * (count), KM_SLEEP);
		if (vecs == NULL) {
			aprint_normal("cannot allocate MSI-X vector table.\n");
			return 1;
		}

		for (i = 0; i < count; i++) {
			vecs[i] = MSI_INT_VEC(pihs[i]);
		}
	}
	else {
		aprint_normal("invalid MSI type.\n");
		return 1;
	}

	msi_pic->pic_msipic->mp_veccnt = count;
	msi_pic->pic_msipic->mp_msixtablei = vecs;
	return 0;
}

int
remap_msix_vectors(struct pic *msi_pic, pci_intr_handle_t *pihs, int count)
{
	struct pci_attach_args *pa = get_msi_pci_attach_args(msi_pic);
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t tag = pa->pa_tag;
	struct msipic *msipic = msi_pic->pic_msipic;
	bus_space_tag_t bstag = msipic->mp_bstag;
	bus_space_handle_t bshandle = msipic->mp_bshandle;
	uint64_t table_off;
	pcireg_t tbl;
	int off;
	int newseq, oldseq, table_idx, veccnt;
	int *vecs;
	struct pci_msix_table_entry *newtable;

	if (pihs == NULL)
		return 1;

	vecs = kmem_zalloc(sizeof(int) * count, KM_SLEEP);
	if (vecs == NULL) {
		aprint_normal("cannot allocate mp_msixtablei.\n");
		return 1;
	}

	newtable = kmem_zalloc(sizeof(struct pci_msix_table_entry) * count,
	    KM_SLEEP);
	if (newtable == NULL) {
		aprint_normal("cannot allocate MSI-X vector table.\n");
		kmem_free(vecs, sizeof(int) * count);
		return 1;
	}

	if (pci_get_capability(pc, tag, PCI_CAP_MSIX, &off, NULL) == 0)
		panic("%s: no msix capability", __func__);
	tbl = pci_conf_read(pc, tag, off + PCI_MSIX_TBLOFFSET);
	table_off = tbl & PCI_MSIX_TBLOFFSET_MASK;

	/* setup new mp_veccnt and mp_msixtablei */
	veccnt = count;
	for (newseq = 0; newseq < count; newseq++) {
		if (pihs[newseq] == MSI_INT_MSIX_INVALID) {
			vecs[newseq] = -1;
			veccnt -= 1;
		} else
			vecs[newseq] = MSI_INT_VEC(pihs[newseq]);
	}

	mutex_enter(&msipic->mp_lock);

	/* setup new MSI-X table */
	for (table_idx = 0; table_idx < count; table_idx++) {
		uint32_t addr_lo, addr_hi, data, vecctl;
		bool existing = false;

		for (oldseq = 0; oldseq < msipic->mp_veccnt; oldseq++) {
			if (msipic->mp_msixtablei[oldseq] == table_idx) {
				existing = true;
				break;
			}
		}

		if (existing) {
			uint64_t entry_base = table_off +
				PCI_MSIX_TABLE_ENTRY_SIZE * table_idx;
			addr_lo = bus_space_read_4(bstag, bshandle,
			    entry_base + PCI_MSIX_TABLE_ENTRY_ADDR_LO);
			addr_hi = bus_space_read_4(bstag, bshandle,
			    entry_base + PCI_MSIX_TABLE_ENTRY_ADDR_HI);
			data = bus_space_read_4(bstag, bshandle,
			    entry_base + PCI_MSIX_TABLE_ENTRY_DATA);
			vecctl = bus_space_read_4(bstag, bshandle,
			    entry_base + PCI_MSIX_TABLE_ENTRY_VECTCTL);
		} else {
			addr_lo = 0;
			addr_hi = 0;
			data = 0;
			vecctl = PCI_MSIX_VECTCTL_HWMASK_MASK;
		}
		newtable[table_idx].pci_msix_addr_lo = addr_lo;
		newtable[table_idx].pci_msix_addr_hi = addr_hi;
		newtable[table_idx].pci_msix_value = data;
		newtable[table_idx].pci_msix_vector_control = vecctl;
	}

	/* write new MSI-X table */
	for (table_idx = 0; table_idx < count; table_idx++) {
			uint64_t entry_base;

		entry_base = table_off + PCI_MSIX_TABLE_ENTRY_SIZE * table_idx;
		bus_space_write_4(bstag, bshandle,
		    entry_base + PCI_MSIX_TABLE_ENTRY_ADDR_LO,
		    newtable[table_idx].pci_msix_addr_lo);
		bus_space_write_4(bstag, bshandle,
		    entry_base + PCI_MSIX_TABLE_ENTRY_ADDR_HI,
		    newtable[table_idx].pci_msix_addr_hi);
		bus_space_write_4(bstag, bshandle,
		    entry_base + PCI_MSIX_TABLE_ENTRY_DATA,
		    newtable[table_idx].pci_msix_value);
		bus_space_write_4(bstag, bshandle,
		    entry_base + PCI_MSIX_TABLE_ENTRY_VECTCTL,
		    newtable[table_idx].pci_msix_vector_control);
	}
	BUS_SPACE_WRITE_FLUSH(bstag, bshandle);

	/* swap msipic members */
	kmem_free(msipic->mp_msixtablei, sizeof(int) * msipic->mp_veccnt);
	msi_pic->pic_msipic->mp_veccnt = veccnt;
	msi_pic->pic_msipic->mp_msixtablei = vecs;

	mutex_exit(&msipic->mp_lock);

	kmem_free(newtable, sizeof(struct pci_msix_table_entry) * count);
	return 0;
}

void
msipic_init(void)
{
	mutex_init(&msipic_list_lock, MUTEX_DEFAULT, IPL_NONE);
}
