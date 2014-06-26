
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
		set_msi_pcitag(vectors[i], pa->pa_tag);
		vectors[i] |= APIC_INT_VIA_MSG;
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
