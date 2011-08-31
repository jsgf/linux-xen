#ifndef _XEN_ACPI_H
#define _XEN_ACPI_H

#include <linux/types.h>

#ifdef CONFIG_XEN_DOM0
#include <asm/xen/hypervisor.h>
#include <xen/xen.h>
#include <asm/acpi.h>
#include <linux/acpi.h>

int xen_acpi_notify_hypervisor_state(u8 sleep_state,
				     u32 pm1a_cnt, u32 pm1b_cnd,
				     bool *skip_rest);

static inline int xen_acpi_suspend_lowlevel(void)
{
	/*
	 * Xen will save and restore CPU context, so
	 * we can skip that and just go straight to
	 * the suspend.
	 */
	acpi_enter_sleep_state(ACPI_STATE_S3);
	return 0;
}
static inline void xen_acpi_sleep_register(void)
{
	if (xen_initial_domain()) {
		acpi_suspend_lowlevel = xen_acpi_suspend_lowlevel;
		__acpi_override_sleep = xen_acpi_notify_hypervisor_state;
	}
}
#else
static inline void xen_acpi_sleep_register(void)
{
}
#endif

#endif	/* _XEN_ACPI_H */
