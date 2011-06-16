#ifndef _XEN_ACPI_H
#define _XEN_ACPI_H

#include <linux/types.h>

#ifdef CONFIG_XEN_DOM0
#include <asm/xen/hypervisor.h>
#include <xen/xen.h>
#include <asm/acpi.h>

int xen_acpi_notify_hypervisor_state(u8 sleep_state,
				     u32 pm1a_cnt, u32 pm1b_cnd,
				     bool *skip_rest);

static inline void xen_acpi_sleep_register(void)
{
	if (xen_initial_domain())
		__acpi_override_sleep = xen_acpi_notify_hypervisor_state;
}
#else
static inline void xen_acpi_sleep_register(void)
{
}
#endif

#endif	/* _XEN_ACPI_H */
