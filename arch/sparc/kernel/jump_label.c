#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/cpu.h>

#include <linux/jump_label.h>
#include <linux/memory.h>

#ifdef HAVE_JUMP_LABEL

static void __jump_label_transform(struct jump_entry *entry,
				   enum jump_label_type type)
{
	u32 val;
	u32 *insn = (u32 *) (unsigned long) entry->code;

	if (type == JUMP_LABEL_ENABLE) {
		s32 off = (s32)entry->target - (s32)entry->code;

#ifdef CONFIG_SPARC64
		/* ba,pt %xcc, . + (off << 2) */
		val = 0x10680000 | ((u32) off >> 2);
#else
		/* ba . + (off << 2) */
		val = 0x10800000 | ((u32) off >> 2);
#endif
	} else {
		val = 0x01000000;
	}

	*insn = val;
	flushi(insn);
}

void arch_jump_label_transform(struct jump_entry *entry,
			       enum jump_label_type type)
{
	get_online_cpus();
	mutex_lock(&text_mutex);

	__jump_label_transform(entry, type);

	mutex_unlock(&text_mutex);
	put_online_cpus();
}

void __init arch_jump_label_transform_early(struct jump_entry *entry,
					    enum jump_label_type type)
{
	__jump_label_transform(entry, type);
}

#endif
