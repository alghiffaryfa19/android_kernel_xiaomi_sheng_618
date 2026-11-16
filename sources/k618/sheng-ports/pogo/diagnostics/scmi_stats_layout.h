/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SHENG_SCMI_STATS_LAYOUT_H
#define SHENG_SCMI_STATS_LAYOUT_H
/* Firmware offsets are four-byte words. Domain header:16, frequency entry:24. */
static bool scmi_stats_domain_range(size_t size, u32 word_offset,
		unsigned int levels, size_t *offset)
{
	u64 start = (u64)word_offset * 4;
	u64 length = 16 + (u64)levels * 24;

	if (levels > 255 || start > size || length > size - start)
		return false;
	*offset = start;
	return true;
}
#endif
