/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SHENG_RIMPS_PAYLOAD_H
#define SHENG_RIMPS_PAYLOAD_H

struct rimps_payload { unsigned int region; size_t offset, length; };

/* Stock firmware's marker counts two equal regions; the first four bytes
 * contain the marker itself. Reject malformed markers before touching memory.
 */
static bool rimps_payload_decode(u32 marker, size_t region_size,
		struct rimps_payload *payload)
{
	struct rimps_payload result;

	if (region_size < 4 || region_size > U32_MAX / 2 || marker < 4 ||
	    marker > region_size * 2)
		return false;
	if (marker <= region_size) {
		result.region = 0;
		result.offset = 4;
		result.length = marker - 4;
	} else {
		result.region = 1;
		result.offset = 0;
		result.length = marker - region_size;
	}
	*payload = result;
	return true;
}

static bool get_last_newline(char *buf, size_t size, size_t *count)
{
	size_t i = size;
	while (i) {
		if (buf[--i] == '\n') {
			buf[i] = '\0';
			*count = i + 1;
			return true;
		}
	}
	*count = size;
	return false;
}
#endif
