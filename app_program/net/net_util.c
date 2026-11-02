/*
 *	net_util.c
 *	Small pure functions used by the bridge: retry timing and topic
 *	matching. No kernel, lwIP or radio calls, so they are built in every
 *	configuration and comms_test.c can check them directly.
 */

#include "net_bridge.h"

EXPORT void net_backoff_reset(NET_BACKOFF *b)
{
	b->step_ms = b->min_ms;
}

/*
 * Exponential backoff: the wait doubles after each failure in a row, up
 * to max_ms. Each wait is randomised to between half and all of the
 * current step ("equal jitter"), so cars that lost the same access point
 * or broker do not all retry at the same moment. random is passed in so
 * the tests can fix it.
 */
EXPORT UW net_backoff_next(NET_BACKOFF *b, UW random)
{
	UW	step = b->step_ms;
	UW	half = step / 2;

	b->step_ms = (step >= b->max_ms / 2) ? b->max_ms : step * 2;
	return half + random % (step - half + 1);
}

/* MQTT topic filter match: '+' matches one level, a trailing '#' any
   number of levels (including none: "a/#" matches "a") */
EXPORT BOOL net_topic_matches(const char *filter, const char *topic)
{
	for (;;) {
		if (*filter == '#') {
			return TRUE;
		}
		if (*topic == '\0' && filter[0] == '/' && filter[1] == '#' &&
		    filter[2] == '\0') {
			return TRUE;
		}
		if (*filter == '+') {
			while (*topic != '\0' && *topic != '/') {
				topic++;
			}
			filter++;
			continue;
		}
		if (*filter != *topic) {
			return FALSE;
		}
		if (*filter == '\0') {
			return TRUE;
		}
		filter++;
		topic++;
	}
}
