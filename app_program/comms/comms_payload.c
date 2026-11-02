/*
 *	comms_payload.c
 *	Converting MQTT payloads to and from numbers (format: protocol.h).
 *
 *	Plain functions with no kernel calls, so comms_test.c can check them
 *	directly.
 */

#include "comms.h"

#define W_MAX	0x7fffffffL

LOCAL BOOL is_separator(UB c)
{
	return c == ' ' || c == ',' || c == '\t' || c == '\r' || c == '\n';
}

/*
 * Numbers are decimal, optionally signed, separated by spaces or commas.
 * Returns E_PAR, leaving *cmd unspecified, for anything else: no numbers,
 * a non-digit, a number out of range, or more than COMMS_MAX_ARGS
 * arguments.
 */
EXPORT ER comms_parse_text(const UB *text, INT len, COMMS_CMD *cmd)
{
	W	values[1 + COMMS_MAX_ARGS];
	INT	count = 0;
	INT	i = 0;
	INT	k;

	while (i < len) {
		BOOL	negative = FALSE;
		BOOL	digits = FALSE;
		UW	value = 0;

		if (is_separator(text[i])) {
			i++;
			continue;
		}
		if (count == 1 + COMMS_MAX_ARGS) {
			return E_PAR;			/* too many numbers */
		}

		if (text[i] == '-' || text[i] == '+') {
			negative = (text[i] == '-');
			i++;
		}
		while (i < len && text[i] >= '0' && text[i] <= '9') {
			value = value * 10 + (UW)(text[i] - '0');
			if (value > (UW)W_MAX + (negative ? 1 : 0)) {
				return E_PAR;		/* does not fit in a W */
			}
			digits = TRUE;
			i++;
		}
		if (!digits || (i < len && !is_separator(text[i]))) {
			return E_PAR;			/* not a number */
		}
		values[count++] = negative ? (W)(0 - value) : (W)value;
	}

	if (count == 0 || values[0] < 0 || values[0] > 0xffff) {
		return E_PAR;				/* no valid identifier */
	}

	cmd->id   = (UH)values[0];
	cmd->argc = count - 1;
	for (k = 0; k < COMMS_MAX_ARGS; k++) {
		cmd->arg[k] = (k < cmd->argc) ? values[k + 1] : 0;
	}
	return E_OK;
}

EXPORT ER comms_parse_cmd(const COMMS_MSG *msg, COMMS_CMD *cmd)
{
	return comms_parse_text(msg->payload, msg->len, cmd);
}

/* tm_sprintf() has no 64-bit conversion, hence this */
EXPORT INT comms_format_u64(UB *buf, UD value)
{
	UB	digits[20];
	INT	n = 0;
	INT	i;

	do {
		digits[n++] = (UB)('0' + value % 10);
		value /= 10;
	} while (value != 0);

	for (i = 0; i < n; i++) {
		buf[i] = digits[n - 1 - i];
	}
	buf[n] = '\0';
	return n;
}
