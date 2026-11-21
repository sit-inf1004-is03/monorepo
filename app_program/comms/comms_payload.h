/**
 * @file    comms_payload.h
 * @brief   Converting MQTT payloads to and from numbers, in the format of
 *          protocol.h.  Coded to BARR-C:2018.
 *
 * Pure functions with no kernel calls, so comms_test.c can check them
 * directly.
 */

#ifndef COMMS_PAYLOAD_H
#define COMMS_PAYLOAD_H

#include <stdint.h>
#include <tk/typedef.h>

#include "../protocol.h"
#include "../net/net_bridge.h"

#define COMMS_PAYLOAD_MAX_ARGS      (4u)
#define COMMS_PAYLOAD_U64_TEXT_MAX  (21u)   /* 20 digits and a NUL. */
#define COMMS_PAYLOAD_WORDS_TEXT_MAX (TLM_WORDS * COMMS_PAYLOAD_U64_TEXT_MAX)

/** A command or setting: decimal integers, the identifier first. */
typedef struct
{
    uint16_t msg_id;                        /* CMD_* or CFG_*. */
    uint32_t arg_count;                     /* Arguments after it. */
    int32_t  arg[COMMS_PAYLOAD_MAX_ARGS];   /* Unused ones are 0. */
} comms_payload_cmd_t;

ER       comms_payload_parse_text (uint8_t const * p_text, uint32_t len,
                                   comms_payload_cmd_t * p_cmd);
ER       comms_payload_parse_msg (net_bridge_msg_t const * p_msg,
                                  comms_payload_cmd_t * p_cmd);
uint32_t comms_payload_format_u64 (char * p_buf, uint64_t value);
uint32_t comms_payload_format_words (char * p_buf,
                                     uint64_t const * p_words,
                                     uint32_t count);

#endif /* COMMS_PAYLOAD_H */

/*** end of file ***/
