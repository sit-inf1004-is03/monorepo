/**
 * @file    comms_payload.c
 * @brief   Converting MQTT payloads to and from numbers, in the format of
 *          protocol.h.  Coded to BARR-C:2018.
 */

#include "../../app.h"
#include "comms_payload.h"

#define NUMBERS_MAX         (1u + COMMS_PAYLOAD_MAX_ARGS)
#define ID_MAX              (65535)         /* Largest identifier. */
#define POSITIVE_MAX        (0x7FFFFFFFu)   /* Largest int32_t. */
#define NEGATIVE_MAX        (0x80000000u)   /* Size of the smallest. */
#define DECIMAL_BASE        (10u)
#define U64_DIGITS_MAX      (20u)

static char const g_digit_chars[] = "0123456789";

static bool is_separator (uint8_t byte);
static bool is_digit (uint8_t byte);
static ER   parse_number (uint8_t const * p_text, uint32_t len,
                          uint32_t * p_idx, int32_t * p_value);

/**
 * @brief   Parse a command or setting: decimal numbers, optionally signed,
 *          separated by spaces or commas, the first being the identifier.
 *          "2 60 -20 1500" gives msg_id 2, 3 arguments, {60, -20, 1500, 0}.
 * @param   p_text  The payload; need not be NUL-terminated.
 * @param   len     Its length.
 * @param   p_cmd   Where to put the result; unspecified on E_PAR.
 * @return  E_OK, or E_PAR for anything else: no numbers, a non-digit, a
 *          number out of range, an identifier outside 0..65535, or more
 *          than COMMS_PAYLOAD_MAX_ARGS arguments.
 */
ER
comms_payload_parse_text (uint8_t const * p_text, uint32_t len,
                          comms_payload_cmd_t * p_cmd)
{
    int32_t  values[NUMBERS_MAX] = {0};
    uint32_t count  = 0u;
    uint32_t idx    = 0u;
    ER       result = E_OK;

    while ((idx < len) && (E_OK == result))
    {
        if (is_separator(p_text[idx]))
        {
            idx++;
        }
        else if (NUMBERS_MAX == count)
        {
            result = E_PAR;                 /* Too many numbers. */
        }
        else
        {
            result = parse_number(p_text, len, &idx, &values[count]);
            count++;
        }
    }

    if ((E_OK != result) || (0u == count) || (values[0] < 0) ||
        (values[0] > ID_MAX))
    {
        result = E_PAR;                     /* No valid identifier. */
    }
    else
    {
        /* Cast: values[0] is within 0..ID_MAX, checked above. */
        p_cmd->msg_id    = (uint16_t) values[0];
        p_cmd->arg_count = count - 1u;

        for (uint32_t arg = 0u; arg < COMMS_PAYLOAD_MAX_ARGS; arg++)
        {
            p_cmd->arg[arg] = (arg < p_cmd->arg_count) ? values[arg + 1u] : 0;
        }
    }

    return (result);
}

/**
 * @brief   Parse the payload of a received message as a command.
 * @param   p_msg  The message.
 * @param   p_cmd  Where to put the result.
 * @return  As comms_payload_parse_text().
 */
ER
comms_payload_parse_msg (net_bridge_msg_t const * p_msg,
                         comms_payload_cmd_t * p_cmd)
{
    return (comms_payload_parse_text(p_msg->payload, p_msg->len, p_cmd));
}

/**
 * @brief   Write a 64-bit number in decimal.  tm_sprintf() has no 64-bit
 *          conversion, hence this.
 * @param   p_buf  At least COMMS_PAYLOAD_U64_TEXT_MAX bytes.
 * @param   value  The number.
 * @return  The text's length, without the NUL that follows it.
 */
uint32_t
comms_payload_format_u64 (char * p_buf, uint64_t value)
{
    char     digits[U64_DIGITS_MAX];
    uint32_t count     = 0u;
    uint64_t remaining = value;

    do
    {
        digits[count] = g_digit_chars[remaining % DECIMAL_BASE];
        count++;
        remaining /= DECIMAL_BASE;
    } while (0u != remaining);

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        p_buf[idx] = digits[count - 1u - idx];
    }

    p_buf[count] = '\0';

    return (count);
}

/**
 * @brief   Write telemetry words in decimal, one space apart: the payload
 *          of robocar/id/telemetry.
 * @param   p_buf    At least COMMS_PAYLOAD_WORDS_TEXT_MAX bytes.
 * @param   p_words  The words.
 * @param   count    How many, at most TLM_WORDS; more are left out.
 * @return  The text's length, without the NUL that follows it.
 */
uint32_t
comms_payload_format_words (char * p_buf, uint64_t const * p_words,
                            uint32_t count)
{
    uint32_t len = 0u;

    for (uint32_t idx = 0u; (idx < count) && (idx < TLM_WORDS); idx++)
    {
        if (idx > 0u)
        {
            p_buf[len] = ' ';
            len++;
        }

        len += comms_payload_format_u64(&p_buf[len], p_words[idx]);
    }

    p_buf[len] = '\0';

    return (len);
}

/**
 * @brief   Does a byte separate two numbers?
 * @param   byte  The byte.
 * @return  true for a space, comma, tab, CR or LF.
 */
static bool
is_separator (uint8_t byte)
{
    return ((' ' == byte) || (',' == byte) || ('\t' == byte) ||
            ('\r' == byte) || ('\n' == byte));
}

/**
 * @brief   Is a byte a decimal digit?
 * @param   byte  The byte.
 * @return  true for 0 to 9.
 */
static bool
is_digit (uint8_t byte)
{
    return ((byte >= '0') && (byte <= '9'));
}

/**
 * @brief   Parse one optionally signed decimal number that must fit an
 *          int32_t and end at a separator or at the end of the text.
 * @param   p_text   The payload.
 * @param   len      Its length.
 * @param   p_idx    In: where the number starts.  Out: just after it.
 * @param   p_value  Out: the number.
 * @return  E_OK, or E_PAR if it is not such a number.
 */
static ER
parse_number (uint8_t const * p_text, uint32_t len, uint32_t * p_idx,
              int32_t * p_value)
{
    uint32_t idx        = *p_idx;
    uint32_t magnitude  = 0u;
    bool     b_negative = false;
    bool     b_digits   = false;
    ER       result     = E_OK;

    if (('-' == p_text[idx]) || ('+' == p_text[idx]))
    {
        b_negative = ('-' == p_text[idx]);
        idx++;
    }

    uint32_t const limit = b_negative ? NEGATIVE_MAX : POSITIVE_MAX;

    while ((idx < len) && (is_digit(p_text[idx])) && (E_OK == result))
    {
        /* Cast: after is_digit(), the byte minus '0' is 0..9. */
        uint32_t const digit = (uint32_t)(p_text[idx] - '0');

        /* Check before multiplying, so the magnitude cannot wrap. */
        if (magnitude > ((limit - digit) / DECIMAL_BASE))
        {
            result = E_PAR;                 /* Does not fit an int32_t. */
        }
        else
        {
            magnitude = (magnitude * DECIMAL_BASE) + digit;
            b_digits  = true;
            idx++;
        }
    }

    if ((E_OK == result) &&
        ((!b_digits) || ((idx < len) && (!is_separator(p_text[idx])))))
    {
        result = E_PAR;                     /* Not a number. */
    }

    if (E_OK == result)
    {
        if (b_negative && (magnitude > 0u))
        {
            /* Cast: magnitude - 1 is at most POSITIVE_MAX, so it fits, and
               the result reaches the smallest int32_t without overflow. */
            *p_value = -((int32_t)(magnitude - 1u)) - 1;
        }
        else
        {
            /* Cast: magnitude is at most POSITIVE_MAX here. */
            *p_value = (int32_t) magnitude;
        }
    }

    *p_idx = idx;

    return (result);
}

/*** end of file ***/
