/**
 * @file    net_topic.c
 * @brief   MQTT topic filter matching for the network bridge.  Coded to
 *          BARR-C:2018.
 */

#include "net_topic.h"

/**
 * @brief   Does a topic match a subscription filter, as a broker decides?
 *          A + matches one level; a trailing # matches any number of
 *          levels, including none (a filter "a/#" matches the topic "a").
 * @param   p_filter  The filter, NUL-terminated.
 * @param   p_topic   The topic, NUL-terminated.
 * @return  true if it matches.
 */
bool
net_topic_matches (char const * p_filter, char const * p_topic)
{
    char const * p_f       = p_filter;
    char const * p_t       = p_topic;
    bool         b_matches = false;
    bool         b_decided = false;

    while (!b_decided)
    {
        if ('#' == *p_f)
        {
            b_matches = true;           /* The rest of the topic matches. */
            b_decided = true;
        }
        else if (('\0' == *p_t) && ('/' == p_f[0]) && ('#' == p_f[1]) &&
                 ('\0' == p_f[2]))
        {
            b_matches = true;           /* "a/#" matches "a". */
            b_decided = true;
        }
        else if ('+' == *p_f)
        {
            /* Skip one level of the topic, whatever it is. */
            while (('\0' != *p_t) && ('/' != *p_t))
            {
                p_t++;
            }

            p_f++;
        }
        else if (*p_f != *p_t)
        {
            b_decided = true;           /* Different: no match. */
        }
        else if ('\0' == *p_f)
        {
            b_matches = true;           /* Both ended together. */
            b_decided = true;
        }
        else
        {
            p_f++;
            p_t++;
        }
    }

    return (b_matches);
}

/*** end of file ***/
