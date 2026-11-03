/**
 * @file    faults.c
 * @brief   Team-wide fault mask on one micro T-Kernel event flag.
 *
 * See faults.h for the bit map, the waiting pattern and the ownership rules.
 * tk_set_flg(), tk_clr_flg() and tk_ref_flg() may be called from tasks and
 * from handlers; faults_wait() and faults_wait_all() only from tasks.
 */

#include "faults.h"

/* Handle of the shared event flag; written once by faults_init(). */
static ID volatile g_h_faults_flg = 0;

/**
 * @brief   Create the shared event flag.  Idempotent.
 *
 * NOTE: must run in phase 1 of usermain() (subsystemN_init), before any task
 * starts.  subsystem4_init() calls it; calling it earlier from app_main.c is
 * also safe because a second call does nothing.
 *
 * @return  E_OK, or the negative error code from tk_cre_flg().
 */
ER
faults_init (void)
{
    ER result = E_OK;

    if (g_h_faults_flg <= 0)
    {
        T_CFLG cflg = {0};

        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0u;

        ID const h_flg = tk_cre_flg(&cflg);

        if (h_flg < E_OK)
        {
            result = h_flg;
        }
        else
        {
            g_h_faults_flg = h_flg;
        }
    }

    return (result);
}

/**
 * @brief   Latch one or more fault bits and notify every consumer.
 *
 * Fault bits and notify bits are set by a single tk_set_flg() call, so a
 * waiting consumer that wakes always sees the new fault bit.
 *
 * @param   fault_bits  Bits from FAULTS_ALL_MASK (caller's own bits only).
 * @return  E_OK, E_PAR for bits outside FAULTS_ALL_MASK, E_NOEXS before
 *          faults_init(), or a kernel error code.
 */
ER
faults_set (uint32_t fault_bits)
{
    ER result = E_OK;

    if ((0u == fault_bits) || (0u != (fault_bits & ~FAULTS_ALL_MASK)))
    {
        result = E_PAR;
    }
    else if (g_h_faults_flg <= 0)
    {
        result = E_NOEXS;
    }
    else
    {
        result = tk_set_flg(g_h_faults_flg,
                            fault_bits | FAULTS_NOTIFY_ALL);
    }

    return (result);
}

/**
 * @brief   Clear one or more latched fault bits.
 * @param   fault_bits  Bits from FAULTS_ALL_MASK.
 * @return  E_OK, E_PAR, E_NOEXS or a kernel error code.
 */
ER
faults_clear (uint32_t fault_bits)
{
    ER result = E_OK;

    if ((0u == fault_bits) || (0u != (fault_bits & ~FAULTS_ALL_MASK)))
    {
        result = E_PAR;
    }
    else if (g_h_faults_flg <= 0)
    {
        result = E_NOEXS;
    }
    else
    {
        /* tk_clr_flg() ANDs the pattern with its argument. */
        result = tk_clr_flg(g_h_faults_flg, ~fault_bits);
    }

    return (result);
}

/**
 * @brief   Read the whole flag pattern (faults, READY and event bits).
 * @return  The current pattern, or 0 if the flag does not exist yet.
 */
uint32_t
faults_get (void)
{
    uint32_t pattern = 0u;

    if (g_h_faults_flg > 0)
    {
        T_RFLG rflg = {0};

        if (E_OK == tk_ref_flg(g_h_faults_flg, &rflg))
        {
            pattern = rflg.flgptn;
        }
    }

    return (pattern);
}

/**
 * @brief   Test whether ANY of the given bits is currently set.
 * @param   fault_bits  Bits to test (any part of the 32-bit pattern).
 * @return  true if at least one of the bits is set.
 */
bool
faults_is_set (uint32_t fault_bits)
{
    return (0u != (faults_get() & fault_bits));
}

/**
 * @brief   Set READY, command-event or test bits (no notify bits).
 * @param   status_bits  Bits from FAULTS_STATUS_MASK.
 * @return  E_OK, E_PAR, E_NOEXS or a kernel error code.
 */
ER
faults_signal (uint32_t status_bits)
{
    ER result = E_OK;

    if ((0u == status_bits) || (0u != (status_bits & ~FAULTS_STATUS_MASK)))
    {
        result = E_PAR;
    }
    else if (g_h_faults_flg <= 0)
    {
        result = E_NOEXS;
    }
    else
    {
        result = tk_set_flg(g_h_faults_flg, status_bits);
    }

    return (result);
}

/**
 * @brief   Clear READY, command-event or test bits.
 * @param   status_bits  Bits from FAULTS_STATUS_MASK.
 * @return  E_OK, E_PAR, E_NOEXS or a kernel error code.
 */
ER
faults_clear_status (uint32_t status_bits)
{
    ER result = E_OK;

    if ((0u == status_bits) || (0u != (status_bits & ~FAULTS_STATUS_MASK)))
    {
        result = E_PAR;
    }
    else if (g_h_faults_flg <= 0)
    {
        result = E_NOEXS;
    }
    else
    {
        result = tk_clr_flg(g_h_faults_flg, ~status_bits);
    }

    return (result);
}

/**
 * @brief   Wait for any of the given NOTIFY or command-event bits.
 *
 * Uses TWF_ORW | TWF_BITCLR, so only the bits in @p wait_bits are cleared
 * when the wait is released.  Never pass fault bits or READY bits here.
 *
 * @param   wait_bits   NOTIFY and/or command-event bits owned by the caller.
 * @param   timeout_ms  Timeout in ms, TMO_POL or TMO_FEVR.
 * @param   p_pattern   Receives the pattern at release time; may be NULL.
 * @return  E_OK, E_TMOUT, E_PAR, E_NOEXS or a kernel error code.
 */
ER
faults_wait (uint32_t wait_bits, TMO timeout_ms, uint32_t * p_pattern)
{
    ER   result  = E_OK;
    UINT pattern = 0u;

    if ((0u == wait_bits) ||
        (0u != (wait_bits & (FAULTS_ALL_MASK | FAULTS_READY_ALL))))
    {
        result = E_PAR;
    }
    else if (g_h_faults_flg <= 0)
    {
        result = E_NOEXS;
    }
    else
    {
        result = tk_wai_flg(g_h_faults_flg, wait_bits,
                            TWF_ORW | TWF_BITCLR, &pattern, timeout_ms);
    }

    if (NULL != p_pattern)
    {
        *p_pattern = pattern;
    }

    return (result);
}

/**
 * @brief   Wait until ALL of the given level bits are set, without clearing
 *          them (e.g. Navigation waiting for every READY bit).
 * @param   wait_bits   READY bits (or other level bits) to wait for.
 * @param   timeout_ms  Timeout in ms, TMO_POL or TMO_FEVR.
 * @return  E_OK, E_TMOUT, E_PAR, E_NOEXS or a kernel error code.
 */
ER
faults_wait_all (uint32_t wait_bits, TMO timeout_ms)
{
    ER   result  = E_OK;
    UINT pattern = 0u;

    if (0u == wait_bits)
    {
        result = E_PAR;
    }
    else if (g_h_faults_flg <= 0)
    {
        result = E_NOEXS;
    }
    else
    {
        result = tk_wai_flg(g_h_faults_flg, wait_bits, TWF_ANDW, &pattern,
                            timeout_ms);
    }

    return (result);
}

/*** end of file ***/
