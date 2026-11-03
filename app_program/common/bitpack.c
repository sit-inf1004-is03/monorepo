/**
 * @file    bitpack.c
 * @brief   Clamp-then-pack helpers for minimum-bit data fields.
 *
 * See bitpack.h for the packing rules.  Every helper is pure: no global
 * state, so it is safe to call from any task, core or handler.
 */

#include "bitpack.h"

/**
 * @brief   Return a mask with the lowest @p width bits set.
 * @param   width  Field width in bits, 0 to 32.  Larger values give 32 bits.
 * @return  The mask, e.g. 0x7F for a width of 7.
 */
uint32_t
bitpack_mask (uint32_t width)
{
    uint32_t mask = 0xFFFFFFFFu;

    if (width < BITPACK_MAX_WIDTH)
    {
        mask = (1u << width) - 1u;
    }

    return (mask);
}

/**
 * @brief   Saturate an unsigned value at @p max_value.
 * @param   value      Value to clamp.
 * @param   max_value  Largest value the field can hold.
 * @return  The clamped value.
 */
uint32_t
bitpack_clamp_u32 (uint32_t value, uint32_t max_value)
{
    uint32_t result = value;

    if (value > max_value)
    {
        result = max_value;
    }

    return (result);
}

/**
 * @brief   Saturate a signed value into [min_value, max_value].
 * @param   value      Value to clamp.
 * @param   min_value  Smallest value the field can hold.
 * @param   max_value  Largest value the field can hold.
 * @return  The clamped value.
 */
int32_t
bitpack_clamp_s32 (int32_t value, int32_t min_value, int32_t max_value)
{
    int32_t result = value;

    if (value < min_value)
    {
        result = min_value;
    }
    else if (value > max_value)
    {
        result = max_value;
    }
    else
    {
        result = value;
    }

    return (result);
}

/**
 * @brief   Write an unsigned field into a word and return the new word.
 *
 * The field is masked to @p width bits as a last line of defence; callers
 * must already have clamped it (rule 1 in bitpack.h).
 *
 * @param   word   Word to modify (a local copy, never a shared word).
 * @param   field  Field value, already clamped.
 * @param   shift  Bit position of the field's least significant bit.
 * @param   width  Field width in bits.
 * @return  @p word with the field replaced.
 */
uint32_t
bitpack_insert (uint32_t word, uint32_t field, uint32_t shift, uint32_t width)
{
    uint32_t result = word;

    if ((width > 0u) && ((shift + width) <= BITPACK_MAX_WIDTH))
    {
        uint32_t const mask = bitpack_mask(width);

        result = (word & ~(mask << shift)) | ((field & mask) << shift);
    }

    return (result);
}

/**
 * @brief   Read an unsigned field out of a word.
 * @param   word   Packed word.
 * @param   shift  Bit position of the field's least significant bit.
 * @param   width  Field width in bits.
 * @return  The field value, or 0 for an invalid shift/width.
 */
uint32_t
bitpack_extract (uint32_t word, uint32_t shift, uint32_t width)
{
    uint32_t result = 0u;

    if ((width > 0u) && ((shift + width) <= BITPACK_MAX_WIDTH))
    {
        result = (word >> shift) & bitpack_mask(width);
    }

    return (result);
}

/**
 * @brief   Clamp a signed value to a field width and encode it as an
 *          unsigned two's complement field.
 *
 * For a width of 8 the range is -128 to +127.  No bitwise operator is
 * applied to signed data (Barr-C Rule 5.3.b): the encoding is arithmetic.
 *
 * @param   value  Signed value.
 * @param   width  Field width in bits, 2 to 31.
 * @return  The field bits, ready for bitpack_insert(); 0 for a bad width.
 */
uint32_t
bitpack_encode_signed (int32_t value, uint32_t width)
{
    uint32_t field = 0u;

    if ((width >= 2u) && (width < BITPACK_MAX_WIDTH))
    {
        /* Cast: width < 32, so 2^(width-1) fits in int32_t. */
        int32_t const half    = (int32_t)(1u << (width - 1u));
        int32_t const clamped = bitpack_clamp_s32(value, -half, half - 1);

        if (clamped < 0)
        {
            /* Cast: clamped + 2*half lies in [half, 2*half), positive. */
            field = (uint32_t)(clamped + (2 * half));
        }
        else
        {
            /* Cast: clamped lies in [0, half), non-negative. */
            field = (uint32_t)clamped;
        }
    }

    return (field);
}

/**
 * @brief   Decode an unsigned two's complement field into a signed value.
 * @param   field  Field bits as returned by bitpack_extract().
 * @param   width  Field width in bits, 2 to 31.
 * @return  The signed value; 0 for a bad width.
 */
int32_t
bitpack_decode_signed (uint32_t field, uint32_t width)
{
    int32_t value = 0;

    if ((width >= 2u) && (width < BITPACK_MAX_WIDTH))
    {
        uint32_t const masked   = field & bitpack_mask(width);
        uint32_t const sign_bit = 1u << (width - 1u);

        /* Cast: masked < 2^31 because width < 32, so it fits in int32_t. */
        value = (int32_t)masked;

        if (masked >= sign_bit)
        {
            /* Cast: the subtraction is done in int64_t; its result lies in
             * [-2^(width-1), 0), which always fits in int32_t.
             */
            value = (int32_t)((int64_t)masked - (int64_t)(1u << width));
        }
    }

    return (value);
}

/*** end of file ***/
