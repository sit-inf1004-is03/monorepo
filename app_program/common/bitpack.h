/**
 * @file    bitpack.h
 * @brief   Clamp-then-pack helpers for minimum-bit data fields.
 *
 * Shared by every subsystem (Tribunal 1, Section 2.6: "Andrew takes on the
 * clamp and pack macros in the shared header").  They are functions rather
 * than macros, as required by Barr-C Rule 6.3.a.
 *
 * Rules every caller follows (Tribunal 1, Section 2.4, "Bit-width limits"):
 *   1. Clamp or saturate a value to its documented range BEFORE packing it.
 *      A value is never allowed to wrap silently and decode as a small one.
 *   2. Pack with explicit shifts and masks, never with C bit-fields or a
 *      memcpy of a struct, so the bit order is identical on the car and on
 *      the laptop decoder.
 *   3. Build a shared word in a local variable and publish it with ONE
 *      32-bit store.  Never read-modify-write a word another task writes.
 *
 * Signed fields are stored in two's complement of the field width, which is
 * the encoding Buddy 1's frame decoder expects.
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>

/** Largest field width, in bits, that the helpers accept. */
#define BITPACK_MAX_WIDTH   (32u)

uint32_t bitpack_mask (uint32_t width);
uint32_t bitpack_clamp_u32 (uint32_t value, uint32_t max_value);
int32_t  bitpack_clamp_s32 (int32_t value, int32_t min_value,
                            int32_t max_value);
uint32_t bitpack_insert (uint32_t word, uint32_t field, uint32_t shift,
                         uint32_t width);
uint32_t bitpack_extract (uint32_t word, uint32_t shift, uint32_t width);
uint32_t bitpack_encode_signed (int32_t value, uint32_t width);
int32_t  bitpack_decode_signed (uint32_t field, uint32_t width);

#endif /* BITPACK_H */

/*** end of file ***/
