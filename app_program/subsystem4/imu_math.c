/**
 * @file    imu_math.c
 * @brief   Small single-precision maths helpers (no libm needed).
 *
 * References:
 *   - atan polynomial: Abramowitz & Stegun, "Handbook of Mathematical
 *     Functions", formula 4.4.49 (|error| <= 1e-5 rad on [-1, 1]).
 *   - sine: Taylor series to x^11 after range reduction to [-pi/2, pi/2].
 *   - square root: Newton-Raphson from an exponent-halving first guess.
 */

#include <math.h>   /* isfinite() only: a compiler built-in, no libm call. */

#include "imu_math.h"

/* atan(z) = z * (A1 + A3 z^2 + A5 z^4 + A7 z^6 + A9 z^8), |z| <= 1. */
#define ATAN_A1                 (0.9998660f)
#define ATAN_A3                 (-0.3302995f)
#define ATAN_A5                 (0.1801410f)
#define ATAN_A7                 (-0.0851330f)
#define ATAN_A9                 (0.0208351f)

/* Taylor coefficients 1/3!, 1/5!, 1/7!, 1/9!, 1/11!. */
#define SIN_C3                  (1.6666667e-1f)
#define SIN_C5                  (8.3333333e-3f)
#define SIN_C7                  (1.9841270e-4f)
#define SIN_C9                  (2.7557319e-6f)
#define SIN_C11                 (2.5052108e-8f)

#define SQRT_NEWTON_STEPS       (4u)
#define DEG_FULL_TURN           (360.0f)
#define DEG_HALF_TURN           (180.0f)
#define DEG_QUARTER_TURN        (90.0f)

/* Largest float that converts to int32_t without overflow. */
#define ROUND_LIMIT             (2147483520.0f)

static float32_t atan_unit (float32_t ratio);

/**
 * @brief   Absolute value.
 * @param   value  Input.
 * @return  |value|.
 */
float32_t
imu_math_abs (float32_t value)
{
    float32_t result = value;

    if (value < 0.0f)
    {
        result = -value;
    }

    return (result);
}

/**
 * @brief   Clamp a value into [min_value, max_value].
 * @param   value      Input.
 * @param   min_value  Lower bound.
 * @param   max_value  Upper bound (must be >= min_value).
 * @return  The clamped value.
 */
float32_t
imu_math_clamp (float32_t value, float32_t min_value, float32_t max_value)
{
    float32_t result = value;

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
 * @brief   Square root.
 * @param   value  Input; values <= 0 and non-finite values return 0.
 * @return  sqrt(value).
 */
float32_t
imu_math_sqrt (float32_t value)
{
    float32_t result = 0.0f;

    if ((value > 0.0f) && imu_math_is_finite(value))
    {
        /* First guess: halve the binary exponent by repeated scaling, so
         * Newton-Raphson starts within a factor of 2 of the answer.
         */
        float32_t guess   = 1.0f;
        float32_t reduced = value;

        while (reduced >= 4.0f)
        {
            reduced = reduced * 0.25f;
            guess   = guess * 2.0f;
        }

        while (reduced < 0.25f)
        {
            reduced = reduced * 4.0f;
            guess   = guess * 0.5f;
        }

        for (uint32_t step = 0u; step < SQRT_NEWTON_STEPS; step++)
        {
            guess = 0.5f * (guess + (value / guess));
        }

        result = guess;
    }

    return (result);
}

/**
 * @brief   Four-quadrant arctangent in degrees.
 * @param   y_value  Opposite side.
 * @param   x_value  Adjacent side.
 * @return  Angle in (-180, +180] degrees; 0 when both inputs are 0.
 */
float32_t
imu_math_atan2_deg (float32_t y_value, float32_t x_value)
{
    float32_t const abs_y = imu_math_abs(y_value);
    float32_t const abs_x = imu_math_abs(x_value);
    float32_t       angle = 0.0f;

    if ((abs_x > 0.0f) || (abs_y > 0.0f))
    {
        /* Evaluate atan on the ratio that is <= 1, then unfold. */
        if (abs_x >= abs_y)
        {
            angle = atan_unit(abs_y / abs_x) * IMU_MATH_DEG_PER_RAD;
        }
        else
        {
            angle = DEG_QUARTER_TURN -
                    (atan_unit(abs_x / abs_y) * IMU_MATH_DEG_PER_RAD);
        }

        if (x_value < 0.0f)
        {
            angle = DEG_HALF_TURN - angle;
        }

        if (y_value < 0.0f)
        {
            angle = -angle;
        }
    }

    return (angle);
}

/**
 * @brief   Sine of an angle given in degrees.
 * @param   angle_deg  Any finite angle.
 * @return  sin(angle).
 */
float32_t
imu_math_sin_deg (float32_t angle_deg)
{
    /* Reduce to (-180, 180], then fold into [-90, 90] using
     * sin(180 - a) = sin(a).
     */
    float32_t angle = imu_math_wrap_360(angle_deg);

    if (angle > DEG_HALF_TURN)
    {
        angle = angle - DEG_FULL_TURN;
    }

    if (angle > DEG_QUARTER_TURN)
    {
        angle = DEG_HALF_TURN - angle;
    }
    else if (angle < -DEG_QUARTER_TURN)
    {
        angle = -DEG_HALF_TURN - angle;
    }
    else
    {
        /* Already in [-90, 90]. */
    }

    float32_t const rad = angle * IMU_MATH_RAD_PER_DEG;
    float32_t const sq  = rad * rad;

    return (rad * (1.0f - (sq * (SIN_C3 - (sq * (SIN_C5 - (sq * (SIN_C7 -
            (sq * (SIN_C9 - (sq * SIN_C11)))))))))));
}

/**
 * @brief   Wrap an angle into [0, 360).
 * @param   angle_deg  Any finite angle.
 * @return  The equivalent angle in [0, 360); 0 for non-finite input.
 */
float32_t
imu_math_wrap_360 (float32_t angle_deg)
{
    float32_t result = 0.0f;

    if (imu_math_is_finite(angle_deg))
    {
        /* Cast: |angle_deg / 360| is bounded by imu_math_round()'s
         * saturation, and the remainder is corrected by the loops below.
         */
        int32_t const turns = imu_math_round(angle_deg / DEG_FULL_TURN);

        result = angle_deg - ((float32_t)turns * DEG_FULL_TURN);

        while (result < 0.0f)
        {
            result = result + DEG_FULL_TURN;
        }

        while (result >= DEG_FULL_TURN)
        {
            result = result - DEG_FULL_TURN;
        }
    }

    return (result);
}

/**
 * @brief   Round half away from zero, saturating at the int32_t range.
 * @param   value  Input; NaN returns 0.
 * @return  The nearest integer.
 */
int32_t
imu_math_round (float32_t value)
{
    int32_t result = 0;

    if (!imu_math_is_finite(value))
    {
        result = 0;
    }
    else if (value >= ROUND_LIMIT)
    {
        result = INT32_MAX;
    }
    else if (value <= -ROUND_LIMIT)
    {
        result = INT32_MIN;
    }
    else if (value >= 0.0f)
    {
        /* Cast: value + 0.5 < ROUND_LIMIT + 1, inside the int32_t range. */
        result = (int32_t)(value + 0.5f);
    }
    else
    {
        /* Cast: value - 0.5 > -ROUND_LIMIT - 1, inside the int32_t range. */
        result = (int32_t)(value - 0.5f);
    }

    return (result);
}

/**
 * @brief   Barr-C Rule 5.4.b.v check: neither NaN nor infinity.
 * @param   value  Input.
 * @return  true if the value is finite.
 */
bool
imu_math_is_finite (float32_t value)
{
    return (0 != isfinite(value));
}

/**
 * @brief   atan() for a ratio in [0, 1], in radians.
 * @param   ratio  Input in [0, 1].
 * @return  atan(ratio).
 */
static float32_t
atan_unit (float32_t ratio)
{
    float32_t const sq = ratio * ratio;

    return (ratio * (ATAN_A1 + (sq * (ATAN_A3 + (sq * (ATAN_A5 +
            (sq * (ATAN_A7 + (sq * ATAN_A9)))))))));
}

/*** end of file ***/
