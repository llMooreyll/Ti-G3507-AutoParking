#include "ultrasonic.h"

/*
 * HC-SR04 Ultrasonic Ranging Driver (GPIO method)
 *
 * Wiring:
 *   TRIG (PA24) → HC-SR04 Trig pin
 *   ECHO (PA9)  → HC-SR04 Echo pin
 *
 * Principle:
 *   1. Pull TRIG high for ~10µs → sensor emits 8 ultrasonic pulses at 40kHz
 *   2. ECHO goes high, stays high for a duration proportional to distance
 *   3. A one-shot timer measures the ECHO high pulse width
 *   4. distance(mm) = pulse_width(µs) × 0.17
 *
 * SysConfig requirements:
 *   GPIO  instance "ULTRASONIC"  : TRIG=PA24(Output), ECHO=PA9(Input)
 *   TIMER instance "TIMER_ULTRASONIC" : One-shot Up, prescaler=80, period=40ms
 */

#define ULTRASONIC_TIMEOUT_CYCLES                                              \
    (8000000U) /* ~100ms at 80MHz for ECHO rising-edge wait */
#define ULTRASONIC_MAX_DIST_MM (6000U) /* reject readings beyond 6 metres */

void Ultrasonic_Init(void)
{
    /* Nothing to initialise — pins are configured by SYSCFG_DL_init(),
     * and the one-shot timer is started on demand inside Read_Ultrasonic().
     */
}

uint16_t Read_Ultrasonic(void)
{
    uint32_t timeout;
    uint16_t distVal;

    /* ---------- 1. Reset timer ---------- */
    DL_Timer_setTimerCount(TIMER_ULTRASONIC_INST, 0);
    DL_Timer_clearInterruptStatus(
        TIMER_ULTRASONIC_INST,
        DL_TIMER_INTERRUPT_LOAD_EVENT);

    /* ---------- 2. Send 10 µs trigger pulse ---------- */
    DL_GPIO_setPins(ULTRASONIC_PORT, ULTRASONIC_TRIG_PIN);
    DL_Common_delayCycles(
        CPUCLK_FREQ / 100000); /* 80M / 100k = 800 cycles ≈ 10 µs */
    DL_GPIO_clearPins(ULTRASONIC_PORT, ULTRASONIC_TRIG_PIN);

    /* ---------- 3. Wait for ECHO rising edge (with timeout) ---------- */
    timeout = ULTRASONIC_TIMEOUT_CYCLES;
    while (!DL_GPIO_readPins(ULTRASONIC_PORT, ULTRASONIC_ECHO_PIN))
    {
        if (--timeout == 0U)
            return 0; /* sensor did not respond */
    }

    /* ---------- 4. Start one-shot timer ---------- */
    DL_Timer_startCounter(TIMER_ULTRASONIC_INST);

    /* ---------- 5. Wait for ECHO falling edge (timer overflow = timeout) ---------- */
    while (DL_GPIO_readPins(ULTRASONIC_PORT, ULTRASONIC_ECHO_PIN))
    {
        if (DL_Timer_getRawInterruptStatus(
                TIMER_ULTRASONIC_INST,
                DL_TIMER_INTERRUPT_LOAD_EVENT))
        {
            DL_Timer_stopCounter(TIMER_ULTRASONIC_INST);
            return 0; /* pulse too long → out of range / sensor error */
        }
    }

    /* ---------- 6. Stop timer and read pulse width ---------- */
    DL_Timer_stopCounter(TIMER_ULTRASONIC_INST);
    distVal = (uint16_t)((float)DL_Timer_getTimerCount(TIMER_ULTRASONIC_INST) *
                         0.17f);

    /* ---------- 7. Reject out-of-range values ---------- */
    if (distVal > ULTRASONIC_MAX_DIST_MM)
        distVal = 0;

    return distVal;
}
