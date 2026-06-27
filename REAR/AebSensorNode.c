#include "AebSensorNode.h"

#include "IfxPort.h"
#include "Stm/Std/IfxStm.h"
#include "IfxCpu_Intrinsics.h"   /* __dsync */

#ifndef NON_CACHED
#define NON_CACHED(addr)  ((void *)((uint32)(addr) | 0x20000000u))
#endif

typedef struct
{
    Ifx_P  *trigPort;
    uint8   trigPin;
    Ifx_P  *echoPort;
    uint8   echoPin;
    uint16  samples[3];
    uint8   sampleCount;
    uint8   sampleIndex;
    uint16  filteredCmX10;
    uint32  lastValidTick;
} AebUltrasonicSensor;

static AebUltrasonicSensor g_leftUltrasonic = {
    AEB_ULTRA_LEFT_TRIG_PORT,  AEB_ULTRA_LEFT_TRIG_PIN,
    AEB_ULTRA_LEFT_ECHO_PORT,  AEB_ULTRA_LEFT_ECHO_PIN,
    {0u, 0u, 0u}, 0u, 0u, 0u, 0u
};

static AebUltrasonicSensor g_rightUltrasonic = {
    AEB_ULTRA_RIGHT_TRIG_PORT, AEB_ULTRA_RIGHT_TRIG_PIN,
    AEB_ULTRA_RIGHT_ECHO_PORT, AEB_ULTRA_RIGHT_ECHO_PIN,
    {0u, 0u, 0u}, 0u, 0u, 0u, 0u
};

/* ---- CPU2 -> CPU0 LMU 더블버퍼 (여기서 정의) ---- */
#pragma section fardata "lmudata"
volatile AebUltraLmuFrame g_ultra_buf[2];
volatile uint8            g_ultra_idx;
#pragma section fardata restore

/* ================= 타이밍 헬퍼 (STM0) ================= */
static uint32 aeb_nowTicks(void) { return IfxStm_getLower(&MODULE_STM2); }

static uint32 aeb_ticksFromUs(uint32 us)
{
    sint32 t = IfxStm_getTicksFromMicroseconds(&MODULE_STM2, us);
    return (t <= 0) ? 1u : (uint32)t;
}
static uint32 aeb_ticksFromMs(uint32 ms)
{
    sint32 t = IfxStm_getTicksFromMilliseconds(&MODULE_STM2, ms);
    return (t <= 0) ? 1u : (uint32)t;
}
static uint32 aeb_ticksToUs(uint32 ticks)
{
    float32 f = IfxStm_getFrequency(&MODULE_STM2);
    return (uint32)(((float32)ticks * 1000000.0f) / f);
}
static void aeb_delayUs(uint32 us) { IfxStm_waitTicks(&MODULE_STM2, aeb_ticksFromUs(us)); }

static void aeb_waitUntilElapsedMs(uint32 startTick, uint32 periodMs)
{
    uint32 periodTicks = aeb_ticksFromMs(periodMs);
    uint32 elapsedTicks = aeb_nowTicks() - startTick;
    if (elapsedTicks < periodTicks)
    {
        IfxStm_waitTicks(&MODULE_STM0, periodTicks - elapsedTicks);
    }
}

static boolean aeb_isFresh(uint32 lastValidTick, uint32 staleMs)
{
    if (lastValidTick == 0u) return FALSE;
    return (aeb_nowTicks() - lastValidTick) <= aeb_ticksFromMs(staleMs);
}

/* ================= 초음파 ================= */
static uint16 aeb_median3(const uint16 *s, uint8 count)
{
    if (count == 0u) return 0u;
    if (count == 1u) return s[0];
    if (count == 2u) return (uint16)((s[0] + s[1]) / 2u);

    uint16 a = s[0], b = s[1], c = s[2];
    if (a > b) { uint16 t = a; a = b; b = t; }
    if (b > c) { uint16 t = b; b = c; c = t; }
    if (a > b) { uint16 t = a; a = b; b = t; }
    return b;
}

static boolean aeb_waitPinState(Ifx_P *port, uint8 pin, boolean state, uint32 timeoutUs)
{
    uint32 start = aeb_nowTicks();
    uint32 timeoutTicks = aeb_ticksFromUs(timeoutUs);
    while (IfxPort_getPinState(port, pin) != state)
    {
        if ((aeb_nowTicks() - start) >= timeoutTicks) return FALSE;
    }
    return TRUE;
}

static void aeb_ultrasonicInit(AebUltrasonicSensor *sensor)
{
    IfxPort_setPinModeOutput(sensor->trigPort, sensor->trigPin, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeInput(sensor->echoPort, sensor->echoPin, IfxPort_InputMode_pullDown);
    IfxPort_setPinLow(sensor->trigPort, sensor->trigPin);
}

static boolean aeb_ultrasonicReadCmX10(AebUltrasonicSensor *sensor, uint16 *cmX10)
{
    IfxPort_setPinLow(sensor->trigPort, sensor->trigPin);
    aeb_delayUs(2u);
    IfxPort_setPinHigh(sensor->trigPort, sensor->trigPin);
    aeb_delayUs(10u);
    IfxPort_setPinLow(sensor->trigPort, sensor->trigPin);

    if (!aeb_waitPinState(sensor->echoPort, sensor->echoPin, TRUE, AEB_ULTRA_ECHO_TIMEOUT_US)) return FALSE;

    uint32 pulseStart = aeb_nowTicks();
    if (!aeb_waitPinState(sensor->echoPort, sensor->echoPin, FALSE, AEB_ULTRA_ECHO_TIMEOUT_US)) return FALSE;

    uint32 pulseUs = aeb_ticksToUs(aeb_nowTicks() - pulseStart);
    uint32 distanceCmX10 = (pulseUs * 1715u + 5000u) / 10000u;

    if ((distanceCmX10 < AEB_ULTRA_MIN_CM_X10) || (distanceCmX10 > AEB_ULTRA_MAX_CM_X10)) return FALSE;

    *cmX10 = (uint16)distanceCmX10;
    return TRUE;
}

static boolean aeb_ultrasonicUpdate(AebUltrasonicSensor *sensor)
{
    uint16 raw;
    if (aeb_ultrasonicReadCmX10(sensor, &raw))
    {
        sensor->samples[sensor->sampleIndex] = raw;
        sensor->sampleIndex = (uint8)((sensor->sampleIndex + 1u) % 3u);
        if (sensor->sampleCount < 3u) sensor->sampleCount++;
        sensor->filteredCmX10 = aeb_median3(sensor->samples, sensor->sampleCount);
        sensor->lastValidTick = aeb_nowTicks();
    }
    return aeb_isFresh(sensor->lastValidTick, AEB_ULTRA_STALE_MS);
}

/* ================= LMU write (CPU2) ================= */
static void aeb_writeUltraLmu(const AebUltraLmuFrame *in)
{
    volatile uint8 *nc_idx = (volatile uint8 *)NON_CACHED(&g_ultra_idx);
    uint8 cur = (uint8)(*nc_idx & 1u);   /* 부팅 직후 가비지 클램프 */
    uint8 wr  = (uint8)(1u - cur);

    volatile AebUltraLmuFrame *nc_f = (volatile AebUltraLmuFrame *)NON_CACHED(&g_ultra_buf[wr]);
    nc_f->left      = in->left;
    nc_f->right     = in->right;
    nc_f->validMask = in->validMask;

    __dsync();
    *nc_idx = wr;
}

/* ================= CPU2 API ================= */
void AebSensorNode_ultraInit(void)
{
    AebUltraLmuFrame zero = {0.0f, 0.0f, 0u};
    aeb_writeUltraLmu(&zero);   /* CPU0 첫 read 시 유효 0 프레임 보장 */

    aeb_ultrasonicInit(&g_leftUltrasonic);
    aeb_ultrasonicInit(&g_rightUltrasonic);
}

void AebSensorNode_ultraRunOnce(void)
{
    uint32 cycleStart = aeb_nowTicks();

    boolean leftValid  = aeb_ultrasonicUpdate(&g_leftUltrasonic);
    aeb_delayUs(AEB_ULTRA_CROSSTALK_DELAY_US);
    boolean rightValid = aeb_ultrasonicUpdate(&g_rightUltrasonic);

    AebUltraLmuFrame frame;
    frame.validMask = 0u;
    if (leftValid)  frame.validMask |= AEB_VALID_ULTRA_LEFT;
    if (rightValid) frame.validMask |= AEB_VALID_ULTRA_RIGHT;
    frame.left  = leftValid  ? ((float32)g_leftUltrasonic.filteredCmX10  * 0.001f) : 0.0f;
    frame.right = rightValid ? ((float32)g_rightUltrasonic.filteredCmX10 * 0.001f) : 0.0f;

    aeb_writeUltraLmu(&frame);

    aeb_waitUntilElapsedMs(cycleStart, AEB_NODE_LOOP_PERIOD_MS);
}
