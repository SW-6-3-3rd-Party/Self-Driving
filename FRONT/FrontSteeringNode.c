#include "FrontSteeringNode.h"

#include <string.h>

#include "IfxPort.h"
#include "IfxStm.h"
#include "Gtm/Std/IfxGtm.h"
#include "Gtm/Std/IfxGtm_Cmu.h"
#include "Gtm/Std/IfxGtm_Tom.h"
#include "_PinMap/IfxGtm_PinMap.h"

#define HPSC_PACKET_SIZE                       (40u)
#define HPSC_HEADER_SIZE                       (32u)
#define HPSC_VERSION                           (1u)
#define HPSC_CONTROL_DISABLED                  (0u)
#define HPSC_CONTROL_STEERING_ANGLE            (1u)
#define HPSC_FLAG_STEERING_VALID               (1u << 0)
#define HPSC_FLAG_EMERGENCY_CENTER             (1u << 1)
#define HPSC_FLAG_UPSTREAM_VALID               (1u << 2)

#define IPV4_PROTOCOL_UDP                      (17u)
#define ETHERTYPE_IPV4                         (0x0800u)

/* Servo signal pin: P02.3, TOM0 channel 3. */
#ifndef FRONT_STEERING_SERVO_PIN
#define FRONT_STEERING_SERVO_PIN               IfxGtm_TOM0_3_TOUT3_P02_3_OUT
#endif
#ifndef FRONT_STEERING_SERVO_TOM
#define FRONT_STEERING_SERVO_TOM               IfxGtm_Tom_0
#endif
#ifndef FRONT_STEERING_SERVO_TOM_CHANNEL
#define FRONT_STEERING_SERVO_TOM_CHANNEL       IfxGtm_Tom_Ch_3
#endif
#ifndef FRONT_STEERING_SERVO_PERIOD_TICKS
#define FRONT_STEERING_SERVO_PERIOD_TICKS      (20000u)
#endif
#ifndef IFXGTM_CMU_CLKEN_FXCLK
#define IFXGTM_CMU_CLKEN_FXCLK                 IFXGTM_CMU_CLKEN_CLK0
#endif

typedef struct
{
    boolean initialized;
    boolean linkValid;
    boolean commandValid;
    boolean emergencyCenter;
    float32 targetAngleRad;
    float32 appliedAngleRad;
    float32 maxRateRadS;
    uint32  lastSequence;
    uint32  lastPacketMs;
    uint32  lastApplyMs;
    uint32  acceptedPackets;
    uint32  rejectedPackets;
} FrontSteeringNode_State;

static FrontSteeringNode_State g_state;

static uint32 front_nowMs(void)
{
    float32 stmFrequency = IfxStm_getFrequency(&MODULE_STM0);
    uint32 ticks = IfxStm_getLower(&MODULE_STM0);
    return (uint32)(((float32)ticks * 1000.0f) / stmFrequency);
}

static uint16 readU16Le(const uint8 *bytes)
{
    return (uint16)(((uint16)bytes[1] << 8) | bytes[0]);
}

static uint16 readU16Be(const uint8 *bytes)
{
    return (uint16)(((uint16)bytes[0] << 8) | bytes[1]);
}

static uint32 readU32Le(const uint8 *bytes)
{
    return ((uint32)bytes[0]) |
        ((uint32)bytes[1] << 8) |
        ((uint32)bytes[2] << 16) |
        ((uint32)bytes[3] << 24);
}

static float32 readFloat32Le(const uint8 *bytes)
{
    uint32 raw = readU32Le(bytes);
    float32 value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static boolean isFiniteFloat(float32 value)
{
    return value == value && value < 3.402823466e+38f && value > -3.402823466e+38f;
}

static float32 absFloat(float32 value)
{
    return (value < 0.0f) ? -value : value;
}

static float32 clampFloat(float32 value, float32 minValue, float32 maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    if (value > maxValue)
    {
        return maxValue;
    }
    return value;
}

static boolean sequenceNewer(uint32 candidate, uint32 reference)
{
    uint32 difference = candidate - reference;
    return difference != 0u && difference < 0x80000000u;
}

static uint32 crc32Ieee(const uint8 *data, uint16 length)
{
    uint32 crc = 0xFFFFFFFFu;
    for (uint16 index = 0u; index < length; index++)
    {
        crc ^= data[index];
        for (uint8 bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static uint16 servoPeriodTicks(void)
{
    uint32 periodTicks = FRONT_STEERING_SERVO_PERIOD_TICKS;

    if (periodTicks == 0u)
    {
        return 1u;
    }
    if (periodTicks > 0xFFFFu)
    {
        return 0xFFFFu;
    }
    return (uint16)periodTicks;
}

static uint16 pulseUsToTicks(uint32 pulseUs)
{
    uint32 periodTicks = (uint32)servoPeriodTicks();
    uint32 ticks = (periodTicks * pulseUs) / FRONT_STEERING_SERVO_PERIOD_US;
    return (uint16)((ticks > periodTicks) ? periodTicks : ticks);
}

static uint32 angleToPulseUs(float32 angleRad)
{
    float32 normalized = clampFloat(angleRad / FRONT_STEERING_MAX_ABS_RAD, -1.0f, 1.0f);
#if FRONT_STEERING_SERVO_INVERT
    normalized = -normalized;
#endif

    if (normalized >= 0.0f)
    {
        float32 span = (float32)(FRONT_STEERING_SERVO_CENTER_PULSE_US - FRONT_STEERING_SERVO_LEFT_PULSE_US);
        return FRONT_STEERING_SERVO_CENTER_PULSE_US - (uint32)(normalized * span + 0.5f);
    }

    float32 span = (float32)(FRONT_STEERING_SERVO_RIGHT_PULSE_US - FRONT_STEERING_SERVO_CENTER_PULSE_US);
    return FRONT_STEERING_SERVO_CENTER_PULSE_US + (uint32)((-normalized) * span + 0.5f);
}

static Ifx_GTM_TOM *servoTom(void)
{
    return &MODULE_GTM.TOM[FRONT_STEERING_SERVO_TOM];
}

static Ifx_GTM_TOM_TGC *servoTgc(Ifx_GTM_TOM *tom)
{
    uint32 tgcIndex = (FRONT_STEERING_SERVO_TOM_CHANNEL <= IfxGtm_Tom_Ch_7) ? 0u : 1u;
    return IfxGtm_Tom_Ch_getTgcPointer(tom, tgcIndex);
}

static void applyServoAngle(float32 angleRad)
{
    uint32 pulseUs = angleToPulseUs(angleRad);
    uint16 dutyTicks = pulseUsToTicks(pulseUs);
    Ifx_GTM_TOM *tom = servoTom();
    Ifx_GTM_TOM_TGC *tgc = servoTgc(tom);

    IfxGtm_Tom_Ch_setCompareShadow(tom, FRONT_STEERING_SERVO_TOM_CHANNEL,
        servoPeriodTicks(), dutyTicks);
    IfxGtm_Tom_Tgc_enableChannelUpdate(tgc, FRONT_STEERING_SERVO_TOM_CHANNEL, TRUE);
    IfxGtm_Tom_Tgc_setChannelForceUpdate(tgc, FRONT_STEERING_SERVO_TOM_CHANNEL, TRUE, FALSE);
    IfxGtm_Tom_Tgc_trigger(tgc);
}

static void initServoPwm(void)
{
    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);

    Ifx_GTM_TOM *tom = servoTom();
    Ifx_GTM_TOM_TGC *tgc = servoTgc(tom);
    uint16 periodTicks = servoPeriodTicks();
    uint16 centerTicks = pulseUsToTicks(FRONT_STEERING_SERVO_CENTER_PULSE_US);

    IfxGtm_Tom_Ch_setClockSource(tom, FRONT_STEERING_SERVO_TOM_CHANNEL, IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0);
    IfxGtm_Tom_Ch_setSignalLevel(tom, FRONT_STEERING_SERVO_TOM_CHANNEL, Ifx_ActiveState_high);
    IfxGtm_Tom_Ch_setResetSource(tom, FRONT_STEERING_SERVO_TOM_CHANNEL, IfxGtm_Tom_Ch_ResetEvent_onCm0);
    IfxGtm_Tom_Ch_setCounterMode(tom, FRONT_STEERING_SERVO_TOM_CHANNEL, IfxGtm_Tom_Ch_CounterMode_up);
    IfxGtm_Tom_Ch_setCounterValue(tom, FRONT_STEERING_SERVO_TOM_CHANNEL, 0u);
    IfxGtm_Tom_Ch_setCompareShadow(tom, FRONT_STEERING_SERVO_TOM_CHANNEL, periodTicks, centerTicks);
    IfxGtm_Tom_Tgc_enableChannelUpdate(tgc, FRONT_STEERING_SERVO_TOM_CHANNEL, TRUE);
    IfxGtm_Tom_Tgc_setChannelForceUpdate(tgc, FRONT_STEERING_SERVO_TOM_CHANNEL, TRUE, TRUE);

    IfxGtm_PinMap_setTomTout(&FRONT_STEERING_SERVO_PIN,
        IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);

    IfxGtm_Tom_Tgc_enableChannel(tgc, FRONT_STEERING_SERVO_TOM_CHANNEL, TRUE, FALSE);
    IfxGtm_Tom_Tgc_enableChannelOutput(tgc, FRONT_STEERING_SERVO_TOM_CHANNEL, TRUE, FALSE);
    IfxGtm_Tom_Tgc_trigger(tgc);
    IfxGtm_Tom_Tgc_setChannelForceUpdate(tgc, FRONT_STEERING_SERVO_TOM_CHANNEL, TRUE, FALSE);
}

static void rejectPacket(void)
{
    g_state.rejectedPackets++;
}

static void acceptTarget(uint32 sequence, float32 angleRad, float32 maxRateRadS, boolean commandValid,
    boolean emergencyCenter, uint32 nowMs)
{
    g_state.initialized = TRUE;
    g_state.lastSequence = sequence;
    g_state.lastPacketMs = nowMs;
    g_state.maxRateRadS = clampFloat(maxRateRadS, 0.05f, 10.0f);
    g_state.emergencyCenter = emergencyCenter;
    g_state.commandValid = commandValid && !emergencyCenter;
    g_state.linkValid = TRUE;
    g_state.targetAngleRad = g_state.commandValid ? clampFloat(angleRad,
        -FRONT_STEERING_MAX_ABS_RAD, FRONT_STEERING_MAX_ABS_RAD) : 0.0f;
    g_state.acceptedPackets++;
}

boolean FrontSteeringNode_acceptHpscPacket(const uint8 *payload, uint16 length, uint32 nowMs)
{
    if (payload == 0 || length != HPSC_PACKET_SIZE)
    {
        rejectPacket();
        return FALSE;
    }

    if (payload[0] != 'H' || payload[1] != 'P' || payload[2] != 'S' || payload[3] != 'C')
    {
        rejectPacket();
        return FALSE;
    }

    uint8 version = payload[4];
    uint8 controlMode = payload[5];
    uint8 flags = payload[6];
    uint8 headerSize = payload[7];
    uint32 sequence = readU32Le(&payload[8]);
    float32 angleRad = readFloat32Le(&payload[20]);
    float32 maxRateRadS = readFloat32Le(&payload[24]);
    uint16 reserved16 = readU16Le(&payload[30]);
    uint32 reserved32 = readU32Le(&payload[32]);
    uint32 receivedCrc = readU32Le(&payload[36]);
    uint32 calculatedCrc = crc32Ieee(payload, 36u);

    if (version != HPSC_VERSION || headerSize != HPSC_HEADER_SIZE ||
        reserved16 != 0u || reserved32 != 0u || receivedCrc != calculatedCrc)
    {
        rejectPacket();
        return FALSE;
    }

    if (g_state.initialized && !sequenceNewer(sequence, g_state.lastSequence))
    {
        return FALSE;
    }

    boolean emergencyCenter = (flags & HPSC_FLAG_EMERGENCY_CENTER) != 0u;
    boolean steeringValid = (flags & HPSC_FLAG_STEERING_VALID) != 0u;
    boolean upstreamValid = (flags & HPSC_FLAG_UPSTREAM_VALID) != 0u;
    boolean commandValid = steeringValid && upstreamValid &&
        controlMode == HPSC_CONTROL_STEERING_ANGLE &&
        isFiniteFloat(angleRad) &&
        isFiniteFloat(maxRateRadS) &&
        absFloat(angleRad) <= FRONT_STEERING_MAX_ABS_RAD &&
        maxRateRadS > 0.0f;

    if (emergencyCenter)
    {
        acceptTarget(sequence, 0.0f, FRONT_STEERING_DEFAULT_MAX_RATE_RAD_S, FALSE, TRUE, nowMs);
        return TRUE;
    }

    if (!commandValid)
    {
        if (controlMode == HPSC_CONTROL_DISABLED)
        {
            acceptTarget(sequence, 0.0f, FRONT_STEERING_DEFAULT_MAX_RATE_RAD_S, FALSE, FALSE, nowMs);
            return TRUE;
        }
        rejectPacket();
        return FALSE;
    }

    acceptTarget(sequence, angleRad, maxRateRadS, TRUE, FALSE, nowMs);
    return TRUE;
}

boolean FrontSteeringNode_acceptEthernetFrame(const uint8 *frame, uint16 length, uint32 nowMs)
{
    if (frame == 0 || length < (14u + 20u + 8u + HPSC_PACKET_SIZE))
    {
        return FALSE;
    }

    uint16 etherType = readU16Be(&frame[12]);
    if (etherType != ETHERTYPE_IPV4)
    {
        return FALSE;
    }

    const uint8 *ip = &frame[14];
    uint8 ihlBytes = (uint8)((ip[0] & 0x0Fu) * 4u);
    if ((ip[0] >> 4) != 4u || ihlBytes < 20u || ip[9] != IPV4_PROTOCOL_UDP)
    {
        return FALSE;
    }

    uint16 totalLength = readU16Be(&ip[2]);
    if (totalLength < (uint16)(ihlBytes + 8u + HPSC_PACKET_SIZE))
    {
        return FALSE;
    }

    if ((uint32)14u + totalLength > length)
    {
        return FALSE;
    }

    const uint8 *udp = &ip[ihlBytes];
    uint16 dstPort = readU16Be(&udp[2]);
    uint16 udpLength = readU16Be(&udp[4]);
    if (dstPort != FRONT_STEERING_UDP_PORT || udpLength < (uint16)(8u + HPSC_PACKET_SIZE))
    {
        return FALSE;
    }

    return FrontSteeringNode_acceptHpscPacket(&udp[8], HPSC_PACKET_SIZE, nowMs);
}

void FrontSteeringNode_init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.maxRateRadS = FRONT_STEERING_DEFAULT_MAX_RATE_RAD_S;
    g_state.lastApplyMs = front_nowMs();
    initServoPwm();
    applyServoAngle(0.0f);
}

void FrontSteeringNode_runOnce(void)
{
    uint32 nowMs = front_nowMs();
    uint32 ageMs = nowMs - g_state.lastPacketMs;

    if (!g_state.initialized || ageMs > FRONT_STEERING_WATCHDOG_MS)
    {
        g_state.linkValid = FALSE;
        g_state.commandValid = FALSE;
        g_state.emergencyCenter = TRUE;
        g_state.targetAngleRad = 0.0f;
    }

    uint32 dtMs = nowMs - g_state.lastApplyMs;
    g_state.lastApplyMs = nowMs;
    float32 maxStep = g_state.maxRateRadS * ((float32)dtMs / 1000.0f);
    if (maxStep < 0.001f)
    {
        maxStep = 0.001f;
    }

    float32 error = g_state.targetAngleRad - g_state.appliedAngleRad;
    if (error > maxStep)
    {
        error = maxStep;
    }
    else if (error < -maxStep)
    {
        error = -maxStep;
    }

    g_state.appliedAngleRad = clampFloat(g_state.appliedAngleRad + error,
        -FRONT_STEERING_MAX_ABS_RAD, FRONT_STEERING_MAX_ABS_RAD);
    applyServoAngle(g_state.appliedAngleRad);
}

FrontSteeringNode_Status FrontSteeringNode_getStatus(void)
{
    uint32 nowMs = front_nowMs();
    FrontSteeringNode_Status status;
    status.linkValid = g_state.linkValid;
    status.commandValid = g_state.commandValid;
    status.emergencyCenter = g_state.emergencyCenter;
    status.targetAngleRad = g_state.targetAngleRad;
    status.appliedAngleRad = g_state.appliedAngleRad;
    status.lastSequence = g_state.lastSequence;
    status.acceptedPackets = g_state.acceptedPackets;
    status.rejectedPackets = g_state.rejectedPackets;
    status.lastPacketAgeMs = g_state.initialized ? (nowMs - g_state.lastPacketMs) : 0xFFFFFFFFu;
    return status;
}
