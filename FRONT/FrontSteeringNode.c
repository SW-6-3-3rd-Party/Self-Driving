#include "FrontSteeringNode.h"

#include <string.h>

#include "IfxPort.h"
#include "IfxStm.h"

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

/* Servo signal pin: P02.3, driven by software GPIO pulses. */
#ifndef FRONT_STEERING_SERVO_PORT
#define FRONT_STEERING_SERVO_PORT              (&MODULE_P02)
#endif
#ifndef FRONT_STEERING_SERVO_PIN
#define FRONT_STEERING_SERVO_PIN               (3u)
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

static void front_delayUs(uint32 delayUs)
{
    sint32 ticks = IfxStm_getTicksFromMicroseconds(&MODULE_STM0, delayUs);

    if (ticks <= 0)
    {
        ticks = 1;
    }

    IfxStm_waitTicks(&MODULE_STM0, (uint32)ticks);
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

static void sendServoPulseUs(uint32 pulseUs)
{
    pulseUs = (uint32)clampFloat((float32)pulseUs,
        (float32)FRONT_STEERING_SERVO_MIN_PULSE_US,
        (float32)FRONT_STEERING_SERVO_MAX_PULSE_US);

    IfxPort_setPinHigh(FRONT_STEERING_SERVO_PORT, FRONT_STEERING_SERVO_PIN);
    front_delayUs(pulseUs);

    IfxPort_setPinLow(FRONT_STEERING_SERVO_PORT, FRONT_STEERING_SERVO_PIN);
    front_delayUs(FRONT_STEERING_SERVO_PERIOD_US - pulseUs);
}

static void applyServoAngle(float32 angleRad)
{
    uint32 pulseUs = angleToPulseUs(angleRad);
    sendServoPulseUs(pulseUs);
}

static void initServoPwm(void)
{
    IfxPort_setPinModeOutput(
        FRONT_STEERING_SERVO_PORT,
        FRONT_STEERING_SERVO_PIN,
        IfxPort_OutputMode_pushPull,
        IfxPort_OutputIdx_general
    );
    IfxPort_setPinLow(FRONT_STEERING_SERVO_PORT, FRONT_STEERING_SERVO_PIN);
}

void FrontSteeringNode_holdCurrentForMs(uint32 holdMs)
{
    uint32 pulseCount = (holdMs * 1000u + FRONT_STEERING_SERVO_PERIOD_US - 1u) /
        FRONT_STEERING_SERVO_PERIOD_US;

    if (pulseCount == 0u)
    {
        pulseCount = 1u;
    }

    for (uint32 index = 0u; index < pulseCount; index++)
    {
        applyServoAngle(g_state.appliedAngleRad);
    }
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
    if (frame == 0 || length < HPSC_PACKET_SIZE)
    {
        return FALSE;
    }

    if (length >= (14u + 20u + 8u + HPSC_PACKET_SIZE))
    {
        uint16 etherType = readU16Be(&frame[12]);
        if (etherType == ETHERTYPE_IPV4)
        {
            const uint8 *ip = &frame[14];
            uint8 ihlBytes = (uint8)((ip[0] & 0x0Fu) * 4u);
            if ((ip[0] >> 4) == 4u && ihlBytes >= 20u && ip[9] == IPV4_PROTOCOL_UDP)
            {
                uint16 totalLength = readU16Be(&ip[2]);
                if (totalLength >= (uint16)(ihlBytes + 8u + HPSC_PACKET_SIZE) &&
                    ((uint32)14u + totalLength) <= length)
                {
                    const uint8 *udp = &ip[ihlBytes];
                    uint16 dstPort = readU16Be(&udp[2]);
                    uint16 udpLength = readU16Be(&udp[4]);
                    if (dstPort == FRONT_STEERING_UDP_PORT &&
                        udpLength >= (uint16)(8u + HPSC_PACKET_SIZE) &&
                        FrontSteeringNode_acceptHpscPacket(&udp[8], HPSC_PACKET_SIZE, nowMs))
                    {
                        return TRUE;
                    }
                }
            }
        }

        if (length >= (20u + 8u + HPSC_PACKET_SIZE) &&
            (frame[0] >> 4) == 4u &&
            frame[9] == IPV4_PROTOCOL_UDP)
        {
            uint8 ihlBytes = (uint8)((frame[0] & 0x0Fu) * 4u);
            if (ihlBytes >= 20u)
            {
                const uint8 *udp = &frame[ihlBytes];
                uint16 dstPort = readU16Be(&udp[2]);
                uint16 udpLength = readU16Be(&udp[4]);
                if (dstPort == FRONT_STEERING_UDP_PORT &&
                    udpLength >= (uint16)(8u + HPSC_PACKET_SIZE) &&
                    FrontSteeringNode_acceptHpscPacket(&udp[8], HPSC_PACKET_SIZE, nowMs))
                {
                    return TRUE;
                }
            }
        }
    }

    for (uint16 offset = 0u; offset <= (uint16)(length - HPSC_PACKET_SIZE); offset++)
    {
        if (frame[offset] == 'H' &&
            frame[offset + 1u] == 'P' &&
            frame[offset + 2u] == 'S' &&
            frame[offset + 3u] == 'C' &&
            FrontSteeringNode_acceptHpscPacket(&frame[offset], HPSC_PACKET_SIZE, nowMs))
        {
            return TRUE;
        }
    }

    return FALSE;
}

void FrontSteeringNode_init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.maxRateRadS = FRONT_STEERING_DEFAULT_MAX_RATE_RAD_S;
    g_state.lastApplyMs = front_nowMs();
    initServoPwm();
    g_state.appliedAngleRad = 0.0f;
    FrontSteeringNode_holdCurrentForMs(FRONT_STEERING_BOOT_CENTER_MS);
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
