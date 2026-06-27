#include "AebSensorNode.h"

#include <string.h>

#include "IfxPort.h"
#include "Stm/Std/IfxStm.h"
#include "Geth/Eth/IfxGeth_Eth.h"

#define AEB_VALID_ULTRA_LEFT                (1u << 0)
#define AEB_VALID_ULTRA_RIGHT               (1u << 1)

#define AEB_ETH_TX_BUFFER_SIZE              (256u)
#define AEB_ETH_RX_BUFFER_SIZE              (256u)
#define AEB_ETH_MIN_FRAME_SIZE              (60u)
#define AEB_ETH_HEADER_SIZE                 (14u)
#define AEB_IPV4_HEADER_SIZE                (20u)
#define AEB_UDP_HEADER_SIZE                 (8u)
#define AEB_REAR_STATUS_PAYLOAD_SIZE        (11u)

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
    AEB_ULTRA_LEFT_TRIG_PORT,
    AEB_ULTRA_LEFT_TRIG_PIN,
    AEB_ULTRA_LEFT_ECHO_PORT,
    AEB_ULTRA_LEFT_ECHO_PIN,
    {0u, 0u, 0u},
    0u,
    0u,
    0u,
    0u
};

static AebUltrasonicSensor g_rightUltrasonic = {
    AEB_ULTRA_RIGHT_TRIG_PORT,
    AEB_ULTRA_RIGHT_TRIG_PIN,
    AEB_ULTRA_RIGHT_ECHO_PORT,
    AEB_ULTRA_RIGHT_ECHO_PIN,
    {0u, 0u, 0u},
    0u,
    0u,
    0u,
    0u
};

static IfxGeth_Eth g_geth;
IFX_ALIGN(4) static uint8 g_ethTxBuffer[AEB_ETH_TX_BUFFER_SIZE * IFXGETH_MAX_TX_DESCRIPTORS];
IFX_ALIGN(4) static uint8 g_ethRxBuffer[AEB_ETH_RX_BUFFER_SIZE * IFXGETH_MAX_RX_DESCRIPTORS];

static const uint8 g_srcMac[6] = {
    AEB_ETH_SRC_MAC0,
    AEB_ETH_SRC_MAC1,
    AEB_ETH_SRC_MAC2,
    AEB_ETH_SRC_MAC3,
    AEB_ETH_SRC_MAC4,
    AEB_ETH_SRC_MAC5
};

/* HPVC MAC is not configured here. Use L2 broadcast so a PC/HPVC listener can receive immediately. */
static const uint8 g_dstMac[6] = {0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu};
static const uint8 g_srcIp[4] = {AEB_ETH_SRC_IP0, AEB_ETH_SRC_IP1, AEB_ETH_SRC_IP2, AEB_ETH_SRC_IP3};
static const uint8 g_dstIp[4] = {AEB_ETH_DST_IP0, AEB_ETH_DST_IP1, AEB_ETH_DST_IP2, AEB_ETH_DST_IP3};

static AebSensorNode_Frame g_latestFrame;
static uint8 g_motorState = AEB_MOTOR_STATE_DISABLED;

static uint32 aeb_nowTicks(void)
{
    return IfxStm_getLower(&MODULE_STM0);
}

static uint32 aeb_ticksFromUs(uint32 us)
{
    sint32 ticks = IfxStm_getTicksFromMicroseconds(&MODULE_STM0, us);
    return (ticks <= 0) ? 1u : (uint32)ticks;
}

static uint32 aeb_ticksFromMs(uint32 ms)
{
    sint32 ticks = IfxStm_getTicksFromMilliseconds(&MODULE_STM0, ms);
    return (ticks <= 0) ? 1u : (uint32)ticks;
}

static uint32 aeb_ticksToUs(uint32 ticks)
{
    float32 stmFrequency = IfxStm_getFrequency(&MODULE_STM0);
    return (uint32)(((float32)ticks * 1000000.0f) / stmFrequency);
}

static uint32 aeb_ticksToMs(uint32 ticks)
{
    float32 stmFrequency = IfxStm_getFrequency(&MODULE_STM0);
    return (uint32)(((float32)ticks * 1000.0f) / stmFrequency);
}

static void aeb_delayUs(uint32 us)
{
    IfxStm_waitTicks(&MODULE_STM0, aeb_ticksFromUs(us));
}

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
    if (lastValidTick == 0u)
    {
        return FALSE;
    }

    return (aeb_nowTicks() - lastValidTick) <= aeb_ticksFromMs(staleMs);
}

static uint16 aeb_median3(const uint16 *samples, uint8 count)
{
    if (count == 0u)
    {
        return 0u;
    }

    if (count == 1u)
    {
        return samples[0];
    }

    if (count == 2u)
    {
        return (uint16)((samples[0] + samples[1]) / 2u);
    }

    uint16 a = samples[0];
    uint16 b = samples[1];
    uint16 c = samples[2];

    if (a > b)
    {
        uint16 t = a;
        a = b;
        b = t;
    }

    if (b > c)
    {
        uint16 t = b;
        b = c;
        c = t;
    }

    if (a > b)
    {
        uint16 t = a;
        a = b;
        b = t;
    }

    return b;
}

static boolean aeb_waitPinState(Ifx_P *port, uint8 pin, boolean state, uint32 timeoutUs)
{
    uint32 start = aeb_nowTicks();
    uint32 timeoutTicks = aeb_ticksFromUs(timeoutUs);

    while (IfxPort_getPinState(port, pin) != state)
    {
        if ((aeb_nowTicks() - start) >= timeoutTicks)
        {
            return FALSE;
        }
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

    if (!aeb_waitPinState(sensor->echoPort, sensor->echoPin, TRUE, AEB_ULTRA_ECHO_TIMEOUT_US))
    {
        return FALSE;
    }

    uint32 pulseStart = aeb_nowTicks();
    if (!aeb_waitPinState(sensor->echoPort, sensor->echoPin, FALSE, AEB_ULTRA_ECHO_TIMEOUT_US))
    {
        return FALSE;
    }

    uint32 pulseUs = aeb_ticksToUs(aeb_nowTicks() - pulseStart);
    uint32 distanceCmX10 = (pulseUs * 1715u + 5000u) / 10000u;

    if ((distanceCmX10 < AEB_ULTRA_MIN_CM_X10) || (distanceCmX10 > AEB_ULTRA_MAX_CM_X10))
    {
        return FALSE;
    }

    *cmX10 = (uint16)distanceCmX10;
    return TRUE;
}

static boolean aeb_ultrasonicUpdate(AebUltrasonicSensor *sensor)
{
    uint16 rawCmX10;

    if (aeb_ultrasonicReadCmX10(sensor, &rawCmX10))
    {
        sensor->samples[sensor->sampleIndex] = rawCmX10;
        sensor->sampleIndex = (uint8)((sensor->sampleIndex + 1u) % 3u);

        if (sensor->sampleCount < 3u)
        {
            sensor->sampleCount++;
        }

        sensor->filteredCmX10 = aeb_median3(sensor->samples, sensor->sampleCount);
        sensor->lastValidTick = aeb_nowTicks();
    }

    return aeb_isFresh(sensor->lastValidTick, AEB_ULTRA_STALE_MS);
}

static uint16 aeb_checksum16(const uint8 *data, uint16 length)
{
    uint32 sum = 0u;

    while (length > 1u)
    {
        sum += ((uint16)data[0] << 8) | data[1];
        data += 2;
        length -= 2u;
    }

    if (length != 0u)
    {
        sum += ((uint16)data[0] << 8);
    }

    while ((sum >> 16) != 0u)
    {
        sum = (sum & 0xffffu) + (sum >> 16);
    }

    return (uint16)(~sum);
}

static void aeb_put16be(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value >> 8);
    buffer[1] = (uint8)value;
}

static void aeb_putFloat32le(uint8 *buffer, float32 value)
{
    union
    {
        float32 f;
        uint8   b[4];
    } raw;

    raw.f = value;
    buffer[0] = raw.b[0];
    buffer[1] = raw.b[1];
    buffer[2] = raw.b[2];
    buffer[3] = raw.b[3];
}

static float32 aeb_cmX10ToMeters(uint16 cmX10, boolean valid)
{
    return valid ? ((float32)cmX10 * 0.001f) : 0.0f;
}

static uint16 aeb_buildPacket(uint8 *txBuffer, const AebSensorNode_Frame *frame)
{
    memset(txBuffer, 0, AEB_ETH_TX_BUFFER_SIZE);

    memcpy(&txBuffer[0], g_dstMac, 6u);
    memcpy(&txBuffer[6], g_srcMac, 6u);
    txBuffer[12] = 0x08u;
    txBuffer[13] = 0x00u;

    uint8 *ip = &txBuffer[AEB_ETH_HEADER_SIZE];
    uint16 ipTotalLength = AEB_IPV4_HEADER_SIZE + AEB_UDP_HEADER_SIZE + AEB_REAR_STATUS_PAYLOAD_SIZE;
    ip[0] = 0x45u;
    ip[1] = 0x00u;
    aeb_put16be(&ip[2], ipTotalLength);
    aeb_put16be(&ip[4], (uint16)(frame->sequence & 0xffffu));
    aeb_put16be(&ip[6], 0x0000u);
    ip[8] = 64u;
    ip[9] = 17u;
    memcpy(&ip[12], g_srcIp, 4u);
    memcpy(&ip[16], g_dstIp, 4u);
    aeb_put16be(&ip[10], aeb_checksum16(ip, AEB_IPV4_HEADER_SIZE));

    uint8 *udp = &ip[AEB_IPV4_HEADER_SIZE];
    aeb_put16be(&udp[0], AEB_ETH_SRC_PORT);
    aeb_put16be(&udp[2], AEB_ETH_DST_PORT);
    aeb_put16be(&udp[4], AEB_UDP_HEADER_SIZE + AEB_REAR_STATUS_PAYLOAD_SIZE);
    aeb_put16be(&udp[6], 0x0000u);

    boolean leftValid = ((frame->validMask & AEB_VALID_ULTRA_LEFT) != 0u);
    boolean rightValid = ((frame->validMask & AEB_VALID_ULTRA_RIGHT) != 0u);

    uint8 *payload = &udp[AEB_UDP_HEADER_SIZE];
    aeb_putFloat32le(&payload[0], aeb_cmX10ToMeters(frame->ultrasonicLeftCmX10, leftValid));
    aeb_putFloat32le(&payload[4], aeb_cmX10ToMeters(frame->ultrasonicRightCmX10, rightValid));
    payload[8] = frame->validMask;
    payload[9] = frame->motorState;
    payload[10] = (uint8)(frame->sequence & 0x0Fu);

    uint16 frameLength = AEB_ETH_HEADER_SIZE + ipTotalLength;
    return (frameLength < AEB_ETH_MIN_FRAME_SIZE) ? AEB_ETH_MIN_FRAME_SIZE : frameLength;
}

static void aeb_ethernetInit(void)
{
    static const IfxGeth_Eth_RmiiPins rmiiPins = {
        &IfxGeth_CRSDVA_P11_11_IN,
        &IfxGeth_REFCLKA_P11_12_IN,
        &IfxGeth_RXD0A_P11_10_IN,
        &IfxGeth_RXD1A_P11_9_IN,
        &IfxGeth_MDIO_P21_3_INOUT,
        &IfxGeth_TXD0_P11_3_OUT,
        &IfxGeth_MDC_P21_2_OUT,
        &IfxGeth_TXD1_P11_2_OUT,
        &IfxGeth_TXEN_P11_6_OUT
    };

    IfxGeth_Eth_Config config;
    IfxGeth_Eth_initModuleConfig(&config, &MODULE_GETH);

    config.phyInterfaceMode = IfxGeth_PhyInterfaceMode_rmii;
    config.pins.rmiiPins = &rmiiPins;
    config.mac.lineSpeed = IfxGeth_LineSpeed_100Mbps;
    config.mac.duplexMode = IfxGeth_DuplexMode_fullDuplex;
    memcpy(config.mac.macAddress, g_srcMac, sizeof(g_srcMac));

    config.dma.txChannel[0].txBuffer1StartAddress = (uint32 *)&g_ethTxBuffer[0];
    config.dma.txChannel[0].txBuffer1Size = AEB_ETH_TX_BUFFER_SIZE;
    config.dma.rxChannel[0].rxBuffer1StartAddress = (uint32 *)&g_ethRxBuffer[0];
    config.dma.rxChannel[0].rxBuffer1Size = AEB_ETH_RX_BUFFER_SIZE;

    IfxGeth_Eth_initModule(&g_geth, &config);
    IfxGeth_Eth_startTransmitters(&g_geth, 1u);
    IfxGeth_Eth_startReceivers(&g_geth, 1u);
}

static void aeb_sendFrame(const AebSensorNode_Frame *frame)
{
    uint8 *txBuffer = (uint8 *)IfxGeth_Eth_waitTransmitBuffer(&g_geth, IfxGeth_TxDmaChannel_0);
    uint16 packetLength = aeb_buildPacket(txBuffer, frame);

    IfxGeth_dma_clearInterruptFlag(g_geth.gethSFR, IfxGeth_DmaChannel_0, IfxGeth_DmaInterruptFlag_transmitInterrupt);
    IfxGeth_Eth_sendTransmitBuffer(&g_geth, packetLength, IfxGeth_TxDmaChannel_0);

    uint32 waitStart = aeb_nowTicks();
    uint32 waitTicks = aeb_ticksFromMs(2u);
    while (!IfxGeth_dma_isInterruptFlagSet(g_geth.gethSFR, IfxGeth_DmaChannel_0, IfxGeth_DmaInterruptFlag_transmitInterrupt))
    {
        if ((aeb_nowTicks() - waitStart) > waitTicks)
        {
            break;
        }
    }

    IfxGeth_dma_clearInterruptFlag(g_geth.gethSFR, IfxGeth_DmaChannel_0, IfxGeth_DmaInterruptFlag_transmitInterrupt);
}

void AebSensorNode_init(void)
{
    memset(&g_latestFrame, 0, sizeof(g_latestFrame));
    g_motorState = AEB_MOTOR_STATE_DISABLED;

    aeb_ultrasonicInit(&g_leftUltrasonic);
    aeb_ultrasonicInit(&g_rightUltrasonic);
    aeb_ethernetInit();
}

void AebSensorNode_runOnce(void)
{
    uint32 cycleStart = aeb_nowTicks();

    boolean leftValid = aeb_ultrasonicUpdate(&g_leftUltrasonic);
    aeb_delayUs(AEB_ULTRA_CROSSTALK_DELAY_US);
    boolean rightValid = aeb_ultrasonicUpdate(&g_rightUltrasonic);

    g_latestFrame.sequence++;
    g_latestFrame.timestampMs = aeb_ticksToMs(aeb_nowTicks());
    g_latestFrame.ultrasonicLeftCmX10 = g_leftUltrasonic.filteredCmX10;
    g_latestFrame.ultrasonicRightCmX10 = g_rightUltrasonic.filteredCmX10;
    g_latestFrame.validMask = 0u;
    g_latestFrame.motorState = g_motorState;

    if (leftValid)
    {
        g_latestFrame.validMask |= AEB_VALID_ULTRA_LEFT;
    }

    if (rightValid)
    {
        g_latestFrame.validMask |= AEB_VALID_ULTRA_RIGHT;
    }

    aeb_sendFrame(&g_latestFrame);
    aeb_waitUntilElapsedMs(cycleStart, AEB_NODE_LOOP_PERIOD_MS);
}

void AebSensorNode_setMotorState(uint8 motorState)
{
    if (motorState <= AEB_MOTOR_STATE_FAULT)
    {
        g_motorState = motorState;
    }
}

AebSensorNode_Frame AebSensorNode_getLatestFrame(void)
{
    return g_latestFrame;
}
