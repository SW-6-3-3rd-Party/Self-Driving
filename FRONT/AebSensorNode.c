#include "AebSensorNode.h"
#include "FrontSteeringNode.h"

#include <string.h>

#include "IfxPort.h"
#include "IfxStm.h"
#include "Geth/Eth/IfxGeth_Eth.h"

#define AEB_VALID_ULTRA_LEFT                  (1u << 0)
#define AEB_VALID_ULTRA_RIGHT                 (1u << 1)
#define AEB_VALID_TOF                         (1u << 2)

#define AEB_ETH_TX_BUFFER_SIZE                (256u)
#define AEB_ETH_RX_BUFFER_SIZE                (256u)
#define AEB_ETH_MIN_FRAME_SIZE                (60u)
#define AEB_ETH_HEADER_SIZE                   (14u)
#define AEB_IPV4_HEADER_SIZE                  (20u)
#define AEB_UDP_HEADER_SIZE                   (8u)
#define AEB_PAYLOAD_SIZE                      (22u)

#define AEB_TOF_DIAG_NOT_TRIED                (0x00u)
#define AEB_TOF_DIAG_MODEL_READ_FAIL          (0xA1u)
#define AEB_TOF_DIAG_UNEXPECTED_MODEL         (0xA2u)
#define AEB_TOF_DIAG_INIT_FAIL                (0xA3u)
#define AEB_TOF_DIAG_CALIBRATION_FAIL         (0xA4u)
#define AEB_TOF_DIAG_VL53L0X_PRESENT          (0x03u)
#define AEB_TOF_DIAG_MEASURE_START_FAIL       (0xB1u)
#define AEB_TOF_DIAG_MEASURE_STATUS_FAIL      (0xB2u)
#define AEB_TOF_DIAG_MEASURE_TIMEOUT          (0xB3u)
#define AEB_TOF_DIAG_RANGE_READ_FAIL          (0xB4u)
#define AEB_TOF_DIAG_RANGE_OUT_OF_LIMIT       (0xB5u)

#define AEB_ULTRA_DIAG_OK                     (0x0000u)
#define AEB_ULTRA_DIAG_NO_ECHO_RISE           (0xF001u)
#define AEB_ULTRA_DIAG_NO_ECHO_FALL           (0xF002u)
#define AEB_ULTRA_DIAG_RANGE_OUT_OF_LIMIT     (0xF003u)

#define VL53L0X_SYSRANGE_START                (0x00u)
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG        (0x01u)
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO  (0x0Au)
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR        (0x0Bu)
#define VL53L0X_RESULT_INTERRUPT_STATUS       (0x13u)
#define VL53L0X_RESULT_RANGE_MM               (0x1Eu)
#define VL53L0X_MSRC_CONFIG_CONTROL           (0x60u)
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH       (0x84u)
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP (0x89u)
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES    (0xB0u)
#define VL53L0X_GLOBAL_CONFIG_REF_START       (0xB6u)

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
    uint16  lastRawCmX10;
    uint16  lastDiag;
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
    0u,
    0u,
    0u
};

static boolean           g_tofPresent = FALSE;
static uint16            g_tofCmX10 = 0u;
static uint32            g_tofLastValidTick = 0u;
static uint32            g_tofNextInitTick = 0u;
static boolean           g_tofInitAttempted = FALSE;
static uint16            g_tofDiag = AEB_TOF_DIAG_NOT_TRIED;

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

static const uint8 g_dstMac[6] = {0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu};
static const uint8 g_srcIp[4] = {AEB_ETH_SRC_IP0, AEB_ETH_SRC_IP1, AEB_ETH_SRC_IP2, AEB_ETH_SRC_IP3};
static const uint8 g_dstIp[4] = {AEB_ETH_DST_IP0, AEB_ETH_DST_IP1, AEB_ETH_DST_IP2, AEB_ETH_DST_IP3};

static const uint8 g_vl53l0xDefaultTuning[][2] = {
    {0xFFu, 0x01u}, {0x00u, 0x00u}, {0xFFu, 0x00u}, {0x09u, 0x00u},
    {0x10u, 0x00u}, {0x11u, 0x00u}, {0x24u, 0x01u}, {0x25u, 0xFFu},
    {0x75u, 0x00u}, {0xFFu, 0x01u}, {0x4Eu, 0x2Cu}, {0x48u, 0x00u},
    {0x30u, 0x20u}, {0xFFu, 0x00u}, {0x30u, 0x09u}, {0x54u, 0x00u},
    {0x31u, 0x04u}, {0x32u, 0x03u}, {0x40u, 0x83u}, {0x46u, 0x25u},
    {0x60u, 0x00u}, {0x27u, 0x00u}, {0x50u, 0x06u}, {0x51u, 0x00u},
    {0x52u, 0x96u}, {0x56u, 0x08u}, {0x57u, 0x30u}, {0x61u, 0x00u},
    {0x62u, 0x00u}, {0x64u, 0x00u}, {0x65u, 0x00u}, {0x66u, 0xA0u},
    {0xFFu, 0x01u}, {0x22u, 0x32u}, {0x47u, 0x14u}, {0x49u, 0xFFu},
    {0x4Au, 0x00u}, {0xFFu, 0x00u}, {0x7Au, 0x0Au}, {0x7Bu, 0x00u},
    {0x78u, 0x21u}, {0xFFu, 0x01u}, {0x23u, 0x34u}, {0x42u, 0x00u},
    {0x44u, 0xFFu}, {0x45u, 0x26u}, {0x46u, 0x05u}, {0x40u, 0x40u},
    {0x0Eu, 0x06u}, {0x20u, 0x1Au}, {0x43u, 0x40u}, {0xFFu, 0x00u},
    {0x34u, 0x03u}, {0x35u, 0x44u}, {0xFFu, 0x01u}, {0x31u, 0x04u},
    {0x4Bu, 0x09u}, {0x4Cu, 0x05u}, {0x4Du, 0x04u}, {0xFFu, 0x00u},
    {0x44u, 0x00u}, {0x45u, 0x20u}, {0x47u, 0x08u}, {0x48u, 0x28u},
    {0x67u, 0x00u}, {0x70u, 0x04u}, {0x71u, 0x01u}, {0x72u, 0xFEu},
    {0x76u, 0x00u}, {0x77u, 0x00u}, {0xFFu, 0x01u}, {0x0Du, 0x01u},
    {0xFFu, 0x00u}, {0x80u, 0x01u}, {0x01u, 0xF8u}, {0xFFu, 0x01u},
    {0x8Eu, 0x01u}, {0x00u, 0x01u}, {0xFFu, 0x00u}, {0x80u, 0x00u}
};

static AebSensorNode_Frame g_latestFrame;

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

static void aeb_delayMs(uint32 ms)
{
    IfxStm_waitTicks(&MODULE_STM0, aeb_ticksFromMs(ms));
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

static boolean aeb_waitPinState(Ifx_P *port, uint8 pin, boolean state, uint32 timeoutUs, uint32 *elapsedTicks)
{
    uint32 start = aeb_nowTicks();
    uint32 timeoutTicks = aeb_ticksFromUs(timeoutUs);

    while (IfxPort_getPinState(port, pin) != state)
    {
        uint32 elapsed = aeb_nowTicks() - start;
        if (elapsed >= timeoutTicks)
        {
            if (elapsedTicks != NULL_PTR)
            {
                *elapsedTicks = elapsed;
            }
            return FALSE;
        }
    }

    if (elapsedTicks != NULL_PTR)
    {
        *elapsedTicks = aeb_nowTicks() - start;
    }

    return TRUE;
}

static void aeb_ultrasonicInit(AebUltrasonicSensor *sensor)
{
    IfxPort_setPinModeOutput(sensor->trigPort, sensor->trigPin, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeInput(sensor->echoPort, sensor->echoPin, IfxPort_InputMode_noPullDevice);
    IfxPort_setPinLow(sensor->trigPort, sensor->trigPin);
}

static boolean aeb_ultrasonicReadCmX10(AebUltrasonicSensor *sensor, uint16 *cmX10)
{
    uint32 pulseTicks;
    sensor->lastRawCmX10 = 0u;
    sensor->lastDiag = AEB_ULTRA_DIAG_OK;

    IfxPort_setPinLow(sensor->trigPort, sensor->trigPin);
    aeb_delayUs(2u);
    IfxPort_setPinHigh(sensor->trigPort, sensor->trigPin);
    aeb_delayUs(10u);
    IfxPort_setPinLow(sensor->trigPort, sensor->trigPin);

    if (!aeb_waitPinState(sensor->echoPort, sensor->echoPin, TRUE, AEB_ULTRA_ECHO_TIMEOUT_US, NULL_PTR))
    {
        sensor->lastDiag = AEB_ULTRA_DIAG_NO_ECHO_RISE;
        return FALSE;
    }

    uint32 pulseStart = aeb_nowTicks();
    if (!aeb_waitPinState(sensor->echoPort, sensor->echoPin, FALSE, AEB_ULTRA_ECHO_TIMEOUT_US, NULL_PTR))
    {
        sensor->lastDiag = AEB_ULTRA_DIAG_NO_ECHO_FALL;
        return FALSE;
    }
    pulseTicks = aeb_nowTicks() - pulseStart;

    uint32 pulseUs = aeb_ticksToUs(pulseTicks);
    uint32 distanceCmX10 = (pulseUs * 1715u + 5000u) / 10000u;
    sensor->lastRawCmX10 = (distanceCmX10 > 0xFFFFu) ? 0xFFFFu : (uint16)distanceCmX10;

    if ((distanceCmX10 < AEB_ULTRA_MIN_CM_X10) || (distanceCmX10 > AEB_ULTRA_MAX_CM_X10))
    {
        sensor->lastDiag = AEB_ULTRA_DIAG_RANGE_OUT_OF_LIMIT;
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

static void aeb_i2cReleaseSda(void)
{
    IfxPort_setPinModeInput(AEB_TOF_SDA_PORT, AEB_TOF_SDA_PIN, IfxPort_InputMode_pullUp);
}

static void aeb_i2cReleaseScl(void)
{
    IfxPort_setPinModeInput(AEB_TOF_SCL_PORT, AEB_TOF_SCL_PIN, IfxPort_InputMode_pullUp);
}

static void aeb_i2cDriveSdaLow(void)
{
    IfxPort_setPinLow(AEB_TOF_SDA_PORT, AEB_TOF_SDA_PIN);
    IfxPort_setPinModeOutput(AEB_TOF_SDA_PORT, AEB_TOF_SDA_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
}

static void aeb_i2cDriveSclLow(void)
{
    IfxPort_setPinLow(AEB_TOF_SCL_PORT, AEB_TOF_SCL_PIN);
    IfxPort_setPinModeOutput(AEB_TOF_SCL_PORT, AEB_TOF_SCL_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
}

static boolean aeb_i2cWaitSclHigh(void)
{
    uint32 start = aeb_nowTicks();
    uint32 timeoutTicks = aeb_ticksFromUs(AEB_TOF_I2C_SCL_TIMEOUT_US);

    while (!IfxPort_getPinState(AEB_TOF_SCL_PORT, AEB_TOF_SCL_PIN))
    {
        if ((aeb_nowTicks() - start) > timeoutTicks)
        {
            return FALSE;
        }
    }

    return TRUE;
}

static void aeb_i2cBusInit(void)
{
    aeb_i2cReleaseSda();
    aeb_i2cReleaseScl();
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);

    for (uint8 i = 0u; i < 9u; i++)
    {
        aeb_i2cDriveSclLow();
        aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
        aeb_i2cReleaseScl();
        (void)aeb_i2cWaitSclHigh();
        aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    }
}

static boolean aeb_i2cStart(void)
{
    aeb_i2cReleaseSda();
    aeb_i2cReleaseScl();
    if (!aeb_i2cWaitSclHigh())
    {
        return FALSE;
    }
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    aeb_i2cDriveSdaLow();
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    aeb_i2cDriveSclLow();
    return TRUE;
}

static boolean aeb_i2cStop(void)
{
    aeb_i2cDriveSdaLow();
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    aeb_i2cReleaseScl();
    if (!aeb_i2cWaitSclHigh())
    {
        return FALSE;
    }
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    aeb_i2cReleaseSda();
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    return TRUE;
}

static boolean aeb_i2cWriteByte(uint8 byte)
{
    for (sint8 bit = 7; bit >= 0; bit--)
    {
        if ((byte & (1u << bit)) != 0u)
        {
            aeb_i2cReleaseSda();
        }
        else
        {
            aeb_i2cDriveSdaLow();
        }

        aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
        aeb_i2cReleaseScl();
        if (!aeb_i2cWaitSclHigh())
        {
            return FALSE;
        }
        aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
        aeb_i2cDriveSclLow();
    }

    aeb_i2cReleaseSda();
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    aeb_i2cReleaseScl();
    if (!aeb_i2cWaitSclHigh())
    {
        return FALSE;
    }
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    boolean ack = !IfxPort_getPinState(AEB_TOF_SDA_PORT, AEB_TOF_SDA_PIN);
    aeb_i2cDriveSclLow();
    return ack;
}

static uint8 aeb_i2cReadByte(boolean ack)
{
    uint8 byte = 0u;

    aeb_i2cReleaseSda();

    for (sint8 bit = 7; bit >= 0; bit--)
    {
        aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
        aeb_i2cReleaseScl();
        if (!aeb_i2cWaitSclHigh())
        {
            return 0xffu;
        }

        if (IfxPort_getPinState(AEB_TOF_SDA_PORT, AEB_TOF_SDA_PIN))
        {
            byte |= (uint8)(1u << bit);
        }

        aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
        aeb_i2cDriveSclLow();
    }

    if (ack)
    {
        aeb_i2cDriveSdaLow();
    }
    else
    {
        aeb_i2cReleaseSda();
    }

    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    aeb_i2cReleaseScl();
    (void)aeb_i2cWaitSclHigh();
    aeb_delayUs(AEB_TOF_I2C_HALF_PERIOD_US);
    aeb_i2cDriveSclLow();
    aeb_i2cReleaseSda();

    return byte;
}

static boolean aeb_i2cWrite(const uint8 *data, uint8 size)
{
    uint8 address = (uint8)(AEB_TOF_I2C_ADDRESS_7BIT << 1);

    if (!aeb_i2cStart())
    {
        (void)aeb_i2cStop();
        return FALSE;
    }

    if (!aeb_i2cWriteByte(address))
    {
        (void)aeb_i2cStop();
        return FALSE;
    }

    for (uint8 i = 0u; i < size; i++)
    {
        if (!aeb_i2cWriteByte(data[i]))
        {
            (void)aeb_i2cStop();
            return FALSE;
        }
    }

    return aeb_i2cStop();
}

static boolean aeb_i2cWriteToRegister(uint8 reg, const uint8 *data, uint8 size)
{
    uint8 address = (uint8)(AEB_TOF_I2C_ADDRESS_7BIT << 1);

    if (!aeb_i2cStart())
    {
        (void)aeb_i2cStop();
        return FALSE;
    }

    if (!aeb_i2cWriteByte(address) || !aeb_i2cWriteByte(reg))
    {
        (void)aeb_i2cStop();
        return FALSE;
    }

    for (uint8 i = 0u; i < size; i++)
    {
        if (!aeb_i2cWriteByte(data[i]))
        {
            (void)aeb_i2cStop();
            return FALSE;
        }
    }

    return aeb_i2cStop();
}

static boolean aeb_i2cReadFromRegister(uint8 reg, uint8 *data, uint8 size)
{
    uint8 writeAddress = (uint8)(AEB_TOF_I2C_ADDRESS_7BIT << 1);
    uint8 readAddress = (uint8)(writeAddress | 1u);

    if (!aeb_i2cStart())
    {
        (void)aeb_i2cStop();
        return FALSE;
    }

    if (!aeb_i2cWriteByte(writeAddress) || !aeb_i2cWriteByte(reg))
    {
        (void)aeb_i2cStop();
        return FALSE;
    }

    if (!aeb_i2cStart())
    {
        (void)aeb_i2cStop();
        return FALSE;
    }

    if (!aeb_i2cWriteByte(readAddress))
    {
        (void)aeb_i2cStop();
        return FALSE;
    }

    for (uint8 i = 0u; i < size; i++)
    {
        data[i] = aeb_i2cReadByte(i < (uint8)(size - 1u));
    }

    return aeb_i2cStop();
}

static boolean aeb_vl53l0xWrite8(uint8 reg, uint8 value)
{
    uint8 data[2] = {reg, value};
    return aeb_i2cWrite(data, 2u);
}

static boolean aeb_vl53l0xRead8(uint8 reg, uint8 *value)
{
    return aeb_i2cReadFromRegister(reg, value, 1u);
}

static boolean aeb_vl53l0xRead16(uint8 reg, uint16 *value)
{
    uint8 data[2];

    if (!aeb_i2cReadFromRegister(reg, data, 2u))
    {
        return FALSE;
    }

    *value = ((uint16)data[0] << 8) | data[1];
    return TRUE;
}

static boolean aeb_vl53l0xWrite16(uint8 reg, uint16 value)
{
    uint8 data[2];

    data[0] = (uint8)(value >> 8);
    data[1] = (uint8)value;

    return aeb_i2cWriteToRegister(reg, data, 2u);
}

static boolean aeb_vl53l0xReadMulti(uint8 reg, uint8 *data, uint8 size)
{
    return aeb_i2cReadFromRegister(reg, data, size);
}

static boolean aeb_vl53l0xWriteMulti(uint8 reg, const uint8 *data, uint8 size)
{
    return aeb_i2cWriteToRegister(reg, data, size);
}

static void aeb_tofResetByXshut(void)
{
#if AEB_TOF_XSHUT_ENABLED
    IfxPort_setPinModeOutput(AEB_TOF_XSHUT_PORT, AEB_TOF_XSHUT_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinLow(AEB_TOF_XSHUT_PORT, AEB_TOF_XSHUT_PIN);
    aeb_delayMs(AEB_TOF_XSHUT_RESET_LOW_MS);
    IfxPort_setPinHigh(AEB_TOF_XSHUT_PORT, AEB_TOF_XSHUT_PIN);
    aeb_delayMs(AEB_TOF_BOOT_WAIT_MS);
#endif
}

static boolean aeb_vl53l0xApplyDefaultTuning(void)
{
    uint32 count = (uint32)(sizeof(g_vl53l0xDefaultTuning) / sizeof(g_vl53l0xDefaultTuning[0]));

    for (uint32 i = 0u; i < count; i++)
    {
        if (!aeb_vl53l0xWrite8(g_vl53l0xDefaultTuning[i][0], g_vl53l0xDefaultTuning[i][1]))
        {
            return FALSE;
        }
    }

    return TRUE;
}

static boolean aeb_vl53l0xGetSpadInfo(uint8 *count, boolean *typeIsAperture)
{
    uint8 value = 0u;

    if (!aeb_vl53l0xWrite8(0x80u, 0x01u) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x01u) ||
        !aeb_vl53l0xWrite8(0x00u, 0x00u) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x06u) ||
        !aeb_vl53l0xRead8(0x83u, &value) ||
        !aeb_vl53l0xWrite8(0x83u, (uint8)(value | 0x04u)) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x07u) ||
        !aeb_vl53l0xWrite8(0x81u, 0x01u) ||
        !aeb_vl53l0xWrite8(0x80u, 0x01u) ||
        !aeb_vl53l0xWrite8(0x94u, 0x6Bu) ||
        !aeb_vl53l0xWrite8(0x83u, 0x00u))
    {
        return FALSE;
    }

    uint32 startTick = aeb_nowTicks();
    uint32 timeoutTicks = aeb_ticksFromMs(100u);

    do
    {
        if (!aeb_vl53l0xRead8(0x83u, &value))
        {
            return FALSE;
        }
    } while ((value == 0u) && ((aeb_nowTicks() - startTick) < timeoutTicks));

    if (value == 0u)
    {
        return FALSE;
    }

    if (!aeb_vl53l0xWrite8(0x83u, 0x01u) ||
        !aeb_vl53l0xRead8(0x92u, &value))
    {
        return FALSE;
    }

    *count = (uint8)(value & 0x7Fu);
    *typeIsAperture = ((value & 0x80u) != 0u);

    if (!aeb_vl53l0xWrite8(0x81u, 0x00u) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x06u) ||
        !aeb_vl53l0xRead8(0x83u, &value) ||
        !aeb_vl53l0xWrite8(0x83u, (uint8)(value & (uint8)~0x04u)) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x01u) ||
        !aeb_vl53l0xWrite8(0x00u, 0x01u) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x00u) ||
        !aeb_vl53l0xWrite8(0x80u, 0x00u))
    {
        return FALSE;
    }

    return TRUE;
}

static boolean aeb_vl53l0xConfigureSpads(void)
{
    uint8 spadCount = 0u;
    boolean spadTypeIsAperture = FALSE;
    uint8 refSpadMap[6] = {0u, 0u, 0u, 0u, 0u, 0u};

    if (!aeb_vl53l0xGetSpadInfo(&spadCount, &spadTypeIsAperture) ||
        !aeb_vl53l0xReadMulti(VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES, refSpadMap, 6u) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x01u) ||
        !aeb_vl53l0xWrite8(0x4Fu, 0x00u) ||
        !aeb_vl53l0xWrite8(0x4Eu, 0x2Cu) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x00u) ||
        !aeb_vl53l0xWrite8(VL53L0X_GLOBAL_CONFIG_REF_START, 0xB4u))
    {
        return FALSE;
    }

    uint8 firstSpadToEnable = spadTypeIsAperture ? 12u : 0u;
    uint8 enabledSpads = 0u;

    for (uint8 i = 0u; i < 48u; i++)
    {
        uint8 byteIndex = (uint8)(i / 8u);
        uint8 bitMask = (uint8)(1u << (i % 8u));

        if ((i < firstSpadToEnable) || (enabledSpads == spadCount))
        {
            refSpadMap[byteIndex] &= (uint8)~bitMask;
        }
        else if ((refSpadMap[byteIndex] & bitMask) != 0u)
        {
            enabledSpads++;
        }
    }

    return aeb_vl53l0xWriteMulti(VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES, refSpadMap, 6u);
}

static boolean aeb_vl53l0xPerformSingleRefCalibration(uint8 vhvInitByte)
{
    if (!aeb_vl53l0xWrite8(VL53L0X_SYSRANGE_START, (uint8)(0x01u | vhvInitByte)))
    {
        return FALSE;
    }

    uint32 startTick = aeb_nowTicks();
    uint32 timeoutTicks = aeb_ticksFromMs(150u);
    uint8 status = 0u;

    do
    {
        if (!aeb_vl53l0xRead8(VL53L0X_RESULT_INTERRUPT_STATUS, &status))
        {
            return FALSE;
        }
    } while (((status & 0x07u) == 0u) && ((aeb_nowTicks() - startTick) < timeoutTicks));

    if ((status & 0x07u) == 0u)
    {
        return FALSE;
    }

    return aeb_vl53l0xWrite8(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01u) &&
           aeb_vl53l0xWrite8(VL53L0X_SYSRANGE_START, 0x00u);
}

static boolean aeb_vl53l0xInitSensor(void)
{
    uint8 value = 0u;

    if (aeb_vl53l0xRead8(VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP, &value))
    {
        (void)aeb_vl53l0xWrite8(VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP, (uint8)(value | 0x01u));
    }

    if (!aeb_vl53l0xWrite8(0x88u, 0x00u) ||
        !aeb_vl53l0xWrite8(0x80u, 0x01u) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x01u) ||
        !aeb_vl53l0xWrite8(0x00u, 0x00u) ||
        !aeb_vl53l0xRead8(0x91u, &value) ||
        !aeb_vl53l0xWrite8(0x00u, 0x01u) ||
        !aeb_vl53l0xWrite8(0xFFu, 0x00u) ||
        !aeb_vl53l0xWrite8(0x80u, 0x00u))
    {
        return FALSE;
    }

    if (!aeb_vl53l0xRead8(VL53L0X_MSRC_CONFIG_CONTROL, &value) ||
        !aeb_vl53l0xWrite8(VL53L0X_MSRC_CONFIG_CONTROL, (uint8)(value | 0x12u)) ||
        !aeb_vl53l0xWrite16(0x44u, 0x0020u) ||
        !aeb_vl53l0xWrite8(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xFFu) ||
        !aeb_vl53l0xConfigureSpads() ||
        !aeb_vl53l0xApplyDefaultTuning())
    {
        return FALSE;
    }

    if (!aeb_vl53l0xWrite8(VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04u) ||
        !aeb_vl53l0xRead8(VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH, &value) ||
        !aeb_vl53l0xWrite8(VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH, (uint8)(value & (uint8)~0x10u)) ||
        !aeb_vl53l0xWrite8(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01u) ||
        !aeb_vl53l0xWrite8(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xE8u))
    {
        return FALSE;
    }

    if (!aeb_vl53l0xWrite8(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x01u) ||
        !aeb_vl53l0xPerformSingleRefCalibration(0x40u) ||
        !aeb_vl53l0xWrite8(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02u) ||
        !aeb_vl53l0xPerformSingleRefCalibration(0x00u))
    {
        g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_CALIBRATION_FAIL;
        return FALSE;
    }

    return aeb_vl53l0xWrite8(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xE8u);
}

static boolean aeb_tofProbe(void)
{
    uint8 modelId = 0u;

    for (uint8 attempt = 0u; attempt < 5u; attempt++)
    {
        modelId = 0u;
        boolean readOk = aeb_vl53l0xRead8(0xC0u, &modelId);

        if (readOk && (modelId == 0xEEu))
        {
            g_tofDiag = ((uint16)modelId << 8) | AEB_TOF_DIAG_VL53L0X_PRESENT;
            return TRUE;
        }

        if (!readOk)
        {
            g_tofDiag = AEB_TOF_DIAG_MODEL_READ_FAIL;
        }
        else
        {
            g_tofDiag = ((uint16)modelId << 8) | AEB_TOF_DIAG_UNEXPECTED_MODEL;
        }

        aeb_delayMs(10u);
    }

    return FALSE;
}

static void aeb_tofInit(void)
{
    aeb_tofResetByXshut();
    aeb_i2cBusInit();

    if (!aeb_tofProbe())
    {
        g_tofPresent = FALSE;
        return;
    }

    if (!aeb_vl53l0xInitSensor())
    {
        if ((g_tofDiag & 0x00FFu) != AEB_TOF_DIAG_CALIBRATION_FAIL)
        {
            g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_INIT_FAIL;
        }
        g_tofPresent = FALSE;
        return;
    }

    g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_VL53L0X_PRESENT;
    g_tofPresent = TRUE;
}

static boolean aeb_tofUpdate(void)
{
    if (!g_tofPresent)
    {
        return FALSE;
    }

    if (!aeb_vl53l0xWrite8(VL53L0X_SYSRANGE_START, 0x01u))
    {
        g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_MEASURE_START_FAIL;
        return aeb_isFresh(g_tofLastValidTick, AEB_TOF_STALE_MS);
    }

    uint32 startTick = aeb_nowTicks();
    uint32 timeoutTicks = aeb_ticksFromMs(AEB_TOF_MEASURE_TIMEOUT_MS);
    uint8 status = 0u;

    do
    {
        if (!aeb_vl53l0xRead8(VL53L0X_RESULT_INTERRUPT_STATUS, &status))
        {
            g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_MEASURE_STATUS_FAIL;
            return aeb_isFresh(g_tofLastValidTick, AEB_TOF_STALE_MS);
        }
    } while (((status & 0x07u) == 0u) && ((aeb_nowTicks() - startTick) < timeoutTicks));

    if ((status & 0x07u) == 0u)
    {
        g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_MEASURE_TIMEOUT;
        return aeb_isFresh(g_tofLastValidTick, AEB_TOF_STALE_MS);
    }

    uint16 rawMm = 0u;
    if (!aeb_vl53l0xRead16(VL53L0X_RESULT_RANGE_MM, &rawMm))
    {
        g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_RANGE_READ_FAIL;
        return aeb_isFresh(g_tofLastValidTick, AEB_TOF_STALE_MS);
    }

    (void)aeb_vl53l0xWrite8(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01u);

    float32 scaledCmX10 = (float32)rawMm * AEB_TOF_DISTANCE_SCALE;
    uint32 cmX10 = (uint32)(scaledCmX10 + 0.5f);

    if ((cmX10 >= AEB_TOF_MIN_CM_X10) && (cmX10 <= AEB_TOF_MAX_CM_X10))
    {
        g_tofCmX10 = (uint16)cmX10;
        g_tofLastValidTick = aeb_nowTicks();
        g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_VL53L0X_PRESENT;
    }
    else
    {
        g_tofDiag = (g_tofDiag & 0xFF00u) | AEB_TOF_DIAG_RANGE_OUT_OF_LIMIT;
    }

    return aeb_isFresh(g_tofLastValidTick, AEB_TOF_STALE_MS);
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

static void aeb_put16le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)value;
    buffer[1] = (uint8)(value >> 8);
}

static void aeb_put32le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)value;
    buffer[1] = (uint8)(value >> 8);
    buffer[2] = (uint8)(value >> 16);
    buffer[3] = (uint8)(value >> 24);
}

static uint16 aeb_buildPacket(uint8 *txBuffer, const AebSensorNode_Frame *frame)
{
    memset(txBuffer, 0, AEB_ETH_TX_BUFFER_SIZE);

    memcpy(&txBuffer[0], g_dstMac, 6u);
    memcpy(&txBuffer[6], g_srcMac, 6u);
    txBuffer[12] = 0x08u;
    txBuffer[13] = 0x00u;

    uint8 *ip = &txBuffer[AEB_ETH_HEADER_SIZE];
    uint16 ipTotalLength = AEB_IPV4_HEADER_SIZE + AEB_UDP_HEADER_SIZE + AEB_PAYLOAD_SIZE;
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
    aeb_put16be(&udp[4], AEB_UDP_HEADER_SIZE + AEB_PAYLOAD_SIZE);
    aeb_put16be(&udp[6], 0x0000u);

    uint8 *payload = &udp[AEB_UDP_HEADER_SIZE];
    payload[0] = 'A';
    payload[1] = 'E';
    payload[2] = 'B';
    payload[3] = '1';
    payload[4] = 1u;
    payload[5] = frame->validMask;
    aeb_put16le(&payload[6], frame->tofDiag);
    aeb_put32le(&payload[8], frame->sequence);
    aeb_put32le(&payload[12], frame->timestampMs);
    aeb_put16le(&payload[16], frame->tofFrontCmX10);
    aeb_put16le(&payload[18], frame->ultrasonicLeftCmX10);
    aeb_put16le(&payload[20], frame->ultrasonicRightCmX10);

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

static boolean aeb_rxLooksLikeIpv4Udp(const uint8 *frame)
{
    return frame[12] == 0x08u &&
        frame[13] == 0x00u &&
        (frame[14] >> 4) == 4u &&
        frame[23] == 17u;
}

static void aeb_pollSteeringRx(void)
{
    uint32 nowMs = aeb_ticksToMs(aeb_nowTicks());

    for (uint32 index = 0u; index < IFXGETH_MAX_RX_DESCRIPTORS; index++)
    {
        uint8 *rxBuffer = &g_ethRxBuffer[index * AEB_ETH_RX_BUFFER_SIZE];
        if (aeb_rxLooksLikeIpv4Udp(rxBuffer))
        {
            (void)FrontSteeringNode_acceptEthernetFrame(rxBuffer, AEB_ETH_RX_BUFFER_SIZE, nowMs);
        }
    }
}

void AebSensorNode_init(void)
{
    memset(&g_latestFrame, 0, sizeof(g_latestFrame));

    aeb_ultrasonicInit(&g_leftUltrasonic);
    aeb_ultrasonicInit(&g_rightUltrasonic);
    aeb_ethernetInit();

    g_tofPresent = FALSE;
    g_tofInitAttempted = FALSE;
    g_tofCmX10 = 0u;
    g_tofLastValidTick = 0u;
    g_tofDiag = AEB_TOF_DIAG_NOT_TRIED;
    g_tofNextInitTick = aeb_nowTicks() + aeb_ticksFromMs(AEB_TOF_RETRY_INTERVAL_MS);
}

void AebSensorNode_runOnce(void)
{
    aeb_pollSteeringRx();

    boolean leftValid = aeb_ultrasonicUpdate(&g_leftUltrasonic);
    aeb_delayUs(AEB_ULTRA_CROSSTALK_DELAY_US);
    boolean rightValid = aeb_ultrasonicUpdate(&g_rightUltrasonic);
    boolean tofValid = aeb_tofUpdate();

    g_latestFrame.sequence++;
    g_latestFrame.timestampMs = aeb_ticksToMs(aeb_nowTicks());
    g_latestFrame.tofFrontCmX10 = g_tofCmX10;
    g_latestFrame.ultrasonicLeftCmX10 = leftValid ?
        g_leftUltrasonic.filteredCmX10 : g_leftUltrasonic.lastDiag;
    g_latestFrame.ultrasonicRightCmX10 = rightValid ?
        g_rightUltrasonic.filteredCmX10 : g_rightUltrasonic.lastDiag;
    g_latestFrame.tofDiag = g_tofDiag;
    g_latestFrame.validMask = 0u;

    if (tofValid)
    {
        g_latestFrame.validMask |= AEB_VALID_TOF;
    }

    if (leftValid)
    {
        g_latestFrame.validMask |= AEB_VALID_ULTRA_LEFT;
    }

    if (rightValid)
    {
        g_latestFrame.validMask |= AEB_VALID_ULTRA_RIGHT;
    }

    aeb_sendFrame(&g_latestFrame);
    aeb_pollSteeringRx();

    if (!g_tofPresent && (g_latestFrame.sequence >= AEB_TOF_INIT_DELAY_FRAMES))
    {
        uint32 now = aeb_nowTicks();
        if (!g_tofInitAttempted || ((now - g_tofNextInitTick) < 0x80000000u))
        {
            g_tofInitAttempted = TRUE;
            g_tofNextInitTick = now + aeb_ticksFromMs(AEB_TOF_RETRY_INTERVAL_MS);
            aeb_tofInit();
        }
    }

    aeb_delayMs(AEB_NODE_LOOP_PERIOD_MS);
}

AebSensorNode_Frame AebSensorNode_getLatestFrame(void)
{
    return g_latestFrame;
}
