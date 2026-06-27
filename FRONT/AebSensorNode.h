/**********************************************************************************************************************
 * AEB sensor Ethernet node for TC375
 *
 * Reads two HC-SR04-style ultrasonic sensors and one front VL53L0X-compatible ToF sensor, filters them to centimeters,
 * and sends the latest values as a UDP broadcast frame over GETH.
 *********************************************************************************************************************/
#ifndef AEB_SENSOR_NODE_H_
#define AEB_SENSOR_NODE_H_

#include "Ifx_Types.h"

/* Main loop period. 40 ms matches the 25 Hz sensor loop used by the Raspberry Pi prototype. */
#define AEB_NODE_LOOP_PERIOD_MS             (40u)

/* Ultrasonic range and timing. Echo pins must be level-shifted to 3.3 V when the sensor drives 5 V. */
#define AEB_ULTRA_MIN_CM_X10                (20u)
#define AEB_ULTRA_MAX_CM_X10                (5000u)
#define AEB_ULTRA_ECHO_TIMEOUT_US           (15000u)
#define AEB_ULTRA_STALE_MS                  (500u)
#define AEB_ULTRA_CROSSTALK_DELAY_US        (5000u)

/* GPIO assignment using the requested nearby pins. */
#define AEB_ULTRA_LEFT_TRIG_PORT            (&MODULE_P02)
#define AEB_ULTRA_LEFT_TRIG_PIN             (3u)
#define AEB_ULTRA_LEFT_ECHO_PORT            (&MODULE_P10)
#define AEB_ULTRA_LEFT_ECHO_PIN             (4u)
#define AEB_ULTRA_RIGHT_TRIG_PORT           (&MODULE_P02)
#define AEB_ULTRA_RIGHT_TRIG_PIN            (1u)
#define AEB_ULTRA_RIGHT_ECHO_PORT           (&MODULE_P02)
#define AEB_ULTRA_RIGHT_ECHO_PIN            (0u)

/* ToF I2C. The minimal built-in reader targets VL53L0X-compatible modules at the same 0x29 address as aeb.py. */
#define AEB_TOF_XSHUT_ENABLED               (1u)
#define AEB_TOF_XSHUT_PORT                  (&MODULE_P02)
#define AEB_TOF_XSHUT_PIN                   (6u)
#define AEB_TOF_XSHUT_RESET_LOW_MS          (10u)
#define AEB_TOF_BOOT_WAIT_MS                (100u)
#define AEB_TOF_INIT_DELAY_FRAMES           (10u)
#define AEB_TOF_RETRY_INTERVAL_MS           (1000u)
#define AEB_TOF_SDA_PORT                    (&MODULE_P02)
#define AEB_TOF_SDA_PIN                     (4u)
#define AEB_TOF_SCL_PORT                    (&MODULE_P02)
#define AEB_TOF_SCL_PIN                     (5u)
#define AEB_TOF_I2C_HALF_PERIOD_US          (10u)
#define AEB_TOF_I2C_SCL_TIMEOUT_US          (1000u)
#define AEB_TOF_I2C_ADDRESS_7BIT            (0x29u)
#define AEB_TOF_MIN_CM_X10                  (30u)
#define AEB_TOF_MAX_CM_X10                  (4000u)
#define AEB_TOF_STALE_MS                    (400u)
#define AEB_TOF_MEASURE_TIMEOUT_MS          (100u)
#define AEB_TOF_DISTANCE_SCALE              (0.5f)  /* Keep the Raspberry Pi aeb.py TOF_DISTANCE_SCALE calibration. */

/* Ethernet/IP/UDP broadcast settings. Listen on UDP port 5005. */
#define AEB_ETH_SRC_MAC0                    (0x02u)
#define AEB_ETH_SRC_MAC1                    (0x37u)
#define AEB_ETH_SRC_MAC2                    (0x50u)
#define AEB_ETH_SRC_MAC3                    (0xAEu)
#define AEB_ETH_SRC_MAC4                    (0xB0u)
#define AEB_ETH_SRC_MAC5                    (0x01u)

#define AEB_ETH_SRC_IP0                     (192u)
#define AEB_ETH_SRC_IP1                     (168u)
#define AEB_ETH_SRC_IP2                     (10u)
#define AEB_ETH_SRC_IP3                     (11u)

#define AEB_ETH_DST_IP0                     (192u)
#define AEB_ETH_DST_IP1                     (168u)
#define AEB_ETH_DST_IP2                     (10u)
#define AEB_ETH_DST_IP3                     (1u)

#define AEB_ETH_SRC_PORT                    (5011u)
#define AEB_ETH_DST_PORT                    (5011u)

typedef struct
{
    uint32  sequence;
    uint32  timestampMs;
    uint16  tofFrontCmX10;
    uint16  ultrasonicLeftCmX10;
    uint16  ultrasonicRightCmX10;
    uint16  tofDiag;               /* high byte: last model ID, low byte: ToF diagnostic code */
    uint8   validMask;              /* bit0: left ultrasonic, bit1: right ultrasonic, bit2: ToF */
} AebSensorNode_Frame;

void AebSensorNode_init(void);
void AebSensorNode_runOnce(void);
AebSensorNode_Frame AebSensorNode_getLatestFrame(void);

#endif /* AEB_SENSOR_NODE_H_ */
