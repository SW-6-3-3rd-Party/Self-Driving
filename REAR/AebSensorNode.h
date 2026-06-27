/**********************************************************************************************************************
 * Rear AEB sensor Ethernet node for TC375
 *
 * Reads two HC-SR04-style rear diagonal ultrasonic sensors, applies the same median/stale preprocessing used by the
 * Front node, and sends RearStatusData_v1 as a UDP frame over GETH.
 *********************************************************************************************************************/
#ifndef AEB_SENSOR_NODE_H_
#define AEB_SENSOR_NODE_H_

#include "Ifx_Types.h"

/* Target Rear -> HPVC status period: 20 Hz. */
#define AEB_NODE_LOOP_PERIOD_MS             (50u)

/* Ultrasonic range and timing. Echo pins must be level-shifted to 3.3 V when the sensor drives 5 V. */
#define AEB_ULTRA_MIN_CM_X10                (20u)
#define AEB_ULTRA_MAX_CM_X10                (5000u)
#define AEB_ULTRA_ECHO_TIMEOUT_US           (15000u)
#define AEB_ULTRA_STALE_MS                  (150u)
#define AEB_ULTRA_CROSSTALK_DELAY_US        (5000u)

/* Rear ultrasonic GPIO assignment. */
#define AEB_ULTRA_LEFT_TRIG_PORT            (&MODULE_P33)
#define AEB_ULTRA_LEFT_TRIG_PIN             (12u)
#define AEB_ULTRA_LEFT_ECHO_PORT            (&MODULE_P33)
#define AEB_ULTRA_LEFT_ECHO_PIN             (11u)
#define AEB_ULTRA_RIGHT_TRIG_PORT           (&MODULE_P32)
#define AEB_ULTRA_RIGHT_TRIG_PIN            (4u)
#define AEB_ULTRA_RIGHT_ECHO_PORT           (&MODULE_P33)
#define AEB_ULTRA_RIGHT_ECHO_PIN            (13u)

/* Rear Zone ECU -> HPVC UDP settings. */
#define AEB_ETH_SRC_MAC0                    (0x02u)
#define AEB_ETH_SRC_MAC1                    (0x37u)
#define AEB_ETH_SRC_MAC2                    (0x52u)
#define AEB_ETH_SRC_MAC3                    (0xAEu)
#define AEB_ETH_SRC_MAC4                    (0xB0u)
#define AEB_ETH_SRC_MAC5                    (0x02u)

#define AEB_ETH_SRC_IP0                     (192u)
#define AEB_ETH_SRC_IP1                     (168u)
#define AEB_ETH_SRC_IP2                     (10u)
#define AEB_ETH_SRC_IP3                     (12u)

#define AEB_ETH_DST_IP0                     (192u)
#define AEB_ETH_DST_IP1                     (168u)
#define AEB_ETH_DST_IP2                     (10u)
#define AEB_ETH_DST_IP3                     (1u)

#define AEB_ETH_SRC_PORT                    (5012u)
#define AEB_ETH_DST_PORT                    (5012u)

#define AEB_MOTOR_STATE_DISABLED            (0u)
#define AEB_MOTOR_STATE_IDLE                (1u)
#define AEB_MOTOR_STATE_RUNNING             (2u)
#define AEB_MOTOR_STATE_FAULT               (3u)

/*
 * RearStatusData_v1 payload, packed little-endian:
 * 0  float32 rear_left_diag_distance_m
 * 4  float32 rear_right_diag_distance_m
 * 8  uint8   rear_sensor_valid   bit0 left, bit1 right
 * 9  uint8   motor_state         0 disabled, 1 idle, 2 running, 3 fault
 * 10 uint8   alive_count         0..15 rolling counter
 */
typedef struct
{
    uint32 sequence;
    uint32 timestampMs;
    uint16 ultrasonicLeftCmX10;
    uint16 ultrasonicRightCmX10;
    uint8  validMask;
    uint8  motorState;
} AebSensorNode_Frame;

void AebSensorNode_init(void);
void AebSensorNode_runOnce(void);
void AebSensorNode_setMotorState(uint8 motorState);
AebSensorNode_Frame AebSensorNode_getLatestFrame(void);

#endif /* AEB_SENSOR_NODE_H_ */
