/**********************************************************************************************************************
 * FRONT steering command receiver for TC375.
 *
 * Accepts HPVC/PC HPSC UDP steering commands, validates them, and drives a
 * servo PWM output with watchdog fail-safe center.
 *********************************************************************************************************************/
#ifndef FRONT_STEERING_NODE_H_
#define FRONT_STEERING_NODE_H_

#include "Ifx_Types.h"

#define FRONT_STEERING_UDP_PORT                (5100u)
#define FRONT_STEERING_WATCHDOG_MS             (200u)
#define FRONT_STEERING_MAX_ABS_RAD             (0.50f)
#define FRONT_STEERING_DEFAULT_MAX_RATE_RAD_S  (1.00f)

/* RC servo calibration measured on the FRONT steering servo. */
#define FRONT_STEERING_SERVO_LEFT_PULSE_US     (1150u)
#define FRONT_STEERING_SERVO_CENTER_PULSE_US   (1650u)
#define FRONT_STEERING_SERVO_RIGHT_PULSE_US    (2000u)
#define FRONT_STEERING_SERVO_MIN_PULSE_US      FRONT_STEERING_SERVO_LEFT_PULSE_US
#define FRONT_STEERING_SERVO_MAX_PULSE_US      FRONT_STEERING_SERVO_RIGHT_PULSE_US
#define FRONT_STEERING_SERVO_PERIOD_US         (20000u)
#define FRONT_STEERING_SERVO_INVERT            (0u)

#define FRONT_STEERING_BOOT_CENTER_MS          (500u)

/*
 * Default PWM output is software-generated GPIO pulses on P02.3.
 * Positive steering angle maps
 * to FRONT_STEERING_SERVO_LEFT_PULSE_US; negative steering angle maps to
 * FRONT_STEERING_SERVO_RIGHT_PULSE_US. If your board routes the servo signal
 * elsewhere, change FRONT_STEERING_SERVO_PORT and FRONT_STEERING_SERVO_PIN in
 * FrontSteeringNode.c.
 */

typedef struct
{
    boolean linkValid;
    boolean commandValid;
    boolean emergencyCenter;
    float32 targetAngleRad;
    float32 appliedAngleRad;
    uint32  lastSequence;
    uint32  acceptedPackets;
    uint32  rejectedPackets;
    uint32  lastPacketAgeMs;
} FrontSteeringNode_Status;

void FrontSteeringNode_init(void);
void FrontSteeringNode_runOnce(void);
void FrontSteeringNode_holdCurrentForMs(uint32 holdMs);
boolean FrontSteeringNode_acceptHpscPacket(const uint8 *payload, uint16 length, uint32 nowMs);
boolean FrontSteeringNode_acceptEthernetFrame(const uint8 *frame, uint16 length, uint32 nowMs);
FrontSteeringNode_Status FrontSteeringNode_getStatus(void);

#endif /* FRONT_STEERING_NODE_H_ */
