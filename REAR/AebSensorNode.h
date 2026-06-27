#ifndef AEB_SENSOR_NODE_H_
#define AEB_SENSOR_NODE_H_

#include "Ifx_Types.h"

/* CPU2 측정 주기: 20 Hz */
#define AEB_NODE_LOOP_PERIOD_MS             (50u)

/* 초음파 range/timing */
#define AEB_ULTRA_MIN_CM_X10                (20u)
#define AEB_ULTRA_MAX_CM_X10                (5000u)
#define AEB_ULTRA_ECHO_TIMEOUT_US           (15000u)
#define AEB_ULTRA_STALE_MS                  (150u)
#define AEB_ULTRA_CROSSTALK_DELAY_US        (5000u)

/* 초음파 GPIO */
#define AEB_ULTRA_LEFT_TRIG_PORT            (&MODULE_P33)
#define AEB_ULTRA_LEFT_TRIG_PIN             (12u)
#define AEB_ULTRA_LEFT_ECHO_PORT            (&MODULE_P33)
#define AEB_ULTRA_LEFT_ECHO_PIN             (11u)
#define AEB_ULTRA_RIGHT_TRIG_PORT           (&MODULE_P32)
#define AEB_ULTRA_RIGHT_TRIG_PIN            (4u)
#define AEB_ULTRA_RIGHT_ECHO_PORT           (&MODULE_P33)
#define AEB_ULTRA_RIGHT_ECHO_PIN            (13u)

#define AEB_VALID_ULTRA_LEFT                (1u << 0)
#define AEB_VALID_ULTRA_RIGHT               (1u << 1)

/* PC 송신 포트 */
#define AEB_REAR_STATUS_PORT                (5012u)

/* CPU2 -> CPU0 LMU 공유 프레임 (단일 writer/단일 reader) */
typedef struct
{
    float32 left;        /* 좌측 대각 거리 [m], invalid 시 0 */
    float32 right;       /* 우측 대각 거리 [m], invalid 시 0 */
    uint8   validMask;   /* bit0 left, bit1 right */
} AebUltraLmuFrame;

/* LMU 버퍼는 AebSensorNode.c에서 정의, 여기서는 선언만 */
extern volatile AebUltraLmuFrame g_ultra_buf[2];
extern volatile uint8            g_ultra_idx;

/* CPU2 전용 API */
void AebSensorNode_ultraInit(void);
void AebSensorNode_ultraRunOnce(void);

#endif /* AEB_SENSOR_NODE_H_ */
