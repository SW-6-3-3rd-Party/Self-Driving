/* =============================================================
 *  CPU1 : ACC 하위 제어기 (속도 추종 FF + PI)
 *
 *  - 상위(라즈베리파이) : 목표 가속도 a_cmd 송신 (50ms, 저크 제한 상위에서 처리)
 *  - 하위(이 코드)      : a_cmd 적분 -> v_ref 궤적 -> PI(속도오차) + FF -> 듀티
 *  - 제어 주기          : CTRL_PERIOD_MS (5 또는 10) , 전부 ISR 안에서 처리
 *  - 출력               : 양수 듀티 -> 구동 / 음수 듀티 -> 코스트 or 단락제동
 *
 *  ※ 튜닝 대상은 PI 게인(Kp,Ki,Kaw)뿐. FF 상수는 고정.
 * ============================================================= */

#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"

#include "Bsp.h"
#include "Stm/Std/IfxStm.h"
#include "gpt12_enc.h"
#include "Motor.h"
#include "gtm_atom_pwm.h"

#include "IfxStm.h"
#include "ConfigurationIsr.h"
#include "Configuration.h"

#include <math.h>


/* =============================================================
 *  텔레메트리 (튜닝용) : 한 제어주기 스냅샷을 통째로 송신
 *    CPU0 가 g_telem_buf[g_telem_idx] 를 읽어 UDP 로 전송
 * ============================================================= */
typedef struct {
    float v_ref;    /* 레퍼런스 속도 */
    float v_meas;   /* 측정 속도 */
    float u;        /* 제어기 출력 듀티(데드존 보상 전, 정규화) */
    float a_cmd;    /* 수신/클램프된 가속도 명령 */
    float i_term;   /* PI 적분항 */
} TelemFrame;


/* =============================================================
 *  [확인 필요 1] 가속도 명령은 float(m/s^2) 그대로 수신.
 *    상위(CPU0) 송신부도 g_acc_buf 를 float 로 맞출 것. clampf 로 ±A_CMD_MAX 방어.
 * -------------------------------------------------------------
 *  [확인 필요 2] Gpt12_Enc_GetSpeed() 단위 = 출력축 rad/s (확인됨, ~30 검증)
 *    ENC_SPEED_TO_MPS = r 로 변환.
 * -------------------------------------------------------------
 *  [확인 필요 3] Motor.c 에 coast/brake 헬퍼 4개 추가 필요
 *    Motor_coastChA/B(), Motor_brakeChA/B(duty 0~100)
 *    구동은 기존 Motor_movChX_PWM 사용 (브레이크 자동 해제).
 *    PWM 인자 범위 = 0~100(%).
 * ============================================================= */


/* ---------------- 제어 주기 ---------------- */
#define CTRL_PERIOD_MS      5                       /* 5 또는 10 */
#define DT                  ((float)CTRL_PERIOD_MS * 0.001f)

/* ---------------- 가속도 명령 안전 클램프 ---------------- */
/* 상위에서 float 가속도를 그대로 수신. 이상값 방어용 한계 */
#define A_CMD_MAX           4.0f                     /* m/s^2 (양/음 대칭) */

/* ---------------- 엔코더 속도 -> m/s ---------------- */
/* Gpt12_Enc_GetSpeed() = 출력축(휠) 각속도 [rad/s]  →  v = omega * r */
#define WHEEL_RADIUS        0.0325f                  /* m (지름 6.5cm) */
#define ENC_SPEED_TO_MPS    WHEEL_RADIUS

/* ---------------- FF: 측정 기반 듀티 맵 (개루프 스윕에서 교정) ---------------- */
/* 무부하 정상상태 직선:  duty% = FF_SPEED_SLOPE * v + FF_DEADZONE   (25~60% 구간) */
#define FF_SPEED_SLOPE      74.94f     /* %/(m/s) : back-EMF+마찰 속도항 기울기 */
#define FF_DEADZONE         15.0f      /* % : 정지마찰/BJT강하 극복 오프셋 (외삽 13.3, 문턱 ~20) */
#define U_EPS               0.01f      /* 이 듀티 이상 구동 의도일 때만 데드존 보상 */

/* ---------------- 차량/모터 FF 상수 (가속항 모델용, 고정) ---------------- */
#define VEH_MASS            3.0f        /* kg */
#define GEAR_RATIO          31.0f
#define KT_MOTOR            0.012f      /* N·m/A (모터축) */
#define KE_MOTOR            0.012f      /* (미사용: 속도항을 측정 직선으로 대체) */
#define R_MOTOR             1.4f        /* ohm (멀티미터 실측, 듀티-전류 피팅 검증 권장) */
#define V_BATT              11.0f       /* V (쉴드 강하 반영한 실효 공급) */

#define CRR                 0.015f      /* 구름저항계수 */
#define GRAVITY             9.81f
#define F_ROLL              (CRR * VEH_MASS * GRAVITY)   /* N */
#define RHO_CDA             0.0f        /* 0.5*rho*Cd*A , RC카 저속이라 ~0 */

/* ---------------- 속도/듀티 한계 ---------------- */
#define V_MAX               1.1f        /* m/s (무부하 330rpm 기준 상한) */
#define DUTY_MAX            1.0f
#define DUTY_MIN           -1.0f

/* ---------------- 음수 듀티 -> 코스트/비상제동 경계 ---------------- */
/* 기본은 coast. |duty|가 BRAKE_TH 초과 = coast로 부족한 강제동 요구 -> full brake */
#define BRAKE_TH            0.30f       /* 비상 단락제동 임계 (튜닝) */
#define MODE_HYST           0.05f       /* 경계 채터링 방지 */
#define V_BRAKE_MIN         0.05f       /* 이 속도 미만 -> 단락제동 무력, coast */


#define PWM_DUTY_FULL       100.0f     /* Motor_movChX_PWM 인자 범위 = 0~100(%) */


#define NON_CACHED(addr)  ((void *)((uint32)(addr) | 0x20000000))


#pragma section fardata "data_cpu1"

volatile uint8 g_cpu1_ctrl_flag = 0;
/* ---------------- 제어 영속 상태 ---------------- */
static float s_v_ref  = 0.0f;   /* 레퍼런스 속도 궤적 */
static float s_I_term = 0.0f;   /* PI 적분항 */

/* 모드 히스테리시스용 (0:coast 1:brake) */
static uint8 s_brake_mode = 0;

/* ---------------- PI 게인 (★튜닝 대상★) ---------------- */
/* 시작값. 속도오차[m/s] -> 듀티 보정. P -> I -> Kaw 순으로 조정 */
static float Kp  = 0.5f;
static float Ki  = 2.0f;
static float Kaw = 10.0f;   /* 안티와인드업 (back-calculation), ~1/Ti */


#pragma section fardata restore


#pragma section fardata "lmudata"
/* CPU1 -> CPU0 : 센서 속도 송신 */
volatile float g_speed_buf[2] = {0.0f, 0.0f};
volatile uint8 g_speed_idx = 0;

/* CPU0 -> CPU1 : 가속도 명령 수신 (float m/s^2, 부호 포함) */
volatile float g_acc_buf[2] = {0.0f, 0.0f};
volatile uint8 g_acc_idx = 0;

/* CPU1 -> CPU0 : 튜닝 텔레메트리 (5개 float 스냅샷) */
volatile TelemFrame g_telem_buf[2];
volatile uint8 g_telem_idx = 0;
#pragma section fardata restore


extern IfxCpu_syncEvent g_cpuSyncEvent;


/* =============================================================
 *  통신 (lock-free 더블버퍼)
 * ============================================================= */
void Cpu1_WriteSpeed(float new_speed)
{
    volatile uint8* nc_idx_ptr = (volatile uint8*)NON_CACHED(&g_speed_idx);
    uint8 current_idx = *nc_idx_ptr;
    uint8 write_idx   = 1 - current_idx;

    volatile float* nc_speed_buf =
            (volatile float*)NON_CACHED(&g_speed_buf[write_idx]);

    *nc_speed_buf = new_speed;
    __dsync();
    *nc_idx_ptr = write_idx;
}

float Cpu1_ReadAcc(void)
{
    volatile uint8* nc_idx_ptr = (volatile uint8*)NON_CACHED(&g_acc_idx);
    uint8 read_idx = *nc_idx_ptr;
    volatile float* nc_acc_buf =
            (volatile float*)NON_CACHED(&g_acc_buf[read_idx]);
    return *nc_acc_buf;   /* m/s^2 (부호 포함) */
}

/* 텔레메트리 한 프레임 송신 (lock-free 더블버퍼, 5개 값 원자적 묶음) */
void Cpu1_WriteTelem(float v_ref, float v_meas, float u, float a_cmd, float i_term)
{
    volatile uint8* nc_idx_ptr = (volatile uint8*)NON_CACHED(&g_telem_idx);
    uint8 current_idx = *nc_idx_ptr;
    uint8 write_idx   = 1 - current_idx;

    volatile TelemFrame* nc_f =
            (volatile TelemFrame*)NON_CACHED(&g_telem_buf[write_idx]);

    nc_f->v_ref  = v_ref;
    nc_f->v_meas = v_meas;
    nc_f->u      = u;
    nc_f->a_cmd  = a_cmd;
    nc_f->i_term = i_term;

    __dsync();
    *nc_idx_ptr = write_idx;
}


/* =============================================================
 *  FF 역모델 :  목표 가속도 -> 듀티
 * ============================================================= */
/* FF는 레퍼런스(v_ref) 기반이어야 함. 측정(v_meas)을 쓰면 양의 피드백→폭주.
 * 측정 피드백 보정은 PI 담당. */
static float inverse_model(float a_cmd, float v_ref)
{
    /* 속도항: 목표 속도 유지에 필요한 듀티 (측정 직선 기울기, back-EMF+마찰) */
    float u_speed = (FF_SPEED_SLOPE * v_ref) / 100.0f;   /* 정규화 듀티 */

    /* 가속항: 부하 토크 모델 (무부하 스윕엔 안 잡히는 성분) */
    float F = VEH_MASS * a_cmd + RHO_CDA * v_ref * v_ref;
    float T_motor = (F * WHEEL_RADIUS) / GEAR_RATIO;
    float i       = T_motor / KT_MOTOR;
    float u_accel = (i * R_MOTOR) / V_BATT;

    return u_speed + u_accel;
}


/* =============================================================
 *  출력 :  듀티 -> 모터 핀 (구동 / 코스트 / 단락제동)
 * ============================================================= */
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void apply_output(float u, float v_meas)
{
    if (u >= 0.0f)
    {
        /* 데드존 역보상: 구동 의도가 있으면 정지마찰 문턱만큼 밀어줌 */
        float u_comp = u;
        if (u_comp > U_EPS) u_comp += FF_DEADZONE / 100.0f;

        /* 구동 : Motor_movChX_PWM 이 브레이크 자동 해제 */
        int duty = (int)(clampf(u_comp, 0.0f, 1.0f) * PWM_DUTY_FULL);
        Motor_movChA_PWM(duty, 1);
        Motor_movChB_PWM(duty, 0);
        return;
    }

    /* 음수 듀티 : 감속.
     * coast 감속이 ~1.7m/s² (실측) 로 ACC 일반 감속을 거의 커버하므로
     * 기본은 coast, 그걸로 부족한 강한 감속에만 full 단락제동(비상). */
    float mag = -u;

    /* 저속 단락제동 무력 구간 -> coast */
    if (v_meas < V_BRAKE_MIN)
    {
        Motor_coastChA();
        Motor_coastChB();
        return;
    }

    /* coast <-> full brake 모드 결정 (경계 히스테리시스) */
    if (mag > BRAKE_TH + MODE_HYST)      s_brake_mode = 1;
    else if (mag < BRAKE_TH - MODE_HYST) s_brake_mode = 0;

    if (s_brake_mode == 1)
    {
        /* 비상 단락제동 : 항상 최대 (PWM 100%) */
        Motor_brakeChA(100);
        Motor_brakeChB(100);
    }
    else
    {
        /* coast : 프리휠 (기어박스 마찰로 ~1.7m/s² 감속) */
        Motor_coastChA();
        Motor_coastChB();
    }
}


/* =============================================================
 *  제어 루프 (ISR 안에서 호출, 주기 = CTRL_PERIOD_MS)
 * ============================================================= */
static void Cpu1_ControlLoop(void)
{
    /* ---- 1. 엔코더 갱신 & 속도 ---- */
    Gpt12_Enc_Update();
    /* GetSpeed()=real_T(double) 출력축 rad/s.  TC375 FPU는 단정밀도라 즉시 float 캐스팅 */
    float v_meas = (float)Gpt12_Enc_GetSpeed() * ENC_SPEED_TO_MPS;   /* m/s */
    /* 엔코더 저속 모드에서 간헐적 비정상값(±inf/NaN) 방어 → 정지로 간주 */
    if (!isfinite(v_meas)) v_meas = 0.0f;

    /* 속도 송신 (CPU0) */
    Cpu1_WriteSpeed(v_meas);

    /* ---- 2. 목표 가속도 수신 (float, 안전 클램프) ---- */
    float a_cmd = Cpu1_ReadAcc();
    /* 통신 깨짐/타입 불일치로 비정상 비트가 들어오면 0 처리 (모터 폭주 방지) */
    if (!isfinite(a_cmd)) a_cmd = 0.0f;
    a_cmd = clampf(a_cmd, -A_CMD_MAX, A_CMD_MAX);

    /* ---- 3. 레퍼런스 속도 궤적 (a_cmd 적분) ---- */
    s_v_ref += a_cmd * DT;
    s_v_ref  = clampf(s_v_ref, 0.0f, V_MAX);       /* ACC: 후진 없음 */

    /* 정지 상태(v_ref=0)면 적분항 청소.
     * 부호 무관: 비상정지 후 잔류 적분이 다음 출발을 막는 것 방지. */
    uint8 freeze_I = (s_v_ref <= 0.0f);

    /* ---- 4. FF + PI(속도오차) ---- */
    float e       = s_v_ref - v_meas;
    float u_ff    = inverse_model(a_cmd, s_v_ref);   /* FF는 레퍼런스 기반 */
    float u_unsat = u_ff + Kp * e + s_I_term;

    /* ---- 5. 포화 + 안티와인드업(back-calculation) ---- */
    float u = clampf(u_unsat, DUTY_MIN, DUTY_MAX);

    if (freeze_I)
        s_I_term = 0.0f;
    else
        s_I_term += (Ki * e + Kaw * (u - u_unsat)) * DT;

    /* ---- 6. 출력 ---- */
    apply_output(u, v_meas);

    /* ---- 7. 텔레메트리 송신 (튜닝용, 보상 전 u) ---- */
    Cpu1_WriteTelem(s_v_ref, v_meas, u, a_cmd, s_I_term);
}


/* =============================================================
 *  STM : CTRL_PERIOD_MS 주기 인터럽트
 * ============================================================= */
void initStmForCpu1(void)
{
    IfxStm_CompareConfig stmCompareConfig;
    IfxStm_initCompareConfig(&stmCompareConfig);

    stmCompareConfig.triggerPriority     = ISR_PRIORITY_OS_TICK;
    stmCompareConfig.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0;
    stmCompareConfig.ticks               = IFX_CFG_STM_TICKS_PER_MS * CTRL_PERIOD_MS;
    stmCompareConfig.typeOfService       = IfxSrc_Tos_cpu1;

    IfxStm_initCompare(&MODULE_STM1, &stmCompareConfig);
}

IFX_INTERRUPT(cpu1_ctrl_ISR, 1, ISR_PRIORITY_OS_TICK);

void cpu1_ctrl_ISR(void)
{
    IfxStm_increaseCompare(&MODULE_STM1,
                           IfxStm_Comparator_0,
                           IFX_CFG_STM_TICKS_PER_MS * CTRL_PERIOD_MS);

    /* 제어 전체를 ISR 안에서 수행 */
    Cpu1_ControlLoop();
}


/* =============================================================
 *  CPU1 main
 * ============================================================= */
void core1_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    Gpt12_Enc_Init();
    Motor_Init();

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    initStmForCpu1();

    /* while 은 비움 — 모든 처리는 cpu1_ctrl_ISR 안에서 */
    while(1)
    {
        __nop();
    }
}
