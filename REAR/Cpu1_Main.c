/* =============================================================
 *  CPU1 : ACC 하위 제어기 (속도 추종 FF + PI) + MANUAL 원격 주행
 *
 *  - ACC    : a_cmd 적분 -> v_ref -> PI + FF -> 듀티 (변경 없음)
 *  - MANUAL : 원격 target_speed + drive_direction -> FF 오픈루프 듀티
 *             (PI 미사용, 적분 누적 불가능, ACC 제어기와 완전 분리)
 *  - AEB/emergency_stop : 최우선 단락제동, 상태 클리어
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


typedef struct {
    float v_ref;
    float v_meas;
    float u;
    float a_cmd;
    float i_term;
} TelemFrame;

/* CPU0 -> CPU1 : 주행 명령 묶음 (control_mode ~ emergency_stop) */
typedef struct {
    uint8 control_mode;     /* 0 MANUAL, 1 ACC, 2 AEB */
    uint8 drive_direction;  /* 0 STOP, 1 FWD, 2 REV   */
    float target_speed;     /* m/s, 항상 양수 크기 */
    float accel_cmd;        /* m/s^2 */
    uint8 emergency_stop;   /* 0/1 */
} DriveCmd;


/* ---------------- 제어 모드 / 방향 ---------------- */
#define MODE_MANUAL         0u
#define MODE_ACC            1u
#define MODE_AEB            2u

#define DIR_STOP            0u
#define DIR_FWD             1u
#define DIR_REV            2u

/* Motor_movChX_PWM 방향 비트 (정방향 기준값) */
#define CHA_FWD_BIT         1
#define CHB_FWD_BIT         0


/* ---------------- 제어 주기 ---------------- */
#define CTRL_PERIOD_MS      5
#define DT                  ((float)CTRL_PERIOD_MS * 0.001f)

#define A_CMD_MAX           4.0f

#define WHEEL_RADIUS        0.0325f
#define ENC_SPEED_TO_MPS    WHEEL_RADIUS

#define FF_SPEED_SLOPE      74.94f
#define FF_DEADZONE         15.0f
#define U_EPS               0.01f

#define VEH_MASS            3.0f
#define GEAR_RATIO          31.0f
#define KT_MOTOR            0.012f
#define KE_MOTOR            0.012f
#define R_MOTOR             1.4f
#define V_BATT              11.0f

#define CRR                 0.015f
#define GRAVITY             9.81f
#define F_ROLL              (CRR * VEH_MASS * GRAVITY)
#define RHO_CDA             0.0f

#define V_MAX               1.1f
#define DUTY_MAX            1.0f
#define DUTY_MIN           -1.0f

#define BRAKE_TH            0.30f
#define MODE_HYST           0.05f
#define V_BRAKE_MIN         0.05f

#define PWM_DUTY_FULL       100.0f

#define NON_CACHED(addr)  ((void *)((uint32)(addr) | 0x20000000))


#pragma section fardata "data_cpu1"

volatile uint8 g_cpu1_ctrl_flag = 0;
static float s_v_ref  = 0.0f;
static float s_I_term = 0.0f;
static uint8 s_brake_mode = 0;
static uint8 s_prev_mode = MODE_ACC;

static float Kp  = 0.5f;
static float Ki  = 2.0f;
static float Kaw = 10.0f;

#pragma section fardata restore


#pragma section fardata "lmudata"
/* CPU1 -> CPU0 : 센서 속도 송신 */
volatile float g_speed_buf[2] = {0.0f, 0.0f};
volatile uint8 g_speed_idx = 0;

/* CPU0 -> CPU1 : 주행 명령 묶음 */
volatile DriveCmd g_drivecmd_buf[2];
volatile uint8    g_drivecmd_idx = 0;

/* CPU1 -> CPU0 : 튜닝 텔레메트리 */
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

void Cpu1_ReadDriveCmd(DriveCmd* out)
{
    volatile uint8* nc_idx_ptr = (volatile uint8*)NON_CACHED(&g_drivecmd_idx);
    uint8 idx = *nc_idx_ptr;
    volatile DriveCmd* nc = (volatile DriveCmd*)NON_CACHED(&g_drivecmd_buf[idx]);
    out->control_mode    = nc->control_mode;
    out->drive_direction = nc->drive_direction;
    out->target_speed    = nc->target_speed;
    out->accel_cmd       = nc->accel_cmd;
    out->emergency_stop  = nc->emergency_stop;
}

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
 *  FF 역모델 (ACC 전용, 변경 없음)
 * ============================================================= */
static float inverse_model(float a_cmd, float v_ref)
{
    float u_speed = (FF_SPEED_SLOPE * v_ref) / 100.0f;
    float F = VEH_MASS * a_cmd + RHO_CDA * v_ref * v_ref;
    float T_motor = (F * WHEEL_RADIUS) / GEAR_RATIO;
    float i       = T_motor / KT_MOTOR;
    float u_accel = (i * R_MOTOR) / V_BATT;
    return u_speed + u_accel;
}

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}


/* =============================================================
 *  구동 출력 (방향 비트 인자화)
 *    fwd != 0 : 정방향,  fwd == 0 : 후진 (방향 비트 반전)
 * ============================================================= */
static void drive_pwm(int duty, uint8 fwd)
{
    int cha = fwd ? CHA_FWD_BIT : (1 - CHA_FWD_BIT);
    int chb = fwd ? CHB_FWD_BIT : (1 - CHB_FWD_BIT);
    Motor_movChA_PWM(duty, cha);
    Motor_movChB_PWM(duty, chb);
}


/* =============================================================
 *  MANUAL 전용 : 원격 속도 -> 오픈루프 듀티 (PI/적분 없음)
 * ============================================================= */
static float manual_speed_to_duty(float v_cmd)
{
    v_cmd = clampf(v_cmd, 0.0f, V_MAX);
    float u = (FF_SPEED_SLOPE * v_cmd) / 100.0f;
    return clampf(u, 0.0f, DUTY_MAX);
}


/* =============================================================
 *  출력 : ACC용 (정방향 구동 / coast / brake) — 기존 로직 유지
 * ============================================================= */
static void apply_output(float u, float v_meas)
{
    if (u >= 0.0f)
    {
        float u_comp = u;
        if (u_comp > U_EPS) u_comp += FF_DEADZONE / 100.0f;
        int duty = (int)(clampf(u_comp, 0.0f, 1.0f) * PWM_DUTY_FULL);
        drive_pwm(duty, 1);   /* ACC는 전진만 */
        return;
    }

    float mag = -u;

    if (v_meas < V_BRAKE_MIN)
    {
        Motor_coastChA();
        Motor_coastChB();
        return;
    }

    if (mag > BRAKE_TH + MODE_HYST)      s_brake_mode = 1;
    else if (mag < BRAKE_TH - MODE_HYST) s_brake_mode = 0;

    if (s_brake_mode == 1)
    {
        Motor_brakeChA(100);
        Motor_brakeChB(100);
    }
    else
    {
        Motor_coastChA();
        Motor_coastChB();
    }
}


static inline void acc_reset_state(void)
{
    s_v_ref  = 0.0f;
    s_I_term = 0.0f;
}


/* =============================================================
 *  제어 루프 (ISR 안에서 호출)
 * ============================================================= */
static void Cpu1_ControlLoop(void)
{
    /* ---- 1. 엔코더 갱신 & 속도 ---- */
    Gpt12_Enc_Update();
    float v_meas = (float)Gpt12_Enc_GetSpeed() * ENC_SPEED_TO_MPS;
    if (!isfinite(v_meas)) v_meas = 0.0f;
    Cpu1_WriteSpeed(v_meas);

    /* ---- 2. 명령 묶음 수신 ---- */
    DriveCmd cmd;
    Cpu1_ReadDriveCmd(&cmd);

    /* ---- 3. AEB / emergency_stop : 모드 무관 최우선 정지 ---- */
    if (cmd.control_mode == MODE_AEB || cmd.emergency_stop)
    {
        acc_reset_state();
        s_brake_mode = 1;
        Motor_brakeChA(100);
        Motor_brakeChB(100);
        s_prev_mode = cmd.control_mode;
        Cpu1_WriteTelem(0.0f, v_meas, 0.0f, 0.0f, 0.0f);
        return;
    }

    /* ---- 4. MANUAL : 원격 속도 오픈루프 (전진/후진/정지) ---- */
    if (cmd.control_mode == MODE_MANUAL)
    {
        if (s_prev_mode != MODE_MANUAL)
            acc_reset_state();          /* ACC 잔류 적분 청소 */
        s_prev_mode = MODE_MANUAL;

        float v_cmd = cmd.target_speed;
        if (!isfinite(v_cmd)) v_cmd = 0.0f;

        if (cmd.drive_direction == DIR_STOP)
        {
            /* 정지: coast (원하면 brake로 교체 가능) */
            Motor_coastChA();
            Motor_coastChB();
            Cpu1_WriteTelem(0.0f, v_meas, 0.0f, 0.0f, 0.0f);
            return;
        }

        float u = manual_speed_to_duty(v_cmd);

        /* 데드존 보상 후 구동 */
        float u_comp = u;
        if (u_comp > U_EPS) u_comp += FF_DEADZONE / 100.0f;
        int duty = (int)(clampf(u_comp, 0.0f, 1.0f) * PWM_DUTY_FULL);

        uint8 fwd = (cmd.drive_direction == DIR_REV) ? 0u : 1u;
        drive_pwm(duty, fwd);

        /* 텔레메트리: 후진이면 v_ref 부호로 방향 표시 */
        float v_show = (cmd.drive_direction == DIR_REV)
                       ? -clampf(v_cmd, 0.0f, V_MAX)
                       :  clampf(v_cmd, 0.0f, V_MAX);
        Cpu1_WriteTelem(v_show, v_meas, u, 0.0f, 0.0f);
        return;
    }

    /* ---- 5. ACC : 기존 가속도 추종 PI (계산식 변경 없음) ---- */
    if (s_prev_mode != MODE_ACC)
        acc_reset_state();
    s_prev_mode = MODE_ACC;

    float a_cmd = cmd.accel_cmd;
    if (!isfinite(a_cmd)) a_cmd = 0.0f;
    a_cmd = clampf(a_cmd, -A_CMD_MAX, A_CMD_MAX);

    s_v_ref += a_cmd * DT;
    s_v_ref  = clampf(s_v_ref, 0.0f, V_MAX);

    uint8 freeze_I = (s_v_ref <= 0.0f);

    float e       = s_v_ref - v_meas;
    float u_ff    = inverse_model(a_cmd, s_v_ref);
    float u_unsat = u_ff + Kp * e + s_I_term;

    float u = clampf(u_unsat, DUTY_MIN, DUTY_MAX);

    if (freeze_I)
        s_I_term = 0.0f;
    else
        s_I_term += (Ki * e + Kaw * (u - u_unsat)) * DT;

    apply_output(u, v_meas);
    Cpu1_WriteTelem(s_v_ref, v_meas, u, a_cmd, s_I_term);
}


/* =============================================================
 *  STM / main (변경 없음)
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
    Cpu1_ControlLoop();
}

void core1_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    Gpt12_Enc_Init();
    Motor_Init();

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    initStmForCpu1();

    while(1)
    {
        __nop();
    }
}
