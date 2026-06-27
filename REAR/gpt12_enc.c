#include "gpt12_enc.h"

/* 전역 엔코더 구조체 인스턴스 */
#pragma section fardata "data_cpu1"
IfxGpt12_IncrEnc gpt12;
#pragma section fardata restore

/* 엔코더 초기화 함수 */
void Gpt12_Enc_Init(void)
{
    /* 1. GPT12 모듈 클럭 활성화 및 프리스케일러 설정 */
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_8);
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_4);

    /* 2. GPT12 Incremental Encoder 설정 구조체 초기화 */
    IfxGpt12_IncrEnc_Config gpt12Config;
    IfxGpt12_IncrEnc_initConfig(&gpt12Config, &MODULE_GPT120);

    /* 3. T3 코어 기반 세부 설정 (제공된 구조체 기준) */
    gpt12Config.offset              = 0;
    gpt12Config.reversed            = FALSE;                // 회전 방향 반전 여부
    gpt12Config.resolution          = 330;

    // 체배 설정 (iLLD 버전에 따라 _fourFold 또는 4 로 입력)
    gpt12Config.resolutionFactor    = IfxGpt12_IncrEnc_ResolutionFactor_fourFold;

    gpt12Config.updatePeriod        = 5e-3;               // 업데이트 주기
    gpt12Config.speedModeThreshold  = 200;                  // 고속/저속 측정 전환 임계값
    gpt12Config.minSpeed            = 10;
    gpt12Config.maxSpeed            = 500;

    /* Z상(Zero)이 없으므로 관련 핀 및 인터럽트 비활성화 */
    gpt12Config.zeroIsrPriority     = 0;
    gpt12Config.pinZ                = NULL_PTR;             // Z상 미사용

    /* 핀 설정 (A, B상만 사용) */
    gpt12Config.pinA                = &IfxGpt120_T3INB_P10_4_IN;  // A상
    gpt12Config.pinB                = &IfxGpt120_T3EUDB_P10_7_IN; // B상
    gpt12Config.pinMode             = IfxPort_InputMode_noPullDevice;

    /* 핀 초기화 드라이버 위임 여부 (구조체에 존재하므로 TRUE로 명시) */
    gpt12Config.initPins            = TRUE;

    /* 4. 모듈 최종 초기화 적용 */
    IfxGpt12_IncrEnc_init(&gpt12, &gpt12Config);
}

/* 주기적인 엔코더 상태 업데이트 함수 */
void Gpt12_Enc_Update(void)
{
    IfxGpt12_IncrEnc_update(&gpt12);
}

/* 속도 값 반환 */
float32 Gpt12_Enc_GetSpeed(void)
{
    return IfxGpt12_IncrEnc_getSpeed(&gpt12);
}

/* 위치(Raw Position) 값 반환 */
sint32 Gpt12_Enc_GetPosition(void)
{
    return IfxGpt12_IncrEnc_getRawPosition(&gpt12);
}

/* 회전 방향 반환 */
IfxGpt12_IncrEnc_Direction Gpt12_Enc_GetDirection(void)
{
    return IfxGpt12_IncrEnc_getDirection(&gpt12);
}
