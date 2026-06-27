#ifndef GPT12_ENC_H_
#define GPT12_ENC_H_

#include "IfxGpt12_IncrEnc.h"
#include "Ifx_Types.h"


/* 전역 엔코더 핸들러 */
extern IfxGpt12_IncrEnc gpt12;

/* 함수 프로토타입 */
void Gpt12_Enc_Init(void);
void Gpt12_Enc_Update(void);
float32 Gpt12_Enc_GetSpeed(void);
sint32 Gpt12_Enc_GetPosition(void);
IfxGpt12_IncrEnc_Direction Gpt12_Enc_GetDirection(void);

#endif /* GPT12_ENC_H_ */
