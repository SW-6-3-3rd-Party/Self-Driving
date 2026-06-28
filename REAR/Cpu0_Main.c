#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"

#include "geth_lwip.h"
#include "udp_txrx.h"
#include "asclin.h"
#include "gettime.h"
#include "AebSensorNode.h"

#define NON_CACHED(addr)  ((void *)((uint32_t)(addr) | 0x20000000))

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

typedef struct {
    float v_ref;
    float v_meas;
    float u;
    float a_cmd;
    float i_term;
} TelemFrame;

typedef struct {
    float rear_left_m;
    float rear_right_m;
    float vehicle_speed_mps;
    uint8 valid_mask;
    uint8 motor_state;
    uint8 alive_count;
    uint8 reserved;
} RearHpvcStatusFrame;

extern volatile float g_speed_buf[2];
extern volatile uint8 g_speed_idx;

extern volatile float g_acc_buf[2];
extern volatile uint8 g_acc_idx;

extern volatile TelemFrame g_telem_buf[2];
extern volatile uint8      g_telem_idx;

/* g_ultra_buf / g_ultra_idx 는 AebSensorNode.h 의 extern 선언 사용 */

/* -------------------------------------------------------------
 * 통신 인터페이스 (Lock-Free)
 * ------------------------------------------------------------- */
void Cpu0_ReadSpeed(float* out) {
    volatile uint8* nc_idx_ptr = (volatile uint8*)NON_CACHED(&g_speed_idx);
    uint8 read_idx = *nc_idx_ptr;
    volatile float* nc_speed_buf = (volatile float*)NON_CACHED(&g_speed_buf[read_idx]);
    *out = *nc_speed_buf;
}

void Cpu0_ReadTelem(TelemFrame* out)
{
    volatile uint8* nc_idx_ptr = (volatile uint8*)NON_CACHED(&g_telem_idx);
    uint8 read_idx = *nc_idx_ptr;
    volatile TelemFrame* nc_f = (volatile TelemFrame*)NON_CACHED(&g_telem_buf[read_idx]);
    out->v_ref  = nc_f->v_ref;
    out->v_meas = nc_f->v_meas;
    out->u      = nc_f->u;
    out->a_cmd  = nc_f->a_cmd;
    out->i_term = nc_f->i_term;
}

/* CPU2가 올린 초음파 LMU 프레임 읽기 */
void Cpu0_ReadUltra(AebUltraLmuFrame* out)
{
    volatile uint8* nc_idx_ptr = (volatile uint8*)NON_CACHED(&g_ultra_idx);
    uint8 read_idx = (uint8)(*nc_idx_ptr & 1u);
    volatile AebUltraLmuFrame* nc_f =
            (volatile AebUltraLmuFrame*)NON_CACHED(&g_ultra_buf[read_idx]);
    out->left      = nc_f->left;
    out->right     = nc_f->right;
    out->validMask = nc_f->validMask;
}

void Cpu0_WriteAcc(float new_acc) {
    volatile uint8* nc_idx_ptr = (volatile uint8*)NON_CACHED(&g_acc_idx);
    uint8 current_idx = *nc_idx_ptr;
    uint8 write_idx = 1 - current_idx;
    volatile float* nc_acc_buf = (volatile float*)NON_CACHED(&g_acc_buf[write_idx]);
    *nc_acc_buf = new_acc;
    __dsync();
    *nc_idx_ptr = write_idx;
}



/* -------------------------------------------------------------
 * CPU0 메인
 * ------------------------------------------------------------- */
void core0_main(void)
{
    IfxCpu_enableInterrupts();

    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    Asclin0_InitUart();
    eth_addr_t ethAddr = {
        .addr[0] = 0x00, .addr[1] = 0x00, .addr[2] = 0x0c,
        .addr[3] = 0x11, .addr[4] = 0x11, .addr[5] = 0x11
    };

    for(volatile int i=0; i<10000000; i++);
    initLwip(ethAddr);
    UdpInit();

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    uint32_t last_send_time  = Get_SystemTime_ms();   /* 텔레메트리 10ms */
    uint8    ultra_alive     = 0u;

    while(1)
    {
        Ifx_Lwip_pollTimerFlags();
        Ifx_Lwip_pollReceiveFlags();

        uint32_t current_time = Get_SystemTime_ms();

        /* 10ms: 모터 텔레메트리 */
        if ((current_time - last_send_time) >= 20)
        {
            last_send_time = current_time;

            TelemFrame tf;
            Cpu0_ReadTelem(&tf);
//            UdpSendToPC(5001, &tf, sizeof(TelemFrame));

            AebUltraLmuFrame u;
            Cpu0_ReadUltra(&u);

            RearHpvcStatusFrame rear;
            rear.rear_left_m = u.left;
            rear.rear_right_m = u.right;
            rear.vehicle_speed_mps = tf.v_meas;
            rear.valid_mask = u.validMask;
            rear.motor_state = 2u;
            rear.alive_count = ultra_alive++ & 0x0Fu;
            rear.reserved = 0u;
            UdpSendToPC(5012, &rear, sizeof(rear));
        }

    }
}
