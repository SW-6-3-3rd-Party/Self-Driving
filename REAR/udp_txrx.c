#include "udp_txrx.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "ip_addr.h"

#include <string.h>

#if LWIP_UDP

/* CPU0 -> CPU1 : 주행 명령 묶음 (control_mode ~ emergency_stop) */
typedef struct {
    uint8 control_mode;     /* 0 MANUAL, 1 ACC, 2 AEB */
    uint8 drive_direction;  /* 0 STOP, 1 FWD, 2 REV   */
    float target_speed;     /* m/s, 항상 양수 크기 */
    float accel_cmd;        /* m/s^2, ACC용 */
    uint8 emergency_stop;   /* 0/1 */
} DriveCmd;

#define CTR_PORT 5000
#define HPVC_REAR_COMMAND_PORT 5110
#define DOIP_PORT 13400
#define SOMEIP_PORT 30492

static struct udp_pcb *udp_send_pcb;
static struct udp_pcb *udp_ctr_pcb;
static struct udp_pcb *udp_hpvcrear_pcb;
static struct udp_pcb *udp_doip_pcb;
static struct udp_pcb *udp_someip_pcb;

static ip_addr_t pcip;
static u16_t     someip_port;

extern void Cpu0_WriteDriveCmd(const DriveCmd* c);

static void udp_receive_acc_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                                 const ip_addr_t *addr, u16_t port)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(upcb);
    LWIP_UNUSED_ARG(port);

    if (p == NULL)
        return;

    if (addr != NULL)
        ip_addr_copy(pcip, *addr);

    uint8 buf[32] __attribute__((aligned(4)));
    pbuf_copy_partial(p, buf, sizeof(buf), 0);

    DriveCmd cmd;
    cmd.control_mode    = buf[16];
    cmd.drive_direction = buf[17];
    memcpy(&cmd.target_speed, buf + 20, sizeof(float));
    memcpy(&cmd.accel_cmd,    buf + 24, sizeof(float));
    cmd.emergency_stop  = buf[28];

    Cpu0_WriteDriveCmd(&cmd);

    pbuf_free(p);
}

static void udp_drop_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(upcb);
    LWIP_UNUSED_ARG(addr);
    LWIP_UNUSED_ARG(port);

    if (p != NULL) {
        pbuf_free(p);
    }
}

static err_t UdpSend_WithPcb(struct udp_pcb *pcb, const ip_addr_t *dst_addr, u16_t dst_port, const void *data, u16_t len)
{
    if (pcb == NULL || dst_addr == NULL || data == NULL || len == 0) return ERR_ARG;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) return ERR_MEM;

    memcpy(p->payload, data, len);
    err_t err = udp_sendto(pcb, p, dst_addr, dst_port);
    pbuf_free(p);

    return err;
}

/* [ID: 5001] PC로 전송 (기억해둔 PC IP 사용, 목적지 포트는 5001) */
err_t UdpSendToPC(u16_t dst_port, const void *data, u16_t len) {
    // 일반 송신용 PCB 사용
    return UdpSend_WithPcb(udp_send_pcb, &pcip, dst_port, data, len);
}

/* [ID: 5002] RPi로 전송 (하드코딩된 IP 사용, 목적지 포트는 5002) */
err_t UdpSendToRPi(u16_t dst_port, const void *data, u16_t len) {
    ip_addr_t rpi_ip;
    IP4_ADDR(&rpi_ip, 192, 168, 10, 1);
    // 일반 송신용 PCB 사용
    return UdpSend_WithPcb(udp_send_pcb, &rpi_ip, dst_port, data, len);
}

/* [ID: 30492] SOME/IP 응답 전송 (PC IP + 기억해둔 상대방 포트 사용) */
err_t UdpSendSomeIpResponse(const void *data, u16_t len) {
    // 30492 포트가 바인딩된 udp_someip_pcb를 사용해야 src_port가 30492로 나감
    return UdpSend_WithPcb(udp_someip_pcb, &pcip, someip_port, data, len);
}

void UdpInit(void)
{
    //송신 전용
    udp_send_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    IP4_ADDR(&pcip, 192, 168, 10, 1);


    udp_ctr_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_ctr_pcb != NULL) {
        err_t err;
        err = udp_bind(udp_ctr_pcb, IP_ANY_TYPE, CTR_PORT);
        if (err == ERR_OK) {
            udp_recv(udp_ctr_pcb, udp_receive_acc_recv, NULL);
        }
    }

    udp_hpvcrear_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_hpvcrear_pcb != NULL) {
        err_t err;
        err = udp_bind(udp_hpvcrear_pcb, IP_ANY_TYPE, HPVC_REAR_COMMAND_PORT);
        if (err == ERR_OK) {
            udp_recv(udp_hpvcrear_pcb, udp_receive_acc_recv, NULL);
        }
    }

    udp_doip_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_doip_pcb != NULL) {
        err_t err;
        err = udp_bind(udp_doip_pcb, IP_ANY_TYPE, DOIP_PORT);
        if (err == ERR_OK) {
            udp_recv(udp_doip_pcb, udp_drop_recv, NULL);
        }
    }

    udp_someip_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_someip_pcb != NULL) {
        err_t err;
        err = udp_bind(udp_someip_pcb, IP_ANY_TYPE, SOMEIP_PORT);
        if (err == ERR_OK) {
            udp_recv(udp_someip_pcb, udp_drop_recv, NULL);
        }
    }
}

/* ------------------------------------------------------------------ */
/* UDP 송신 함수                                                       */
/* ------------------------------------------------------------------ */
err_t UdpSend(const ip_addr_t *dst_addr, u16_t dst_port,
              const void *data, u16_t len)
{
    struct pbuf *p;
    err_t err;

    if (udp_send_pcb == NULL || dst_addr == NULL || data == NULL || len == 0) {
        return ERR_ARG;
    }

    /* PBUF_RAM: payload를 새 버퍼에 복사해 둠 (호출자가 data 버퍼를 즉시 재사용 가능) */
    p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) {
        return ERR_MEM;
    }

    /* 사용자 데이터를 pbuf payload로 복사 */
    memcpy(p->payload, data, len);

    /* 송신 */
    err = udp_sendto(udp_send_pcb, p, dst_addr, dst_port);

    /* 송신 성공/실패와 무관하게 pbuf는 반드시 해제 */
    pbuf_free(p);

    return err;
}


/* 브로드캐스트 송신 (255.255.255.255) */
err_t UdpSendBroadcast(u16_t dst_port, const void *data, u16_t len)
{
    ip_addr_t bcast;
    /* lwIP 매크로: 255.255.255.255 */
    IP_ADDR4(&bcast, 255, 255, 255, 255);


    /* 브로드캐스트는 PCB에 SOF_BROADCAST 옵션이 있어야 송신 가능 */
    if (udp_send_pcb != NULL) {
        ip_set_option(udp_send_pcb, SOF_BROADCAST);
    }
    return UdpSend(&bcast, dst_port, data, len);
}

#endif /* LWIP_UDP */
