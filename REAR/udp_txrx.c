#include "udp_txrx.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "ip_addr.h"

#include <string.h>

#if LWIP_UDP

#define CTR_PORT 5000
#define DOIP_PORT 13400
#define SOMEIP_PORT 30492

static struct udp_pcb *udp_send_pcb;
static struct udp_pcb *udp_ctr_pcb;
static struct udp_pcb *udp_doip_pcb;
static struct udp_pcb *udp_someip_pcb;

static ip_addr_t pcip;
static u16_t     someip_port;


extern void Cpu0_WriteAcc(float new_acc);

static void udp_receive_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                             const ip_addr_t *addr, u16_t port)
{
    LWIP_UNUSED_ARG(arg);

    if (p != NULL)
    {
        if (addr != NULL)
        {
            ip_addr_copy(pcip, *addr);
        }

        /* 4바이트(float) 수신 확인 — tot_len 기준 (chained pbuf 대응) */
        if (p->tot_len >= sizeof(float))
        {
            uint8_t raw[4];
            float received_acc;

            /* payload를 안전하게 복사 (정렬/분할 pbuf 문제 회피) */
            pbuf_copy_partial(p, raw, sizeof(raw), 0);

            /* PC(x86)와 TC375 모두 little-endian -> 바이트 스왑 없이 그대로 복사.
             * (PC 송신부가 빅엔디안/네트워크 바이트오더로 보낸다면, 그때만 스왑) */
            memcpy(&received_acc, raw, sizeof(float));

            /* 더블 버퍼링 알고리즘이 적용된 함수로 안전하게 대입 */
            Cpu0_WriteAcc(received_acc);

        }

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
    IP4_ADDR(&rpi_ip, 192, 168, 20, 2);
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


    udp_ctr_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_ctr_pcb != NULL) {
        err_t err;
        err = udp_bind(udp_ctr_pcb, IP_ANY_TYPE, CTR_PORT);
        if (err == ERR_OK) {
            udp_recv(udp_ctr_pcb, udp_receive_recv, NULL);
        }
    }

    udp_doip_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_doip_pcb != NULL) {
        err_t err;
        err = udp_bind(udp_doip_pcb, IP_ANY_TYPE, DOIP_PORT);
        if (err == ERR_OK) {
            udp_recv(udp_doip_pcb, udp_receive_recv, NULL);
        }
    }

    udp_someip_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_someip_pcb != NULL) {
        err_t err;
        err = udp_bind(udp_someip_pcb, IP_ANY_TYPE, SOMEIP_PORT);
        if (err == ERR_OK) {
            udp_recv(udp_someip_pcb, udp_receive_recv, NULL);
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
