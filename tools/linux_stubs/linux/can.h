// Minimal Linux SocketCAN header stubs for host-only syntax checks on macOS.
// DO NOT USE ON THE REAL TARGET (the real Linux headers must be picked up
// when you build on the Raspberry Pi).
#pragma once

#include <cstdint>
#include <sys/socket.h>

#ifndef AF_CAN
#define AF_CAN 29
#endif
#ifndef PF_CAN
#define PF_CAN AF_CAN
#endif
#ifndef SOL_CAN_BASE
#define SOL_CAN_BASE 100
#endif
#ifndef SOL_CAN_RAW
#define SOL_CAN_RAW (SOL_CAN_BASE + 1)
#endif

#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U
#define CAN_SFF_MASK 0x000007FFU
#define CAN_EFF_MASK 0x1FFFFFFFU

typedef uint32_t canid_t;

struct can_frame {
    canid_t  can_id;
    uint8_t  can_dlc;
    uint8_t  __pad;
    uint8_t  __res0;
    uint8_t  __res1;
    uint8_t  data[8];
};

struct sockaddr_can {
    sa_family_t can_family;
    int         can_ifindex;
    union {
        struct { canid_t rx_id, tx_id; } tp;
    } can_addr;
};

struct can_filter {
    canid_t can_id;
    canid_t can_mask;
};
