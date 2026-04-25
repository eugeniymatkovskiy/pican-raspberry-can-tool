#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstdint>

#include "obd2_pids.h"
#include "obd2_parser.h"

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cerr << "Error: can't create a socket" << std::endl;
        return 1;
    }

    std::strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error: can't bind socket" << std::endl;
        return 1;
    }

    std::cout << "Connected to can0. Waiting for responses from the engine..." << std::endl;

    struct can_frame frame;
    while (true) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes > 0) {
            uint32_t can_id = frame.can_id;
            if (can_id & CAN_EFF_FLAG) {
                can_id &= CAN_EFF_MASK;
            }

            // 1. Ignore the spam from Gateway
            if (can_id == OBD2::CAN_ID_GATEWAY) {
                continue; 
            }

            // 2. Catch the response from the engine brain (ECU)
            // The standard response has ID 0x7E8 (or 0x7E9 from the gearbox)
            if (can_id == OBD2::CAN_ID_ECU || can_id == OBD2::CAN_ID_GEARBOX) {
                OBD2::parseAndPrintResponse(can_id, frame);
            } else {
                std::cout << "Unknown ID: " << std::hex << can_id << std::endl;
            }
        }
    }

    close(s);
    return 0;
}
