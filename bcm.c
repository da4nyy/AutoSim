/* bcm.c - A simple Body Control Module simulator
 * Listen for door lock bitmask commands (CAN ID = 0x123)
 * Respond with door status bitmask (CAN ID = 0x124)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// Our door command & status IDs
#define BCM_CMD_ID   0x123  // ICSim/Controls -> BCM
#define BCM_STAT_ID  0x124  // BCM -> ICSim/Controls

// Bit definitions (per door)
#define DOOR1_BIT  (1 << 0) // 0x01
#define DOOR2_BIT  (1 << 1) // 0x02
#define DOOR3_BIT  (1 << 2) // 0x04
#define DOOR4_BIT  (1 << 3) // 0x08


int main() {
    // 1) Create raw CAN socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        return 1;
    }

    // 2) Bind to vcan1 (this is our BCM interface)
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan1");
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(s);
        return 1;
    }

    struct sockaddr_can addr;
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return 1;
    }

    printf("[BCM] Bound to vcan1, listening for door bitmask on ID=0x123.\n");

    // We'll store up to 4 bits (1=locked, 0=unlocked)
    // e.g. doorState=0x05 => door1 and door3 locked, door2 and door4 unlocked
    unsigned char doorState = 0x0;

    while (1) {
        struct can_frame rx;
        int nbytes = read(s, &rx, sizeof(rx));
        if (nbytes < 0) {
            perror("read");
            break;
        }
        if ((size_t)nbytes < sizeof(rx)) {
            fprintf(stderr, "[BCM] Incomplete CAN frame\n");
            continue;
        }

        // Only handle 0x123
        if (rx.can_id == BCM_CMD_ID) {
            unsigned char newDoorBits = rx.data[0]; // The new bitmask

            // Log each door's state
            printf("[BCM] Received bitmask=0x%02X =>\n", newDoorBits);
            for (int i = 0; i < 4; i++) {
                int isLocked = (newDoorBits & (1 << i)) ? 1 : 0;
                printf("   Door%d: %s\n", i + 1, isLocked ? "Locked" : "Unlocked");
            }
            // Store it
            doorState = newDoorBits;

            // Now send back a status (ID=0x124) with that same bitmask
            struct can_frame tx;
            memset(&tx, 0, sizeof(tx));
            tx.can_id  = BCM_STAT_ID;
            tx.can_dlc = 1;
            tx.data[0] = doorState;  // reflect the new door bits

            if (write(s, &tx, sizeof(tx)) != sizeof(tx)) {
                perror("[BCM] write");
            } else {
                printf("[BCM] => Sent door bitmask on 0x124: 0x%02X\n", doorState);
            }
        }
    }

    close(s);
    return 0;
}
