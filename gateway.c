/* gateway.c - Minimal two-way bridging
 *
 * Usage: ./gateway vcan0 vcan1
 *
 * Relays door bitmask frames:
 *   0x123 from vcan0 -> vcan1
 *   0x124 from vcan1 -> vcan0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// IDs we want to forward
#define BCM_CMD_ID   0x123
#define BCM_STAT_ID  0x124

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <srcIf> <dstIf>\n", argv[0]);
        exit(1);
    }

    int s1, s2;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // -- Socket 1 (srcIf)
    if ((s1 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket s1");
        exit(1);
    }
    strcpy(ifr.ifr_name, argv[1]);
    if (ioctl(s1, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl s1");
        close(s1);
        exit(1);
    }
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s1, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind s1");
        close(s1);
        exit(1);
    }
    printf("[Gateway] Bound to %s (socket s1)\n", argv[1]);

    // -- Socket 2 (dstIf)
    if ((s2 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket s2");
        close(s1);
        exit(1);
    }
    strcpy(ifr.ifr_name, argv[2]);
    if (ioctl(s2, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl s2");
        close(s2);
        close(s1);
        exit(1);
    }
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s2, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind s2");
        close(s2);
        close(s1);
        exit(1);
    }
    printf("[Gateway] Bound to %s (socket s2)\n", argv[2]);

    printf("[Gateway] Forwarding: 0x123 from %s->%s, 0x124 from %s->%s\n",
           argv[1], argv[2], argv[2], argv[1]);

    fd_set fds;
    int maxfd = (s1 > s2) ? s1 : s2;

    while (1) {
        FD_ZERO(&fds);
        FD_SET(s1, &fds);
        FD_SET(s2, &fds);

        int ret = select(maxfd+1, &fds, NULL, NULL, NULL);
        if (ret < 0) {
            perror("select");
            break;
        }

        // If there's data on s1, read & forward certain IDs
        if (FD_ISSET(s1, &fds)) {
            struct can_frame frame;
            int nbytes = read(s1, &frame, sizeof(frame));
            if (nbytes < 0) {
                perror("[Gateway] read s1");
                break;
            }
            if (nbytes == sizeof(frame)) {
                if (frame.can_id == BCM_CMD_ID) {
                    // Forward 0x123 from s1->s2
                    int w = write(s2, &frame, sizeof(frame));
                    if (w < 0) perror("[Gateway] write s2");
                    else printf("[Gateway] s1->s2: Forwarded ID=0x%03X\n", frame.can_id);
                }
            }
        }

        // If there's data on s2, read & forward certain IDs
        if (FD_ISSET(s2, &fds)) {
            struct can_frame frame;
            int nbytes = read(s2, &frame, sizeof(frame));
            if (nbytes < 0) {
                perror("[Gateway] read s2");
                break;
            }
            if (nbytes == sizeof(frame)) {
                if (frame.can_id == BCM_STAT_ID) {
                    // Forward 0x124 from s2->s1
                    int w = write(s1, &frame, sizeof(frame));
                    if (w < 0) perror("[Gateway] write s1");
                    else printf("[Gateway] s2->s1: Forwarded ID=0x%03X\n", frame.can_id);
                }
            }
        }
    }

    close(s1);
    close(s2);
    return 0;
}
