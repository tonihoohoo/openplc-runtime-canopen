// SPDX-License-Identifier: Apache-2.0
//
// host_slave.cpp - runs the editor's board firmware module (canopen_slave.cpp, built with
// CANOPEN_SLAVE_HOST) on Linux: SocketCAN in place of TWAI, a table in place of the program's
// located variables, and the slave e2e program written out in C. The PLC is stopped and started
// by the editor's 0x4B frame (Modbus RTU, slave 1) on a TCP port, as on the board's serial port.
//
//     host_slave vcan0 5020

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "canopen_slave.h"

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t rtr;
    uint8_t data[8];
} co_board_frame_t;

static int can_fd          = -1;
static const char *can_if  = "vcan0";
static volatile bool plc   = true;
static volatile bool quit  = false;

extern "C" bool co_board_can_open(unsigned kbit, int tx_pin, int rx_pin)
{
    (void)kbit;
    (void)tx_pin;
    (void)rx_pin;
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0)
        return false;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof ifr);
    snprintf(ifr.ifr_name, sizeof ifr.ifr_name, "%s", can_if);
    if (ioctl(can_fd, SIOCGIFINDEX, &ifr) < 0)
        return false;
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof addr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof addr) < 0)
        return false;
    return fcntl(can_fd, F_SETFL, O_NONBLOCK) == 0;
}

extern "C" bool co_board_can_send(const co_board_frame_t *f)
{
    struct can_frame c;
    memset(&c, 0, sizeof c);
    c.can_id  = f->id | (f->rtr ? CAN_RTR_FLAG : 0);
    c.can_dlc = f->dlc;
    memcpy(c.data, f->data, 8);
    return write(can_fd, &c, sizeof c) == static_cast<ssize_t>(sizeof c);
}

extern "C" bool co_board_can_receive(co_board_frame_t *f)
{
    struct can_frame c;
    for (;;)
    {
        if (read(can_fd, &c, sizeof c) != static_cast<ssize_t>(sizeof c))
            return false;
        if (c.can_id & (CAN_EFF_FLAG | CAN_ERR_FLAG))
            continue;
        f->id  = c.can_id & CAN_SFF_MASK;
        f->rtr = (c.can_id & CAN_RTR_FLAG) ? 1 : 0;
        f->dlc = c.can_dlc > 8 ? 8 : c.can_dlc;
        memcpy(f->data, c.data, 8);
        return true;
    }
}

extern "C" uint8_t co_board_can_state(uint16_t *tec, uint16_t *rec)
{
    *tec = 0;
    *rec = 0;
    return 0;
}

extern "C" void co_board_can_recover(void) {}

// The program's located variables: one 8-byte slot per address, made on first use.
struct Slot
{
    char area;
    char size;
    uint16_t index;
    uint8_t bit;
    uint64_t value;
};
static Slot slots[128];
static unsigned slot_count = 0;

extern "C" void *co_board_variable(char area, char size, uint16_t index, uint8_t bit)
{
    for (unsigned i = 0; i < slot_count; ++i)
        if (slots[i].area == area && slots[i].size == size && slots[i].index == index &&
            slots[i].bit == bit)
            return &slots[i].value;
    if (slot_count == sizeof slots / sizeof slots[0])
        return NULL;
    slots[slot_count] = Slot{area, size, index, bit, 0};
    return &slots[slot_count++].value;
}

extern "C" bool co_board_plc_running(void)
{
    return plc;
}

extern "C" uint32_t co_board_micros(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<uint32_t>(t.tv_sec * 1000000ull + t.tv_nsec / 1000);
}

template <typename T> static T &var(char area, char size, uint16_t index, uint8_t bit = 0)
{
    return *static_cast<T *>(co_board_variable(area, size, index, bit));
}

// The slave e2e program (e2e/fixtures/canopen-slave-board in the editor).
static void program(void)
{
    var<int16_t>('Q', 'W', 0)  = var<int16_t>('I', 'W', 0);
    var<int16_t>('Q', 'W', 2)  = var<int16_t>('I', 'W', 0);
    var<uint16_t>('Q', 'W', 1) = var<uint16_t>('I', 'W', 1);
    for (uint8_t b = 0; b < 8; ++b)
        var<uint8_t>('Q', 'X', 0, b) = var<uint8_t>('I', 'X', 0, b);
}

static uint16_t crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i)
    {
        crc ^= p[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

// Reads 0x4B frames [1][0x4B][state][crc lo][crc hi] from the TCP client.
static void serve(int listen_fd, int &client, uint8_t *buf, size_t &len)
{
    if (client < 0)
    {
        client = accept(listen_fd, NULL, NULL);
        if (client >= 0)
            fcntl(client, F_SETFL, O_NONBLOCK);
        return;
    }
    const ssize_t n = read(client, buf + len, 64 - len);
    if (n == 0)
    {
        close(client);
        client = -1;
        len    = 0;
        return;
    }
    if (n < 0)
        return;
    len += static_cast<size_t>(n);
    while (len >= 5)
    {
        const uint16_t crc = static_cast<uint16_t>(buf[3] | (buf[4] << 8));
        if (buf[0] == 1 && buf[1] == 0x4B && crc16(buf, 3) == crc)
        {
            plc = buf[2] != 0;
            printf("plc %s\n", plc ? "run" : "stop");
            fflush(stdout);
            memmove(buf, buf + 5, len - 5);
            len -= 5;
        }
        else
        {
            memmove(buf, buf + 1, len - 1);
            len -= 1;
        }
    }
}

static void on_signal(int)
{
    quit = true;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "usage: host_slave <can interface> <tcp port>\n");
        return 2;
    }
    can_if = argv[1];
    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);

    const int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    const int one       = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(atoi(argv[2])));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(listen_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof addr) < 0 ||
        listen(listen_fd, 1) < 0)
    {
        perror("listen");
        return 1;
    }
    fcntl(listen_fd, F_SETFL, O_NONBLOCK);

    canopen_slave_init();
    printf("host slave on %s\n", can_if);
    fflush(stdout);
    int client = -1;
    uint8_t buf[64];
    size_t len       = 0;
    uint32_t scan_us = co_board_micros();
    while (!quit)
    {
        canopen_slave_task();
        serve(listen_fd, client, buf, len);
        const uint32_t now = co_board_micros();
        if (now - scan_us >= 5000)
        {
            scan_us = now;
            canopen_slave_scan_start();
            if (plc)
                program();
            canopen_slave_scan_end();
        }
        struct timespec t = {0, 500000};
        nanosleep(&t, NULL);
    }
    return 0;
}
