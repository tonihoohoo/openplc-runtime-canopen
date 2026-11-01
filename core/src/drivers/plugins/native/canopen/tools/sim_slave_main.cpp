// SPDX-License-Identifier: Apache-2.0
//
// canopen_sim_slave - a simulated generic I/O module on a SocketCAN interface, for the
// end-to-end check of a PLC program that copies the module's digital inputs to its outputs:
//
//   canopen_sim_slave --interface vcan0 [--node 2] [--steps 5] [--timeout 60]
//                     [--expect-parameter 0x12345678] [--expect-master-loss 5] [--linger 10]
//                     [--blocks 1] [--parameter-read-only 1] [--expect-resends 10]
//
// It sets its digital inputs to 1, 2, ... and waits each time until its outputs follow. With
// --expect-master-loss it then prints "watching the master" and waits up to that many seconds
// for its master watch (0x1016) to put it in pre-operational. With --linger it stays on the bus
// that many seconds more, so its status can be read. With --parameter-read-only 1 it refuses
// writes to 0x2100. With --expect-resends N it then needs at least N writes of its unchanged
// outputs within 2 s, sent by the master for an RPDO deadline. With --blocks 1 it then reads commands for
// the PLC function block test program from stdin, one per line:
//   run <n>   sets its inputs to 0x40 + n, sends an emergency 0x5000 every 500 ms and waits up to
//             60 s for the program to write 0xC0DE0000 + n * 0x1000 + 0x1FF to 0x2100; prints
//             "run <n> started" when the program writes its start mark (0x0BAD0000 + n * 0x1000)
//             and "run <n> ok" or "run <n> FAIL ..."
//   quit      ends.
// Exit status 0 when every step
// matched (and the parameter was written, and the master loss was seen), 1 otherwise.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "bus.h"
#include "sim_slave.h"

using canopen_sim::SimSlave;
using canopen_sim::SimSlaveSpec;

namespace
{

// Runs the commands of the PLC block test (see the header); false on a failed run.
bool run_blocks(SimSlave &slave)
{
    std::printf("blocks: ready\n");
    std::fflush(stdout);
    char line[64];
    bool ok = true;
    while (std::fgets(line, sizeof line, stdin))
    {
        unsigned n = 0;
        if (std::strncmp(line, "quit", 4) == 0)
            break;
        if (std::sscanf(line, "run %u", &n) != 1 || n > 15)
        {
            std::printf("blocks: unknown command %s", line);
            std::fflush(stdout);
            continue;
        }
        const uint32_t mark   = 0x0BAD0000u + n * 0x1000u;
        const uint32_t wanted = 0xC0DE0000u + n * 0x1000u + 0x1FFu;
        slave.set_inputs(static_cast<uint8_t>(0x40 + n), 0);
        auto end       = std::chrono::steady_clock::now() + std::chrono::seconds(60);
        auto next_emcy = std::chrono::steady_clock::now();
        bool started   = false;
        uint32_t last  = slave.parameter();
        bool done      = false;
        while (std::chrono::steady_clock::now() < end)
        {
            uint32_t p = slave.parameter();
            if (p != last)
            {
                std::printf("run %u: 0x2100 = 0x%08X\n", n, p);
                last = p;
            }
            if (p == mark && !started)
            {
                started = true;
                std::printf("run %u started\n", n);
                std::fflush(stdout);
            }
            if (p == wanted)
            {
                done = true;
                break;
            }
            if (std::chrono::steady_clock::now() >= next_emcy)
            {
                slave.send_emcy(0x5000, 0x01);
                next_emcy += std::chrono::milliseconds(500);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (done)
            std::printf("run %u ok\n", n);
        else
        {
            ok = false;
            std::printf("run %u FAIL: 0x2100 = 0x%08X, expected 0x%08X\n", n, slave.parameter(),
                        wanted);
        }
        std::fflush(stdout);
    }
    return ok;
}

} // namespace

int main(int argc, char **argv)
{
    std::string iface;
    SimSlaveSpec spec;
    int steps           = 5;
    int timeout_s       = 60;
    bool check_param    = false;
    unsigned long param = 0;
    int master_loss_s   = 0;
    int linger_s        = 0;
    bool blocks         = false;
    int resends         = 0;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        std::string opt = argv[i];
        const char *val = argv[i + 1];
        if (opt == "--interface")
            iface = val;
        else if (opt == "--node")
            spec.node_id = static_cast<unsigned>(std::strtoul(val, nullptr, 0));
        else if (opt == "--steps")
            steps = std::atoi(val);
        else if (opt == "--timeout")
            timeout_s = std::atoi(val);
        else if (opt == "--linger")
            linger_s = std::atoi(val);
        else if (opt == "--blocks")
            blocks = std::atoi(val) != 0;
        else if (opt == "--parameter-read-only")
            spec.parameter_read_only = std::atoi(val) != 0;
        else if (opt == "--expect-resends")
            resends = std::atoi(val);
        else if (opt == "--expect-master-loss")
            master_loss_s = std::atoi(val);
        else if (opt == "--expect-parameter")
        {
            check_param = true;
            param       = std::strtoul(val, nullptr, 0);
        }
        else
        {
            std::fprintf(stderr, "unknown option %s\n", opt.c_str());
            return 2;
        }
    }
    if (iface.empty())
    {
        std::fprintf(stderr, "usage: %s --interface <can> [options]\n", argv[0]);
        return 2;
    }

    canopen_plugin::prepare_wake_signal();
    SimSlave slave(spec, "/tmp/canopen_sim_node" + std::to_string(spec.node_id) + ".dcf", nullptr,
                   iface);
    slave.start();
    std::printf("simulated node %u on %s\n", spec.node_id, iface.c_str());
    std::fflush(stdout);

    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    for (int step = 1; step <= steps; ++step)
    {
        slave.set_inputs(static_cast<uint8_t>(step), 0);
        while (slave.outputs() != step)
        {
            if (std::chrono::steady_clock::now() > end)
            {
                std::printf("FAIL: inputs %d, outputs still %u\n", step, slave.outputs());
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::printf("step %d: outputs follow the inputs\n", step);
        std::fflush(stdout);
    }
    if (check_param && slave.parameter() != param)
    {
        std::printf("FAIL: parameter 0x%08X, expected 0x%08lX\n", slave.parameter(), param);
        return 1;
    }
    if (resends > 0)
    {
        const unsigned before = slave.output_writes();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        const unsigned got = slave.output_writes() - before;
        if (got < static_cast<unsigned>(resends))
        {
            std::printf("FAIL: %u writes of the unchanged outputs in 2 s, expected %d\n", got,
                        resends);
            return 1;
        }
        std::printf("unchanged outputs written %u times in 2 s\n", got);
        std::fflush(stdout);
    }
    if (master_loss_s > 0)
    {
        std::printf("watching the master\n");
        std::fflush(stdout);
        auto until = std::chrono::steady_clock::now() + std::chrono::seconds(master_loss_s);
        while (slave.last_command() != 0x80)
        {
            if (std::chrono::steady_clock::now() > until)
            {
                std::printf("FAIL: still operational %d s after the master's heartbeat was due\n",
                            master_loss_s);
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::printf("master loss seen: pre-operational\n");
    }
    if (blocks && !run_blocks(slave))
        return 1;
    if (linger_s > 0)
        std::this_thread::sleep_for(std::chrono::seconds(linger_s));
    std::printf("PASS\n");
    return 0;
}
