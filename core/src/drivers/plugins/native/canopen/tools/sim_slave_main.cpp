// SPDX-License-Identifier: Apache-2.0
//
// canopen_sim_slave - a simulated generic I/O module on a SocketCAN interface, for the
// end-to-end check of a PLC program that copies the module's digital inputs to its outputs:
//
//   canopen_sim_slave --interface vcan0 [--node 2] [--steps 5] [--timeout 60]
//                     [--expect-parameter 0x12345678] [--expect-master-loss 5] [--linger 10]
//
// It sets its digital inputs to 1, 2, ... and waits each time until its outputs follow. With
// --expect-master-loss it then prints "watching the master" and waits up to that many seconds
// for its master watch (0x1016) to put it in pre-operational. With --linger it stays on the bus
// that many seconds more, so its status can be read. Exit status 0 when every step
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
    if (linger_s > 0)
        std::this_thread::sleep_for(std::chrono::seconds(linger_s));
    std::printf("PASS\n");
    return 0;
}
