// SPDX-License-Identifier: Apache-2.0
//
// canopen_sim_slave - a simulated generic I/O module on a SocketCAN interface, for the
// end-to-end check of a PLC program that copies the module's digital inputs to its outputs:
//
//   canopen_sim_slave --interface vcan0 [--node 2] [--steps 5] [--timeout 60]
//                     [--expect-parameter 0x12345678]
//
// It sets its digital inputs to 1, 2, ... and waits each time until its outputs follow. Exit
// status 0 when every step matched (and the parameter was written), 1 otherwise.

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
    std::printf("PASS\n");
    return 0;
}
