// SPDX-License-Identifier: Apache-2.0
//
// slave_none.cpp - the slave in a build without CANopenNode: no file with devices is accepted
// (slave_supported() is false), so none of these run.

#include "slave.h"

namespace canopen_plugin
{

bool slave_supported()
{
    return false;
}

struct Slave::Impl
{
};

std::unique_ptr<SlaveLink> socket_slave_link(const std::string &, unsigned)
{
    return nullptr;
}

std::unique_ptr<SlaveLink> virtual_slave_link(std::shared_ptr<lely::io::VirtualCanController>)
{
    return nullptr;
}

Slave::Slave(const DeviceConfig &dev, std::unique_ptr<SlaveLink> link)
    : dev_(dev), link_(std::move(link)), impl_(new Impl)
{
}

Slave::~Slave() = default;

bool Slave::start(std::string &problem)
{
    problem = "this runtime has no CANopen slave support";
    return false;
}

void Slave::stop() {}
void Slave::copy_to_plc(const plugin_runtime_args_t &) {}
void Slave::copy_from_plc(const plugin_runtime_args_t &) {}
void Slave::set_plc_running(bool) {}
bool Slave::forget_lss()
{
    return false;
}
void Slave::thread_main() {}

SlaveSnapshot Slave::snapshot() const
{
    return status_;
}

void Slave::plc_status(canopen_plc_slave_status &out) const
{
    out = canopen_plc_slave_status{};
}
int Slave::plc_nmt(uint8_t)
{
    return CANOPEN_PLC_NO_DRIVER;
}
int Slave::plc_nmt_done(uint8_t, bool &) const
{
    return CANOPEN_PLC_NO_DRIVER;
}
int Slave::plc_emcy_claim(uint32_t &)
{
    return CANOPEN_PLC_NO_DRIVER;
}
int Slave::plc_emcy(uint32_t, bool, uint16_t, uint32_t)
{
    return CANOPEN_PLC_NO_DRIVER;
}
int Slave::plc_written(uint16_t, uint8_t, uint32_t &) const
{
    return CANOPEN_PLC_NO_DRIVER;
}
bool Slave::program_code_allowed(uint16_t)
{
    return false;
}
uint8_t Slave::error_class_bit(uint16_t)
{
    return 0;
}

} // namespace canopen_plugin
