// SPDX-License-Identifier: Apache-2.0

#include "sim_slave.h"

#include "bus.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <sstream>

#include <lely/coapp/slave.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/io2/vcan.hpp>

// <lely/co/nmt.h> clashes with the C++ headers; only these are needed.
struct __co_emcy;
extern "C" __co_emcy *co_nmt_get_emcy(const __co_nmt *nmt);
extern "C" int co_emcy_clear(__co_emcy *emcy);

namespace canopen_sim
{

namespace
{

class Dcf
{
  public:
    void var(const char *idx, const char *name, unsigned type, const char *access, uint32_t value,
             bool map = false)
    {
        list_.push_back(idx);
        body_ << "[" << idx << "]\nParameterName=" << name << "\nObjectType=0x7\nDataType=0x"
              << std::hex << type << "\nAccessType=" << access << "\nDefaultValue=0x" << value
              << std::dec << "\nPDOMapping=" << (map ? 1 : 0) << "\n\n";
    }
    void str(const char *idx, const char *name, const char *value)
    {
        list_.push_back(idx);
        body_ << "[" << idx << "]\nParameterName=" << name
              << "\nObjectType=0x7\nDataType=0x9\nAccessType=const\nDefaultValue=" << value
              << "\nPDOMapping=0\n\n";
    }
    void rec(const char *idx, const char *name, unsigned subs, unsigned highest,
             const char *sub0_access = "ro")
    {
        list_.push_back(idx);
        body_ << "[" << idx << "]\nParameterName=" << name << "\nObjectType=0x9\nSubNumber=" << subs
              << "\n\n";
        sub(idx, 0, "Highest sub-index supported", 0x5, sub0_access, highest);
    }
    void sub(const char *idx, unsigned s, const char *name, unsigned type, const char *access,
             uint32_t value, bool map = false)
    {
        body_ << "[" << idx << "sub" << std::hex << std::uppercase << s
              << "]\nParameterName=" << name << "\nObjectType=0x7\nDataType=0x" << type
              << "\nAccessType=" << access << "\nDefaultValue=0x" << value << std::dec
              << "\nPDOMapping=" << (map ? 1 : 0) << "\n\n";
    }
    std::string text(unsigned node_id) const
    {
        std::ostringstream out;
        out << "[FileInfo]\nFileName=sim.dcf\nFileVersion=1\nFileRevision=0\nEDSVersion=4.0\n\n"
               "[DeviceInfo]\nVendorName=Simulated\nProductName=Generic I/O module\n"
               "BaudRate_125=1\nBaudRate_250=1\nBaudRate_500=1\nBaudRate_1000=1\n"
               "SimpleBootUpMaster=0\nSimpleBootUpSlave=1\nGranularity=8\nNrOfRXPDO=1\n"
               "NrOfTXPDO=1\nLSS_Supported=0\n\n[DeviceComissioning]\nNodeID="
            << node_id << "\n\n";
        std::ostringstream mand, opt, man;
        int nm = 0, no = 0, nf = 0;
        for (const auto &i : list_)
        {
            if (i == "1000" || i == "1001" || i == "1018")
                mand << ++nm << "=0x" << i << "\n";
            else if (i[0] == '2')
                man << ++nf << "=0x" << i << "\n";
            else
                opt << ++no << "=0x" << i << "\n";
        }
        out << "[MandatoryObjects]\nSupportedObjects=" << nm << "\n" << mand.str() << "\n";
        out << "[OptionalObjects]\nSupportedObjects=" << no << "\n" << opt.str() << "\n";
        out << "[ManufacturerObjects]\nSupportedObjects=" << nf << "\n" << man.str() << "\n";
        // Every dummy object may be mapped (CiA 306).
        out << "[DummyUsage]\n";
        for (int i = 1; i <= 7; ++i)
            out << "Dummy000" << i << "=1\n";
        out << "\n" << body_.str();
        return out.str();
    }

  private:
    std::vector<std::string> list_;
    std::ostringstream body_;
};

} // namespace

std::string sim_slave_dcf(const SimSlaveSpec &s)
{
    Dcf d;
    d.var("1000", "Device type", 0x7, "ro", s.device_type);
    d.var("1001", "Error register", 0x5, "ro", 0);
    d.str("1008", "Manufacturer device name", "Generic I/O module");
    d.var("100C", "Guard time", 0x6, "rw", 0);
    d.var("100D", "Life time factor", 0x5, "rw", 0);
    d.rec("1010", "Store parameters", 4, 3);
    d.sub("1010", 1, "Save all", 0x7, "rw", 1);
    d.sub("1010", 2, "Save communication", 0x7, "rw", 1);
    d.sub("1010", 3, "Save application", 0x7, "rw", 1);
    d.rec("1011", "Restore default parameters", 4, 3);
    d.sub("1011", 1, "Restore all", 0x7, "rw", 1);
    d.sub("1011", 2, "Restore communication", 0x7, "rw", 1);
    d.sub("1011", 3, "Restore application", 0x7, "rw", 1);
    d.var("1014", "COB-ID EMCY", 0x7, "rw", 0x80 + s.node_id);
    d.rec("1016", "Consumer heartbeat time", 2, 1);
    d.sub("1016", 1, "Consumer heartbeat time", 0x7, "rw", 0);
    d.var("1017", "Producer heartbeat time", 0x6, "rw", s.heartbeat_ms);
    d.rec("1018", "Identity object", 5, 4);
    d.sub("1018", 1, "Vendor-ID", 0x7, "ro", s.vendor_id);
    d.sub("1018", 2, "Product code", 0x7, "ro", s.product_code);
    d.sub("1018", 3, "Revision number", 0x7, "ro", s.revision);
    d.sub("1018", 4, "Serial number", 0x7, "ro", 0);
    d.rec("1029", "Error behaviour", 2, 1);
    d.sub("1029", 1, "Communication error", 0x5, "rw", 0);
    d.rec("1400", "RPDO communication parameter", 4, 5);
    d.sub("1400", 1, "COB-ID", 0x7, "rw", 0x200 + s.node_id);
    d.sub("1400", 2, "Transmission type", 0x5, "rw", 0xFF);
    d.sub("1400", 5, "Event timer", 0x6, "rw", 0);
    d.rec("1600", "RPDO mapping parameter", 3, 1, "rw");
    d.sub("1600", 1, "Mapped object", 0x7, "rw", 0x62000108);
    d.sub("1600", 2, "Mapped object", 0x7, "rw", 0);
    d.rec("1800", "TPDO communication parameter", 5, 5);
    d.sub("1800", 1, "COB-ID", 0x7, "rw", 0x180 + s.node_id);
    d.sub("1800", 2, "Transmission type", 0x5, "rw", 0xFF);
    d.sub("1800", 3, "Inhibit time", 0x6, "rw", 0);
    d.sub("1800", 5, "Event timer", 0x6, "rw", 20);
    d.rec("1A00", "TPDO mapping parameter", 3, 2, "rw");
    d.sub("1A00", 1, "Mapped object", 0x7, "rw", 0x60000108);
    d.sub("1A00", 2, "Mapped object", 0x7, "rw", 0x64010110);
    d.var("2100", "Parameter", 0x7, s.parameter_read_only ? "ro" : "rw", 0);
    d.var("2101", "Pre-operational parameter", 0x6, "rw", 10);
    // Lely reads a REAL32 default as its bit pattern: 1.5.
    d.var("2102", "Real parameter", 0x8, "rw", 0x3FC00000);
    d.rec("6000", "Digital inputs", 2, 1);
    d.sub("6000", 1, "Inputs 1-8", 0x5, "rw", 0, true);
    d.rec("6200", "Digital outputs", 2, 1);
    d.sub("6200", 1, "Outputs 1-8", 0x5, "rw", 0, true);
    d.rec("6401", "Analogue inputs", 2, 1);
    d.sub("6401", 1, "Input 1", 0x3, "rw", 0, true);
    return d.text(s.node_id);
}

namespace
{

class Slave : public lely::canopen::BasicSlave
{
  public:
    Slave(ev_exec_t *exec, lely::io::TimerBase &timer, lely::io::CanChannelBase &chan,
          const std::string &dcf, std::atomic<uint8_t> &nmt, std::atomic<unsigned> &restores,
          std::atomic<unsigned> &saves, std::atomic<unsigned> &output_writes, bool refuse)
        : BasicSlave(exec, timer, chan, dcf, ""), nmt_(nmt)
    {
        // Also called for each RPDO that maps the outputs.
        OnWrite<uint8_t>(0x6200, 1,
                         [&output_writes](uint16_t, uint8_t, uint8_t &, uint8_t) -> std::error_code
                         {
                             ++output_writes;
                             return {};
                         });
        for (uint8_t sub = 1; sub <= 3; ++sub)
            OnWrite<uint32_t>(
                0x1010, sub,
                [&saves](uint16_t, uint8_t, uint32_t &value, uint32_t) -> std::error_code
                {
                    if (value != 0x65766173)
                        return lely::canopen::SdoErrc::DATA;
                    ++saves;
                    value = 1;
                    return {};
                });
        // Taken only while the node is not operational.
        OnWrite<uint16_t>(0x2101, 0,
                          [this](uint16_t, uint8_t, uint16_t &, uint16_t) -> std::error_code
                          {
                              if (nmt_.load() == 1)
                                  return lely::canopen::SdoErrc::DATA_DEV;
                              return {};
                          });
        // Every object comes back from the DCF at the next reset anyway, so a restore only has
        // to be accepted and counted.
        for (uint8_t sub = 1; sub <= 3; ++sub)
            OnWrite<uint32_t>(
                0x1011, sub,
                [&restores, refuse](uint16_t, uint8_t, uint32_t &value, uint32_t) -> std::error_code
                {
                    if (refuse)
                        return lely::canopen::SdoErrc::NO_WRITE;
                    if (value != 0x64616F6C)
                        return lely::canopen::SdoErrc::DATA;
                    ++restores;
                    value = 1;
                    return {};
                });
    }

    void SendEmcy(uint16_t code, uint8_t reg)
    {
        std::lock_guard<lely::util::BasicLockable> lock(*this);
        Error(code, reg);
    }
    void ClearEmcy()
    {
        std::lock_guard<lely::util::BasicLockable> lock(*this);
        co_emcy_clear(co_nmt_get_emcy(nmt()));
    }

  protected:
    void OnCommand(lely::canopen::NmtCommand cs) noexcept override
    {
        nmt_ = static_cast<uint8_t>(cs);
    }

  private:
    std::atomic<uint8_t> &nmt_;
};

} // namespace

SimSlave::SimSlave(const SimSlaveSpec &spec, std::string dcf_path,
                   std::shared_ptr<lely::io::VirtualCanController> vbus, std::string interface)
    : spec_(spec), dcf_path_(std::move(dcf_path)), vbus_(std::move(vbus)),
      interface_(std::move(interface))
{
}

SimSlave::~SimSlave()
{
    stop();
}

void SimSlave::start()
{
    if (thread_.joinable())
        return;
    std::ofstream(dcf_path_, std::ios::trunc) << sim_slave_dcf(spec_);
    stop_     = false;
    ready_    = false;
    restores_ = 0;
    saves_    = 0;
    thread_   = std::thread([this] { thread_main(); });
    while (!ready_ && !stop_)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void SimSlave::stop()
{
    stop_ = true;
    if (thread_.joinable())
        thread_.join();
}

void SimSlave::set_inputs(uint8_t digital, int16_t analogue)
{
    digital_      = digital;
    analogue_     = analogue;
    inputs_dirty_ = true;
}

void SimSlave::send_emcy(uint16_t code, uint8_t reg)
{
    emcy_reg_     = reg;
    emcy_request_ = 0x10000u | code;
}

void SimSlave::clear_emcy()
{
    emcy_clear_ = true;
}

void SimSlave::set_tpdo_enabled(bool on)
{
    tpdo_request_ = on ? 1 : 0;
}

void SimSlave::thread_main()
{
    try
    {
        lely::io::IoGuard io_guard;
        lely::io::Context ctx;
        lely::io::Poll poll(ctx, canopen_plugin::wake_signal());
        lely::ev::Loop loop(poll.get_poll());
        auto exec = loop.get_executor();
        lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);
        std::unique_ptr<lely::io::VirtualCanChannel> vchan;
        std::unique_ptr<lely::io::CanController> ctrl;
        std::unique_ptr<lely::io::CanChannel> chan;
        lely::io::CanChannelBase *base = nullptr;
        if (vbus_)
        {
            vchan.reset(new lely::io::VirtualCanChannel(ctx, exec));
            vchan->open(*vbus_);
            base = vchan.get();
        }
        else
        {
            ctrl.reset(new lely::io::CanController(interface_.c_str()));
            chan.reset(new lely::io::CanChannel(poll, exec));
            chan->open(*ctrl);
            base = chan.get();
        }
        Slave slave(exec, timer, *base, dcf_path_, nmt_state_, restores_, saves_, output_writes_,
                    spec_.restore_refused);
        slave.Reset();
        ready_ = true;
        while (!stop_)
        {
            loop.run_for(std::chrono::milliseconds(5));
            if (inputs_dirty_.exchange(false))
            {
                slave.Write<uint8_t>(0x6000, 1, digital_.load());
                slave.Write<int16_t>(0x6401, 1, analogue_.load());
            }
            uint32_t emcy = emcy_request_.exchange(0);
            if (emcy)
                slave.SendEmcy(static_cast<uint16_t>(emcy & 0xFFFF), emcy_reg_.load());
            if (emcy_clear_.exchange(false))
                slave.ClearEmcy();
            int tpdo = tpdo_request_.exchange(-1);
            if (tpdo >= 0)
            {
                // Bit 31 of the COB-ID: PDO not valid.
                uint32_t cob = slave.Read<uint32_t>(0x1800, 1);
                slave.Write<uint32_t>(0x1800, 1, tpdo ? cob & ~0x80000000u : cob | 0x80000000u);
            }
            outputs_   = slave.Read<uint8_t>(0x6200, 1);
            parameter_ = slave.Read<uint32_t>(0x2100, 0);
        }
        ctx.shutdown();
        loop.run_for(std::chrono::milliseconds(50));
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "simulated node %u: %s\n", spec_.node_id, e.what());
        ready_ = true;
    }
}

} // namespace canopen_sim
