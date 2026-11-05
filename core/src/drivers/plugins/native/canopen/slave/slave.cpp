// SPDX-License-Identifier: Apache-2.0

#include "slave.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>

#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/vcan.hpp>

#include "CANopen.h"
#include "bus_monitor.h"
#include "bus_off_recovery.h"
#include "can_link.h"
#include "log.h"
#include "lss_store.h"
#include "slave_od.h"

namespace canopen_plugin
{

bool slave_supported()
{
    return true;
}

namespace
{

using Clock = std::chrono::steady_clock;

// Error status bit of a PLC stop: the first manufacturer bit (CiA 301 leaves them free).
constexpr uint8_t kPlcStopBit = CO_EM_MANUFACTURER_START;
constexpr int kRetryMs        = 1000;
constexpr int kTickMs         = 1;
constexpr uint16_t kLssMasterId = 0x7E5;
constexpr uint8_t kLssFastscan  = 0x51;

class SocketLink : public SlaveLink
{
  public:
    SocketLink(std::string interface, unsigned bitrate)
        : interface_(std::move(interface)), bitrate_(bitrate)
    {
    }
    ~SocketLink() override
    {
        close();
    }

    bool open(std::string &problem) override
    {
        close();
        if (prepare_link(system_link_ops(), interface_, bitrate_, problem, setup_) !=
            LinkResult::Ready)
            return false;
        fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
        if (fd_ < 0)
        {
            problem = std::string("cannot open a CAN socket: ") + std::strerror(errno);
            return false;
        }
        sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = static_cast<int>(if_nametoindex(interface_.c_str()));
        if (addr.can_ifindex == 0 ||
            bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
        {
            problem = "cannot bind to " + interface_ + ": " + std::strerror(errno);
            close();
            return false;
        }
        return true;
    }

    void close() override
    {
        if (fd_ >= 0)
            ::close(fd_);
        fd_ = -1;
    }

    int read(Frame &frame, int timeout_ms) override
    {
        pollfd p{fd_, POLLIN, 0};
        int rc = poll(&p, 1, timeout_ms);
        if (rc == 0)
            return 0;
        if (rc < 0)
            return errno == EINTR ? 0 : -1;
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
            return -1;
        can_frame f{};
        ssize_t n = ::read(fd_, &f, sizeof(f));
        if (n < 0)
            return errno == EAGAIN || errno == EINTR ? 0 : -1;
        if (n != static_cast<ssize_t>(sizeof(f)) || (f.can_id & (CAN_EFF_FLAG | CAN_ERR_FLAG)))
            return 0;
        frame.id  = (f.can_id & CAN_SFF_MASK) | ((f.can_id & CAN_RTR_FLAG) ? kRtr : 0);
        frame.dlc = f.can_dlc > 8 ? 8 : f.can_dlc;
        std::memcpy(frame.data, f.data, 8);
        return 1;
    }

    void set_bitrate(unsigned bitrate) override
    {
        bitrate_ = bitrate;
    }

    int write(const Frame &frame) override
    {
        can_frame f{};
        f.can_id  = (frame.id & CAN_SFF_MASK) | ((frame.id & kRtr) ? CAN_RTR_FLAG : 0);
        f.can_dlc = frame.dlc;
        std::memcpy(f.data, frame.data, 8);
        return ::write(fd_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f)) ? 0 : -1;
    }

    bool info(LinkInfo &out) override
    {
        return system_link_ops().get(interface_, out) == 0;
    }

    int take_down() override
    {
        return system_link_ops().set_up(interface_, false);
    }

    bool restart_refused() const override
    {
        return setup_.restart_refused;
    }

  private:
    std::string interface_;
    unsigned bitrate_;
    int fd_ = -1;
    LinkSetup setup_; // restart_refused is kept across reopens
};

class VirtualLink : public SlaveLink
{
  public:
    explicit VirtualLink(std::shared_ptr<lely::io::VirtualCanController> bus) : bus_(bus) {}

    bool open(std::string &) override
    {
        chan_.reset(new lely::io::VirtualCanChannel(ctx_, loop_.get_executor()));
        chan_->open(*bus_);
        return true;
    }
    void close() override
    {
        chan_.reset();
    }
    int read(Frame &frame, int timeout_ms) override
    {
        can_msg msg = CAN_MSG_INIT;
        std::error_code ec;
        if (chan_->read(&msg, nullptr, nullptr, timeout_ms, ec) != 1)
            return 0;
        if (msg.flags & CAN_FLAG_IDE)
            return 0;
        frame.id  = (msg.id & 0x7FF) | ((msg.flags & CAN_FLAG_RTR) ? kRtr : 0);
        frame.dlc = msg.len > 8 ? 8 : msg.len;
        std::memcpy(frame.data, msg.data, 8);
        return 1;
    }
    int write(const Frame &frame) override
    {
        can_msg msg = CAN_MSG_INIT;
        msg.id      = frame.id & 0x7FF;
        if (frame.id & kRtr)
            msg.flags |= CAN_FLAG_RTR;
        msg.len = frame.dlc;
        std::memcpy(msg.data, frame.data, 8);
        std::error_code ec;
        chan_->write(msg, 100, ec);
        return ec ? -1 : 0;
    }

  private:
    std::shared_ptr<lely::io::VirtualCanController> bus_;
    lely::io::Context ctx_;
    lely::ev::Loop loop_;
    std::unique_ptr<lely::io::VirtualCanChannel> chan_;
};

} // namespace

std::unique_ptr<SlaveLink> socket_slave_link(const std::string &interface, unsigned bitrate)
{
    return std::unique_ptr<SlaveLink>(new SocketLink(interface, bitrate));
}

std::unique_ptr<SlaveLink> virtual_slave_link(std::shared_ptr<lely::io::VirtualCanController> bus)
{
    return std::unique_ptr<SlaveLink>(new VirtualLink(bus));
}

// 0x1003 without the "error reset" (0000h) messages that CANopenNode keeps in the same FIFO:
// the number of errors when `nth` is 0, else the nth newest error. False when there is none.
static bool history_entry(const CO_EM_t *em, uint8_t nth, uint32_t &value)
{
    uint8_t found = 0;
    for (uint8_t k = 1; k <= em->fifoCount; ++k)
    {
        int index = static_cast<int>(em->fifoWrPtr) - k;
        if (index < 0)
            index += em->fifoSize;
        const uint32_t msg = em->fifo[index].msg;
        if ((msg & 0xFFFFu) == 0)
            continue;
        if (++found == nth)
        {
            value = msg;
            return true;
        }
    }
    value = found;
    return nth == 0;
}

static ODR_t read_history(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *count_read)
{
    if (stream == nullptr || buf == nullptr || count_read == nullptr || count < 1 ||
        (stream->subIndex > 0 && count < 4))
        return ODR_DEV_INCOMPAT;
    const CO_EM_t *em = static_cast<const CO_EM_t *>(stream->object);
    uint32_t value    = 0;
    if (!history_entry(em, stream->subIndex, value))
        return ODR_NO_DATA;
    if (stream->subIndex == 0)
    {
        CO_setUint8(buf, static_cast<uint8_t>(value));
        *count_read = 1;
    }
    else
    {
        CO_setUint32(buf, value);
        *count_read = 4;
    }
    return ODR_OK;
}

constexpr unsigned kPdos = 4; // RPDOs and TPDOs each

struct Slave::Impl
{
    SlaveOd od;
    SlaveCanPort port{};
    SlaveLink *link         = nullptr;
    unsigned node_id        = 0;
    unsigned master_node_id = 0; // 0: not watched
    uint32_t emcy_id        = 0;
    uint64_t rx             = 0;
    uint64_t tx             = 0;
    uint16_t last_emcy      = 0;
    bool emcy_sent          = false;
    unsigned write_failures = 0;
    OD_extension_t history{};

    // What the frames show, for the status (slave thread only).
    uint32_t pdo_cob[2 * kPdos]{}; // RPDOs then TPDOs, 11-bit; refreshed with the status
    uint64_t pdo_frames[2 * kPdos]{};
    int64_t pdo_last[2 * kPdos]{};
    int64_t master_last = 0;
    bool nmt_command    = false; // an NMT command for this node since the last state check
    struct SdoDownload
    {
        bool active      = false;
        bool expedited   = false;
        bool last        = false; // the last segment was sent
        uint16_t index   = 0;
        uint8_t subindex = 0;
    } sdo;
    std::deque<SlaveEvent> events; // newest first

    void push(SlaveEvent e)
    {
        e.time = wall_ms();
        events.push_front(e);
        if (events.size() > kSlaveEvents)
            events.pop_back();
    }

    void count_pdo(uint32_t id, unsigned first)
    {
        for (unsigned i = first; i < first + kPdos; ++i)
            if (pdo_cob[i] && pdo_cob[i] == id)
            {
                ++pdo_frames[i];
                pdo_last[i] = wall_ms();
                return;
            }
    }

    // A frame received from the bus.
    void seen_rx(const SlaveLink::Frame &f)
    {
        const uint32_t id = f.id & 0x7FF;
        if (f.id & SlaveLink::kRtr)
            return;
        if (id == 0 && f.dlc >= 2 && (f.data[1] == 0 || f.data[1] == node_id))
        {
            nmt_command = true;
            if (f.data[0] == CO_NMT_RESET_NODE || f.data[0] == CO_NMT_RESET_COMMUNICATION)
            {
                SlaveEvent e;
                e.kind = SlaveEventKind::Reset;
                e.node = f.data[0] == CO_NMT_RESET_NODE;
                push(e);
            }
            return;
        }
        if (master_node_id && id == 0x700u + master_node_id)
        {
            master_last = wall_ms();
            return;
        }
        if (id == 0x600u + node_id && f.dlc == 8)
        {
            const uint8_t ccs = f.data[0] >> 5;
            if (ccs == 1) // initiate download
            {
                sdo.active    = true;
                sdo.expedited = (f.data[0] & 0x02) != 0;
                sdo.last      = false;
                sdo.index     = static_cast<uint16_t>(f.data[1] | (f.data[2] << 8));
                sdo.subindex  = f.data[3];
            }
            else if (ccs == 0 && sdo.active) // download segment
                sdo.last = (f.data[0] & 0x01) != 0;
            else if (ccs == 4) // abort by the master
                sdo.active = false;
            return;
        }
        count_pdo(id, 0);
    }

    // A frame the slave sent.
    void seen_tx(const SlaveLink::Frame &f)
    {
        const uint32_t id = f.id & 0x7FF;
        if (id == emcy_id && f.dlc >= 3)
        {
            const uint16_t code = static_cast<uint16_t>(f.data[0] | (f.data[1] << 8));
            last_emcy           = code;
            emcy_sent           = true;
            SlaveEvent e;
            e.kind = code ? SlaveEventKind::Emcy : SlaveEventKind::EmcyReset;
            e.code = code;
            e.reg  = f.data[2];
            push(e);
            return;
        }
        if (id == 0x580u + node_id && f.dlc == 8 && sdo.active)
        {
            const uint16_t index = static_cast<uint16_t>(f.data[1] | (f.data[2] << 8));
            SlaveEvent e;
            e.kind     = SlaveEventKind::SdoWrite;
            e.index    = sdo.index;
            e.subindex = sdo.subindex;
            if (f.data[0] == 0x80) // abort
            {
                if (index != sdo.index || f.data[3] != sdo.subindex)
                    return;
                e.abort_code = static_cast<uint32_t>(f.data[4]) | (f.data[5] << 8) |
                               (f.data[6] << 16) | (static_cast<uint32_t>(f.data[7]) << 24);
                sdo.active = false;
                push(e);
            }
            else if ((f.data[0] >> 5) == 3 && sdo.expedited) // initiate download response
            {
                e.ok       = true;
                sdo.active = false;
                push(e);
            }
            else if ((f.data[0] >> 5) == 1 && sdo.last) // last segment acknowledged
            {
                e.ok       = true;
                sdo.active = false;
                push(e);
            }
            return;
        }
        count_pdo(id, kPdos);
    }

    static int send(void *ctx, uint32_t ident, uint8_t dlc, const uint8_t *data)
    {
        Impl *self = static_cast<Impl *>(ctx);
        SlaveLink::Frame f;
        f.id  = (ident & 0x7FF) | ((ident & CO_CAN_RTR_FLAG) ? SlaveLink::kRtr : 0);
        f.dlc = dlc > 8 ? 8 : dlc;
        std::memcpy(f.data, data, f.dlc);
        if (self->link->write(f) != 0)
        {
            ++self->write_failures;
            return -1;
        }
        ++self->tx;
        self->seen_tx(f);
        return 0;
    }

    void fill_details(CO_t *co, SlaveSnapshot &s);
};

namespace
{

template <typename T> T od_value(OD_t *od, uint16_t index, uint8_t sub)
{
    T v           = 0;
    OD_entry_t *e = OD_find(od, index);
    if (e)
        (void)OD_get_value(e, sub, &v, sizeof(v), true);
    return v;
}

} // namespace

// The details of the status, read in the slave thread; `s` keeps its vectors' memory.
void Slave::Impl::fill_details(CO_t *co, SlaveSnapshot &s)
{
    OD_t *d           = od.od();
    s.detailed        = true;
    s.error_register  = CO_getErrorRegister(co->em);
    s.heartbeat_ms    = od_value<uint16_t>(d, 0x1017, 0);
    s.master_watch_ms = static_cast<uint16_t>(od_value<uint32_t>(d, 0x1016, 1) & 0xFFFFu);
    s.on_master_lost  = od.error_behaviour();
    s.master_last     = master_last;

    s.error_history.clear();
    uint32_t count = 0;
    history_entry(co->em, 0, count);
    for (uint8_t k = 1; k <= count; ++k)
    {
        uint32_t v = 0;
        if (history_entry(co->em, k, v))
            s.error_history.push_back(v);
    }
    // Active errors: the set status bits, each with the code of its newest report.
    s.emcy_active.clear();
    for (uint8_t k = 1; k <= co->em->fifoCount; ++k)
    {
        int index = static_cast<int>(co->em->fifoWrPtr) - k;
        if (index < 0)
            index += co->em->fifoSize;
        const uint32_t msg  = co->em->fifo[index].msg;
        const uint16_t code = static_cast<uint16_t>(msg & 0xFFFFu);
        const uint8_t bit   = static_cast<uint8_t>(msg >> 24);
        if (code == 0 || !CO_isError(co->em, bit))
            continue;
        bool known = false;
        for (uint16_t c : s.emcy_active)
            known = known || c == code;
        if (!known)
            s.emcy_active.push_back(code);
    }

    s.pdos.resize(2 * kPdos);
    for (unsigned i = 0; i < 2 * kPdos; ++i)
    {
        const bool tpdo        = i >= kPdos;
        const uint16_t comm    = static_cast<uint16_t>((tpdo ? 0x1800 : 0x1400) + i % kPdos);
        const uint16_t mapping = static_cast<uint16_t>(comm + 0x200);
        SlavePdoStatus &p      = s.pdos[i];
        p.number               = i % kPdos + 1;
        p.tpdo                 = tpdo;
        p.cob_id               = od_value<uint32_t>(d, comm, 1);
        // The bare pre-defined base follows the node ID in use, as in CO_PDO.c.
        const uint32_t base = (tpdo ? 0x180u : 0x200u) + 0x100u * (i % kPdos);
        if (node_id != 0 && (p.cob_id & 0x7FFu) == base)
            p.cob_id += node_id;
        p.transmission  = od_value<uint8_t>(d, comm, 2);
        p.inhibit_100us = tpdo ? od_value<uint16_t>(d, comm, 3) : 0;
        p.event_ms      = od_value<uint16_t>(d, comm, 5);
        const uint8_t n = od_value<uint8_t>(d, mapping, 0);
        p.mapping.clear();
        for (uint8_t k = 1; k <= n && k <= 8; ++k)
            p.mapping.push_back(od_value<uint32_t>(d, mapping, k));
        p.frames   = pdo_frames[i];
        p.last     = pdo_last[i];
        pdo_cob[i] = (p.cob_id & 0x80000000u) ? 0 : (p.cob_id & 0x7FFu);
    }

    s.values.clear();
    for (const SlaveObject &o : od.config().objects)
    {
        if (o.string)
            continue;
        SlaveValue v;
        v.index       = o.index;
        v.subindex    = o.subindex;
        v.bytes       = static_cast<uint8_t>(o.bytes() > 8 ? 8 : o.bytes());
        OD_entry_t *e = OD_find(d, o.index);
        if (e)
            (void)OD_get_value(e, o.subindex, v.data, v.bytes, true);
        s.values.push_back(v);
    }
    s.events = events;
}

Slave::Slave(const DeviceConfig &dev, std::unique_ptr<SlaveLink> link)
    : dev_(dev), link_(std::move(link)), impl_(new Impl)
{
    status_.name           = dev_.name;
    status_.interface      = dev_.interface;
    status_.node_id        = dev_.node_id;
    status_.master_node_id = dev_.master_watch_ms ? dev_.master_node_id : 0;
}

Slave::~Slave()
{
    stop();
}

bool Slave::start(std::string &problem)
{
    store_path_ = lss_store_path(dev_.interface);
    load_lss_values();
    if (!impl_->od.build(dev_, serial_, problem))
        return false;
    link_->set_bitrate(link_bitrate_);
    in_.resize(impl_->od.from_master().size());
    out_.resize(impl_->od.to_master().size());
    // The defaults reach the PLC before the first frame does.
    uint64_t *back = in_.back();
    for (size_t i = 0; i < impl_->od.from_master().size(); ++i)
        back[i] = SlaveOd::raw_value(impl_->od.from_master()[i]);
    in_.publish();
    plc_running_.store(true);
    stop_.store(false);
    thread_ = std::thread([this] { thread_main(); });
    return true;
}

void Slave::stop()
{
    stop_.store(true);
    if (thread_.joinable())
        thread_.join();
}

void Slave::copy_to_plc(const plugin_runtime_args_t &rt)
{
    const uint64_t *v = in_.latest();
    const auto &slots = impl_->od.from_master();
    for (size_t i = 0; i < slots.size(); ++i)
        image_write_input(rt, slots[i].obj->location, v[i]);
}

void Slave::copy_from_plc(const plugin_runtime_args_t &rt)
{
    if (out_.size() == 0)
        return;
    uint64_t *back    = out_.back();
    const auto &slots = impl_->od.to_master();
    if (rt.image_lock)
        rt.image_lock();
    for (size_t i = 0; i < slots.size(); ++i)
        back[i] = image_read_output(rt, slots[i].obj->location);
    if (rt.image_unlock)
        rt.image_unlock();
    out_.publish();
}

void Slave::set_plc_running(bool running)
{
    plc_running_.store(running);
}

bool Slave::forget_lss()
{
    if (!dev_.lss.enabled)
        return false;
    forget_.store(true);
    return true;
}

namespace
{

std::string rate_text(unsigned bitrate)
{
    if (!bitrate)
        return "the interface's own bit rate";
    return std::to_string(bitrate / 1000) + " kbit/s";
}

} // namespace

void Slave::load_lss_values()
{
    const std::string label = dev_.label();
    LssStored stored;
    std::string problem;
    const bool have = lss_load(store_path_, stored, problem);
    if (!problem.empty())
        log_warn("%s: %s; it is ignored", label.c_str(), problem.c_str());
    if (dev_.lss.serial)
    {
        serial_      = dev_.lss.serial;
        serial_auto_ = false;
    }
    else if (!serial_)
    {
        serial_auto_ = true;
        if (have && stored.serial)
            serial_ = stored.serial;
        else
        {
            serial_       = lss_random_serial();
            stored.serial = serial_;
            if (lss_save(store_path_, stored, problem))
                log_info("%s: serial number 0x%08X (automatic)", label.c_str(), serial_);
            else
                log_warn("%s: serial number 0x%08X (automatic) cannot be kept: %s", label.c_str(),
                         serial_, problem.c_str());
        }
    }
    stored_node_ = have ? stored.node_id : 0;
    stored_rate_ = have ? stored.bitrate : 0;

    unsigned node     = dev_.node_id;
    unsigned rate     = dev_.bitrate;
    const char *nfrom = "project";
    const char *rfrom = "project";
    if (dev_.lss.enabled)
    {
        if (dev_.lss.wait_for_master)
            node = 0xFF;
        if (stored_node_)
        {
            node  = stored_node_;
            nfrom = "stored";
        }
        if (stored_rate_)
        {
            rate  = stored_rate_;
            rfrom = "stored";
        }
        if (stored_node_ || stored_rate_)
            log_info("%s: %s%s and %s from stored LSS values (project: node ID %u, %s)",
                     label.c_str(), node == 0xFF ? "no node ID" : "node ID ",
                     node == 0xFF ? "" : std::to_string(node).c_str(), rate_text(rate).c_str(),
                     dev_.node_id, rate_text(dev_.bitrate).c_str());
    }
    else if (stored_node_ || stored_rate_)
        log_info("%s: LSS is off; the stored LSS values are not used", label.c_str());
    pending_node_ = static_cast<uint8_t>(node);
    start_node_   = static_cast<uint8_t>(node);
    start_from_   = nfrom;
    pending_kbit_ = static_cast<uint16_t>(rate / 1000);
    link_bitrate_ = rate;

    std::lock_guard<PiMutex> lock(status_mutex_);
    status_.serial          = serial_;
    status_.lss             = dev_.lss.enabled;
    status_.serial_auto     = serial_auto_;
    status_.node_from       = nfrom;
    status_.rate_from       = rfrom;
    status_.bitrate         = rate;
    status_.project_node_id = dev_.node_id;
    status_.project_bitrate = dev_.bitrate;
    status_.stored_node_id  = stored_node_;
    status_.stored_bitrate  = stored_rate_;
}

bool Slave::store_lss(uint8_t node_id, uint16_t kbit)
{
    const std::string label = dev_.label();
    LssStored stored;
    std::string problem;
    lss_load(store_path_, stored, problem);
    if (serial_auto_)
        stored.serial = serial_;
    stored.node_id = node_id;
    stored.bitrate = static_cast<unsigned>(kbit) * 1000;
    if (!lss_save(store_path_, stored, problem))
    {
        log_error("%s: LSS store failed: %s", label.c_str(), problem.c_str());
        return false;
    }
    stored_node_ = stored.node_id;
    stored_rate_ = stored.bitrate;
    log_info(
        "%s: LSS store: %s%s, %s saved", label.c_str(), node_id == 0xFF ? "no node ID" : "node ID ",
        node_id == 0xFF ? "" : std::to_string(node_id).c_str(), rate_text(stored.bitrate).c_str());
    std::lock_guard<PiMutex> lock(status_mutex_);
    status_.stored_node_id = stored_node_;
    status_.stored_bitrate = stored_rate_;
    if (!status_.unconfigured && status_.node_id == stored_node_)
        status_.node_from = "stored";
    return true;
}

SlaveSnapshot Slave::snapshot() const
{
    std::lock_guard<PiMutex> lock(status_mutex_);
    return status_;
}

namespace
{

uint16_t nmt_control(const DeviceConfig &dev, uint8_t behaviour)
{
    uint16_t control = dev.start_alone ? CO_NMT_STARTUP_TO_OPERATIONAL : 0;
    if (behaviour == kErrorPreOperational)
        control |= CO_NMT_ERR_ON_BUSOFF_HB;
    else if (behaviour == kErrorStopped)
        control |= CO_NMT_ERR_ON_BUSOFF_HB | CO_NMT_ERR_TO_STOPPED;
    return control;
}

void enter_realtime(int priority, const std::string &label)
{
    sched_param sp{};
    sp.sched_priority = priority;
    int rc            = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0)
    {
        log_warn("%s: cannot use real-time priority %d (%s); it runs at normal priority",
                 label.c_str(), priority, std::strerror(rc));
        return;
    }
    prctl(PR_SET_TIMERSLACK, 1UL, 0, 0, 0);
}

const char *state_text(uint8_t nmt)
{
    switch (nmt)
    {
    case CO_NMT_OPERATIONAL:
        return "operational";
    case CO_NMT_PRE_OPERATIONAL:
        return "pre-operational";
    case CO_NMT_STOPPED:
        return "stopped";
    default:
        return "initializing";
    }
}

} // namespace

void Slave::thread_main()
{
    const std::string label = dev_.label();
    enter_realtime(dev_.priority, label);
    Impl &im          = *impl_;
    im.link           = link_.get();
    im.emcy_id        = 0x80u + dev_.node_id;
    im.port.ctx       = &im;
    im.port.send      = Impl::send;
    im.node_id        = dev_.node_id;
    im.master_node_id = status_.master_node_id;
    std::string last_problem;
    bool first_plc = true;
    bool plc_was   = true;
    BusMonitor monitor(dev_.interface);
    BusOffRecovery recovery;
    bool reset_forbidden = false;
    uint32_t link_resets = 0;
    BusStatus bus        = monitor.simulated();
    BusState bus_was     = BusState::Unknown;
    Clock::time_point last_monitor;

    auto set_problem = [&](const std::string &problem)
    {
        std::lock_guard<PiMutex> lock(status_mutex_);
        status_.bus_open = problem.empty();
        status_.problem  = problem;
    };
    auto sleep_or_stop = [&](int ms)
    {
        for (int waited = 0; waited < ms && !stop_.load(); waited += 50)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };

    // Reads the link into `bus`, tells the stack its error state, and resets a bus-off link the
    // kernel does not restart (as a master network does). True when the link was taken down.
    auto supervise = [&](CO_t *co) -> bool
    {
        LinkInfo li;
        const auto now = BusMonitor::clock::now();
        if (!link_->info(li))
        {
            bus = monitor.simulated();
            return false;
        }
        if (!li.up)
        {
            bus = monitor.unusable(BusState::Down);
            return false;
        }
        bus            = monitor.update(li, now);
        bus.has_resets = li.kind == "can";
        bus.resets     = link_resets;
        uint16_t err   = 0;
        if (bus.state == BusState::BusOff)
            err |= CO_CAN_ERRTX_BUS_OFF;
        if (li.has_berr)
        {
            if (li.tx_errors >= 128)
                err |= CO_CAN_ERRTX_PASSIVE;
            else if (li.tx_errors >= 96)
                err |= CO_CAN_ERRTX_WARNING;
            if (li.rx_errors >= 128)
                err |= CO_CAN_ERRRX_PASSIVE;
            else if (li.rx_errors >= 96)
                err |= CO_CAN_ERRRX_WARNING;
        }
        const uint16_t mask = CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERR_WARN_PASSIVE;
        co->CANmodule->CANerrorStatus =
            static_cast<uint16_t>((co->CANmodule->CANerrorStatus & ~mask) | err);
        if (li.kind != "can" || reset_forbidden)
            return false;
        const LinkFault fault = bus.state == BusState::BusOff ? LinkFault::BusOff : LinkFault::None;
        if (!recovery.reading(fault, li.restart_ms, link_->restart_refused(), now))
        {
            if (recovery.held_back_news())
            {
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              recovery.next_reset() - now)
                              .count();
                log_warn("%s: CAN interface %s is bus-off again; resetting the link in %lld s",
                         label.c_str(), dev_.interface.c_str(),
                         static_cast<long long>((ms + 999) / 1000));
            }
            return false;
        }
        recovery.reset_done(now);
        SlaveEvent e;
        e.kind       = SlaveEventKind::BusReset;
        const int rc = link_->take_down();
        if (rc == 0)
        {
            log_error("%s: CAN interface %s is bus-off; resetting the link", label.c_str(),
                      dev_.interface.c_str());
            ++link_resets;
            e.outcome = ResetOutcome::Done;
            im.push(e);
            return true;
        }
        if (rc == -EPERM)
        {
            reset_forbidden = true;
            log_error("%s: CAN interface %s is bus-off and the runtime is not permitted to reset "
                      "it; reset it by hand with \"ip link set %s down\" and then \"ip link set "
                      "%s up\"",
                      label.c_str(), dev_.interface.c_str(), dev_.interface.c_str(),
                      dev_.interface.c_str());
            e.outcome = ResetOutcome::NotPermitted;
        }
        else
        {
            log_error("%s: CAN interface %s is bus-off and resetting the link failed: %s",
                      label.c_str(), dev_.interface.c_str(), std::strerror(-rc));
            e.outcome = ResetOutcome::Failed;
        }
        im.push(e);
        return false;
    };
    // Brings the link up again after a reset, keeping the stack and its state. False on stop.
    auto reopen = [&](const std::string &) -> bool
    {
        link_->close();
        std::string problem;
        while (!stop_.load() && !link_->open(problem))
        {
            set_problem(problem);
            sleep_or_stop(kRetryMs);
        }
        if (stop_.load())
            return false;
        set_problem("");
        log_info("%s: CAN interface %s is back after the reset", label.c_str(),
                 dev_.interface.c_str());
        return true;
    };

    while (!stop_.load())
    {
        std::string problem;
        if (!link_->open(problem))
        {
            if (problem != last_problem)
                log_warn("%s: %s; retrying", label.c_str(), problem.c_str());
            last_problem = problem;
            set_problem(problem);
            sleep_or_stop(kRetryMs);
            continue;
        }
        last_problem.clear();
        set_problem("");

        uint32_t heap = 0;
        CO_t *co      = CO_new(im.od.co_config(), &heap);
        if (!co)
        {
            log_error("%s: cannot allocate the CANopen stack", label.c_str());
            link_->close();
            sleep_or_stop(kRetryMs);
            continue;
        }
        CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
        bool link_ok             = true;
        bool new_bitrate         = false; // a reset node brought another bit rate
        bool said_seen           = false; // survives a reset communication
        const unsigned opened_at = link_bitrate_;
        while (!stop_.load() && link_ok && reset != CO_RESET_QUIT)
        {
            co->CANmodule->CANnormal = false;
            CO_CANsetConfigurationMode(&im.port);
            CO_CANmodule_disable(co->CANmodule);
            uint32_t err_info    = 0;
            CO_ReturnError_t err = CO_CANinit(co, &im.port, 0);
            uint8_t node         = pending_node_;
            if (err == CO_ERROR_NO && dev_.lss.enabled)
            {
                CO_LSS_address_t address{};
                address.identity.vendorID       = dev_.vendor_id;
                address.identity.productCode    = dev_.product_code;
                address.identity.revisionNumber = dev_.revision;
                address.identity.serialNumber   = serial_;
                err = CO_LSSinit(co, &address, &pending_node_, &pending_kbit_);
                if (err == CO_ERROR_NO)
                {
                    CO_LSSslave_initCkBitRateCall(
                        co->LSSslave, this,
                        [](void *object, uint16_t kbit) -> bool_t
                        {
                            const Slave *self = static_cast<const Slave *>(object);
                            return self->dev_.lss.bitrate_change && kbit != 0;
                        });
                    CO_LSSslave_initCfgStoreCall(
                        co->LSSslave, this, [](void *object, uint8_t id, uint16_t kbit) -> bool_t
                        { return static_cast<Slave *>(object)->store_lss(id, kbit); });
                    node = co->LSSslave->activeNodeID;
                }
            }
            if (err == CO_ERROR_NO)
                err = CO_CANopenInit(co, nullptr, nullptr, im.od.od(), nullptr,
                                     nmt_control(dev_, im.od.error_behaviour()), 500, 1000, 0,
                                     false, node, &err_info);
            const bool unconfigured = err == CO_ERROR_NODE_ID_UNCONFIGURED_LSS;
            if (unconfigured)
                err = CO_ERROR_NO;
            if (err == CO_ERROR_NO && !unconfigured)
                err = CO_CANopenInitPDO(co, co->em, im.od.od(), node, &err_info);
            if (err != CO_ERROR_NO)
            {
                log_error("%s: the CANopen stack refused its configuration (error %d, object "
                          "0x%X)",
                          label.c_str(), static_cast<int>(err), static_cast<unsigned>(err_info));
                reset = CO_RESET_QUIT;
                stop_.store(true);
                break;
            }
            im.emcy_id = 0x80u + node;
            im.node_id = unconfigured ? 0 : node;
            if (!unconfigured)
            {
                im.history      = co->em->OD_1003_extension;
                im.history.read = read_history;
                OD_extension_init(OD_find(im.od.od(), 0x1003), &im.history);
            }
            CO_CANsetNormalMode(co->CANmodule);
            if (!unconfigured && !plc_running_.load())
                CO_errorReport(co->em, kPlcStopBit, CO_EMC_SOFTWARE_USER, 0);
            if (unconfigured)
                log_info("%s: no node ID; waiting for an LSS master (serial number 0x%08X)",
                         label.c_str(), serial_);
            else
                log_info("%s: node %u on the bus", label.c_str(), node);
            {
                std::lock_guard<PiMutex> lock(status_mutex_);
                status_.node_id      = unconfigured ? 0 : node;
                status_.unconfigured = unconfigured;
                // Stored wins; the start value keeps its source; any other ID came from a master.
                if (!unconfigured)
                    status_.node_from = stored_node_ && node == stored_node_ ? "stored"
                                        : node == start_node_               ? start_from_
                                                                            : "lss";
            }
            {
                SlaveEvent e;
                e.kind = SlaveEventKind::OnBus;
                im.push(e);
            }
            im.nmt_command      = false;
            im.sdo.active       = false;
            uint8_t pending_was = pending_node_;

            reset                  = CO_RESET_NOT;
            Clock::time_point last = Clock::now();
            Clock::time_point last_status;
            uint8_t last_state = 0xFF;
            uint8_t status_nmt = 0xFF;
            bool master_was    = false;
            uint8_t behaviour  = im.od.error_behaviour();
            while (!stop_.load() && reset == CO_RESET_NOT)
            {
                SlaveLink::Frame f;
                int got = link_->read(f, kTickMs);
                for (int n = 0; got == 1 && n < 64; ++n)
                {
                    CO_CANrxMsg_t msg;
                    msg.ident = f.id & 0x7FF;
                    if (f.id & SlaveLink::kRtr)
                        msg.ident |= CO_CAN_RTR_FLAG;
                    msg.dlc = f.dlc;
                    std::memcpy(msg.data, f.data, 8);
                    ++im.rx;
                    im.seen_rx(f);
                    // CiA 305: only a slave in the waiting state takes part in fastscan.
                    const bool fastscan_ignored =
                        dev_.lss.enabled && msg.ident == kLssMasterId && msg.dlc >= 1 &&
                        msg.data[0] == kLssFastscan &&
                        CO_LSSslave_getState(co->LSSslave) != CO_LSS_STATE_WAITING;
                    if (!fastscan_ignored)
                        co_slave_receive(co->CANmodule, &msg);
                    got = link_->read(f, 0);
                }
                if (got < 0)
                {
                    log_warn("%s: the CAN interface failed; reopening it", label.c_str());
                    link_ok = false;
                    break;
                }

                // PLC start and stop.
                const bool plc = plc_running_.load();
                if (plc != plc_was)
                {
                    if (!plc)
                    {
                        if (!unconfigured)
                            CO_errorReport(co->em, kPlcStopBit, CO_EMC_SOFTWARE_USER, 0);
                        if (dev_.on_plc_stop == SlaveOnStop::Zero)
                            for (auto &slot : im.od.to_master())
                                im.od.set_to_master(slot, 0);
                    }
                    else
                    {
                        if (!first_plc)
                            im.od.restore_defaults(SlaveOd::Restore::Application);
                        if (!unconfigured)
                            CO_errorReset(co->em, kPlcStopBit, 0);
                    }
                    plc_was = plc;
                    SlaveEvent e;
                    e.kind = SlaveEventKind::Plc;
                    e.ok   = plc;
                    im.push(e);
                }
                first_plc = false;

                // Values to the master from the newest scan.
                bool fresh          = false;
                const uint64_t *out = out_.size() ? out_.latest(&fresh) : nullptr;
                if (plc && fresh && out)
                {
                    auto &slots = im.od.to_master();
                    for (size_t i = 0; i < slots.size(); ++i)
                        im.od.set_to_master(slots[i], out[i]);
                }

                const Clock::time_point now = Clock::now();
                uint32_t dt                 = static_cast<uint32_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(now - last).count());
                last      = now;
                reset     = CO_process(co, false, dt, nullptr);
                bool sync = CO_process_SYNC(co, dt, nullptr);
                CO_process_RPDO(co, sync, dt, nullptr);
                CO_process_TPDO(co, sync, dt, nullptr);

                // LSS: a node ID set by a master, and a forget asked by the editor.
                if (pending_node_ != pending_was)
                {
                    if (unconfigured)
                        log_info("%s: LSS master gave node ID %u", label.c_str(), pending_node_);
                    else if (pending_node_ == 0xFF)
                        log_info("%s: LSS master cleared the node ID (no node ID after reset "
                                 "communication)",
                                 label.c_str());
                    else
                        log_info("%s: LSS master set node ID %u (active after reset "
                                 "communication)",
                                 label.c_str(), pending_node_);
                    pending_was = pending_node_;
                }
                if (forget_.exchange(false))
                {
                    LssStored stored;
                    std::string problem;
                    lss_load(store_path_, stored, problem);
                    stored.node_id = 0;
                    stored.bitrate = 0;
                    if (serial_auto_)
                        stored.serial = serial_;
                    if (lss_save(store_path_, stored, problem))
                        log_info("%s: stored LSS values forgotten; restarting with the project's",
                                 label.c_str());
                    else
                        log_error("%s: cannot forget the stored LSS values: %s", label.c_str(),
                                  problem.c_str());
                    reset = CO_RESET_APP;
                }

                // A master's write to 0x1029.
                if (im.od.error_behaviour() != behaviour)
                {
                    behaviour           = im.od.error_behaviour();
                    co->NMT->NMTcontrol = nmt_control(dev_, behaviour);
                }

                // Values from the master to the scan.
                if (im.od.take_written())
                {
                    uint64_t *back = in_.back();
                    auto &slots    = im.od.from_master();
                    for (size_t i = 0; i < slots.size(); ++i)
                        back[i] = SlaveOd::raw_value(slots[i]);
                    in_.publish();
                }

                const uint8_t state = static_cast<uint8_t>(CO_NMT_getInternalState(co->NMT));
                if (state != last_state)
                {
                    log_info("%s: %s", label.c_str(), state_text(state));
                    SlaveNmtBy by = SlaveNmtBy::Boot;
                    if (im.nmt_command)
                        by = SlaveNmtBy::Master;
                    else if (last_state == CO_NMT_OPERATIONAL)
                        by = SlaveNmtBy::Self;
                    else if (state == CO_NMT_OPERATIONAL && dev_.start_alone)
                        by = SlaveNmtBy::StartAlone;
                    if (state != CO_NMT_INITIALIZING)
                    {
                        SlaveEvent e;
                        e.kind  = SlaveEventKind::Nmt;
                        e.state = state;
                        e.by    = by;
                        im.push(e);
                    }
                    std::lock_guard<PiMutex> lock(status_mutex_);
                    status_.since  = wall_ms();
                    status_.nmt_by = by;
                    last_state     = state;
                }
                im.nmt_command = false;
                bool master    = false;
                if (status_.master_node_id && !unconfigured)
                    master = CO_HBconsumer_getState(co->HBcons, 0) == CO_HBconsumer_ACTIVE;
                if (status_.master_node_id && !unconfigured && master != master_was)
                {
                    if (master && !said_seen)
                    {
                        log_info("%s: heartbeat of master node %u seen", label.c_str(),
                                 dev_.master_node_id);
                        SlaveEvent e;
                        e.kind = SlaveEventKind::Heartbeat;
                        e.ok   = true;
                        im.push(e);
                    }
                    else if (!master && CO_isError(co->em, CO_EM_HEARTBEAT_CONSUMER))
                    {
                        log_warn("%s: heartbeat of master node %u lost", label.c_str(),
                                 dev_.master_node_id);
                        said_seen = false;
                        SlaveEvent e;
                        e.kind = SlaveEventKind::Heartbeat;
                        im.push(e);
                    }
                    if (master)
                        said_seen = true;
                    master_was = master;
                }
                // Bus status and bus-off recovery, once a second.
                if (now - last_monitor >= std::chrono::seconds(1))
                {
                    last_monitor = now;
                    if (supervise(co))
                    {
                        if (!reopen(label))
                            break;
                        im.sdo.active = false;
                    }
                    if (bus.state != bus_was)
                    {
                        if (bus_was != BusState::Unknown || bus.state != BusState::ErrorActive)
                        {
                            SlaveEvent e;
                            e.kind      = SlaveEventKind::BusState;
                            e.bus_state = bus.state;
                            im.push(e);
                        }
                        bus_was = bus.state;
                    }
                }
                if (now - last_status >= std::chrono::milliseconds(100) || state != status_nmt)
                {
                    status_nmt  = state;
                    last_status = now;
                    std::lock_guard<PiMutex> lock(status_mutex_);
                    status_.nmt          = state;
                    status_.plc_running  = plc;
                    status_.master_seen  = master;
                    status_.last_emcy    = im.last_emcy;
                    status_.emcy_sent    = im.emcy_sent;
                    status_.rx_frames    = im.rx;
                    status_.tx_frames    = im.tx;
                    status_.bus          = bus;
                    status_.lss_selected = dev_.lss.enabled && CO_LSSslave_getState(co->LSSslave) ==
                                                                   CO_LSS_STATE_CONFIGURATION;
                    im.fill_details(co, status_);
                }
            }
            if (reset == CO_RESET_APP)
            {
                log_info("%s: reset node", label.c_str());
                im.od.restore_defaults(SlaveOd::Restore::All);
                reset = CO_RESET_NOT;
                load_lss_values();
                if (link_bitrate_ != opened_at)
                {
                    link_->set_bitrate(link_bitrate_);
                    new_bitrate = true;
                    link_ok     = false;
                }
            }
            else if (reset == CO_RESET_COMM)
            {
                log_info("%s: reset communication", label.c_str());
                im.od.restore_defaults(SlaveOd::Restore::Communication);
                reset = CO_RESET_NOT;
            }
        }
        CO_CANsetConfigurationMode(&im.port);
        CO_delete(co);
        link_->close();
        set_problem(stop_.load() || new_bitrate ? "" : "the CAN interface failed");
        {
            std::lock_guard<PiMutex> lock(status_mutex_);
            status_.bus_open = false;
        }
        if (!stop_.load() && !new_bitrate)
            sleep_or_stop(kRetryMs);
    }
    log_info("%s: left the bus", label.c_str());
}

} // namespace canopen_plugin
