// SPDX-License-Identifier: Apache-2.0

#include "slave.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <deque>
#include <functional>
#include <mutex>

#include <pthread.h>
#include <sys/prctl.h>

#include <lely/can/msg.h>
#include <lely/can/net.hpp>
#include <lely/co/dcf.hpp>
#include <lely/co/dev.hpp>
#include <lely/co/emcy.hpp>
#include <lely/co/lss.hpp>
#include <lely/co/nmt.hpp>
#include <lely/co/obj.hpp>
#include <lely/co/pdo.h>
#include <lely/co/rpdo.hpp>
#include <lely/co/sdo.h>
#include <lely/co/tpdo.hpp>
#include <lely/co/type.hpp>
#include <lely/co/val.hpp>

#include "bus_monitor.h"
#include "bus_off_recovery.h"
#include "can_link.h"
#include "log.h"
#include "lss_store.h"
#include "plc_port.h"
#include "slave_dcf.h"

namespace canopen_plugin
{

namespace
{

using Clock = std::chrono::steady_clock;

// Error codes (CiA 301 7.2.7) the slave sends itself.
constexpr uint16_t kEmcyNoError     = 0x0000;
constexpr uint16_t kEmcyPlcStop     = 0x6200; // software, user
constexpr uint16_t kEmcyHeartbeat   = 0x8130;
constexpr uint16_t kEmcyPassive     = 0x8120;
constexpr uint16_t kEmcyBusOff      = 0x8140;
constexpr uint16_t kEmcyRpdoTimeout = 0x8250;
constexpr uint16_t kEmcyRpdoShort   = 0x8210; // the stack's: fewer bytes than mapped
constexpr uint16_t kEmcyRpdoLong    = 0x8220; // the stack's: more bytes than mapped
// Error register bits (CiA 301 7.5.2.2).
constexpr uint8_t kRegCurrent       = 0x02;
constexpr uint8_t kRegVoltage       = 0x04;
constexpr uint8_t kRegTemperature   = 0x08;
constexpr uint8_t kRegCommunication = 0x10;
constexpr uint8_t kRegManufacturer  = 0x80;
// 0x1029 values (CiA 301): what the slave does on a communication error.
constexpr uint8_t kErrorPreOperational = 0;
constexpr uint8_t kErrorNoChange       = 1;
constexpr uint8_t kErrorStopped        = 2;
constexpr size_t kHistory              = 8; // 0x1003 entries
constexpr int kRetryMs                 = 1000;
constexpr int kTickMs                  = 1;
constexpr uint32_t kLssMasterId        = 0x7E5;
constexpr uint32_t kLssSlaveId         = 0x7E4;
constexpr uint8_t kLssSwitchGlobal     = 0x04;
constexpr uint8_t kLssSwitchSelected   = 0x44;
constexpr uint8_t kLssFastscan         = 0x51;
constexpr uint8_t kLssFastscanAnswer   = 0x4F;

timespec monotonic_now()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

uint8_t nmt_state(co_nmt_t *nmt)
{
    return static_cast<uint8_t>(co_nmt_get_st(nmt) & ~CO_NMT_ST_TOGGLE);
}

} // namespace

struct Slave::Impl
{
    Slave *self = nullptr;

    // ---- built in start(), kept for the slave's life ----
    std::string dcf_base; // the DCF with node ID 255; re-made per start of the stack
    // One application object in the dictionary; `sub` belongs to the stack running now.
    struct Slot
    {
        const SlaveObject *obj = nullptr;
        co_sub_t *sub          = nullptr;
    };
    std::vector<Slot> from_master;
    std::vector<Slot> to_master;
    struct AppSub
    {
        Impl *impl                   = nullptr;
        const SlaveObject *obj       = nullptr;
        std::atomic<uint32_t> *count = nullptr;
    };
    std::deque<AppSub> app_subs;
    std::deque<std::atomic<uint32_t>> counts;

    // ---- the stack running now (slave thread only) ----
    co_dev_t *dev    = nullptr;
    can_net_t *net   = nullptr;
    co_nmt_t *nmt    = nullptr;
    void *app_dcf    = nullptr; // the application values at start, for a PLC start
    bool booting     = false;   // the stack's first reset node: the LSS values are loaded
    bool restart     = false;   // a reset node brought another bit rate: start the stack anew
    bool reset_comm  = false;   // a reset communication ran; the slave is on the bus anew
    bool after_node  = false;   // that reset communication followed a reset node
    bool written     = false;   // a master wrote a value from it, or the defaults came back
    bool mute_emcy   = false;   // drops an EMCY being replaced by another one
    uint8_t st_known = 0xFF;    // the NMT state the indications were installed for

    std::function<void()> at_reset_comm; // the slave's status for the node ID now in use
    SlaveLink *link         = nullptr;
    unsigned node_id        = 0;
    unsigned master_node_id = 0; // 0: not watched
    uint32_t emcy_id        = 0;
    uint64_t rx             = 0;
    uint64_t tx             = 0;
    uint16_t last_emcy      = 0;
    bool emcy_sent          = false;
    unsigned write_failures = 0;
    std::deque<uint32_t> history; // 0x1003, newest first

    // What the frames show, for the status (slave thread only).
    uint32_t pdo_cob[2 * kSlavePdos]{}; // RPDOs then TPDOs, 11-bit; refreshed with the status
    uint64_t pdo_frames[2 * kSlavePdos]{};
    int64_t pdo_last[2 * kSlavePdos]{};
    int64_t master_last = 0;
    bool master_beat    = false; // a heartbeat of the master came since the last loss or reset
    bool master_lost    = false; // the stack reported its loss
    bool master_missing = false; // the loss is not resolved yet (8130h belongs on the stack)
    bool nmt_command    = false; // an NMT command for this node since the last state check
    bool lss_selected   = false; // in the LSS configuration state
    struct Fastscan
    {
        uint8_t bitchk  = 0;
        uint8_t lsssub  = 0;
        uint8_t lssnext = 0;
    } fastscan;
    struct SdoDownload
    {
        bool active      = false;
        bool expedited   = false;
        bool last        = false; // the last segment was sent
        uint16_t index   = 0;
        uint8_t subindex = 0;
    } sdo;
    std::deque<SlaveEvent> events; // newest first

    // The program's emergencies as applied to the stack, by slot.
    struct ProgramEmcy
    {
        uint32_t gen  = 0;
        bool on       = false;
        uint16_t code = 0;
        uint32_t info = 0;
        uint8_t klass = 0;
    };
    ProgramEmcy program[CANOPEN_PLC_SLAVE_EMCY_SLOTS];
    bool plc_stop_on  = false; // EMCY 6200h wanted
    bool passive_on   = false; // EMCY 8120h wanted
    bool bus_off_on   = false; // EMCY 8140h wanted
    bool program_nmt  = false; // the program asked for the last NMT change
    bool plc_stop_nmt = false; // the PLC stop setting (0x1029 sub 2) made it

    ~Impl()
    {
        destroy();
    }

    void destroy()
    {
        if (nmt)
            co_nmt_destroy(nmt);
        nmt = nullptr;
        if (net)
            can_net_destroy(net);
        net = nullptr;
        if (app_dcf)
            co_val_fini(CO_DEFTYPE_DOMAIN, &app_dcf);
        app_dcf = nullptr;
        if (dev)
            co_dev_destroy(dev);
        dev = nullptr;
        for (Slot &s : from_master)
            s.sub = nullptr;
        for (Slot &s : to_master)
            s.sub = nullptr;
    }

    const std::atomic<uint32_t> *write_count(uint16_t index, uint8_t subindex) const
    {
        for (const AppSub &a : app_subs)
            if (a.obj->index == index && a.obj->subindex == subindex)
                return a.count;
        return nullptr;
    }

    uint8_t od_u8(uint16_t index, uint8_t sub) const
    {
        return dev ? co_dev_get_val_u8(dev, index, sub) : 0;
    }
    uint16_t od_u16(uint16_t index, uint8_t sub) const
    {
        return dev ? co_dev_get_val_u16(dev, index, sub) : 0;
    }
    uint32_t od_u32(uint16_t index, uint8_t sub) const
    {
        return dev ? co_dev_get_val_u32(dev, index, sub) : 0;
    }
    uint8_t error_behaviour() const
    {
        return od_u8(0x1029, 1);
    }
    uint8_t plc_stop_state() const
    {
        return od_u8(0x1029, 2);
    }

    static uint64_t raw_value(const Slot &slot)
    {
        uint64_t v = 0;
        if (slot.sub)
            std::memcpy(&v, co_sub_addressof_val(slot.sub),
                        std::min<size_t>(slot.obj->bytes(), sizeof(v)));
        else
            v = slot.obj->value;
        return v;
    }

    // Sets a value to the master and asks for its event-driven TPDOs when it changed.
    void set_to_master(Slot &slot, uint64_t raw)
    {
        if (!slot.sub)
            return;
        const size_t len = slot.obj->bytes();
        if (std::memcmp(co_sub_addressof_val(slot.sub), &raw, len) == 0)
            return;
        co_sub_set_val(slot.sub, &raw, len);
        if (slot.obj->exch == SlaveExchange::Pdo)
            co_dev_tpdo_event(dev, slot.obj->index, slot.obj->subindex);
    }

    void push(SlaveEvent e)
    {
        e.time = wall_ms();
        events.push_front(e);
        if (events.size() > kSlaveEvents)
            events.pop_back();
    }

    void count_pdo(uint32_t id, unsigned first)
    {
        for (unsigned i = first; i < first + kSlavePdos; ++i)
            if (pdo_cob[i] && pdo_cob[i] == id)
            {
                ++pdo_frames[i];
                pdo_last[i] = wall_ms();
                return;
            }
    }

    bool program_code_active(uint16_t code) const
    {
        for (const ProgramEmcy &p : program)
            if (p.on && p.code == code)
                return true;
        return false;
    }

    // ---- EMCY: the stack's producer, with the error register bits we give ----
    co_emcy_t *emcy() const
    {
        return nmt ? co_nmt_get_emcy(nmt) : nullptr;
    }
    bool emcy_active(uint16_t code) const
    {
        co_emcy_t *e = emcy();
        return e && co_emcy_find(e, code) >= 0;
    }
    // Sends `code` with `info` in EMCY bytes 4-7; byte 3 stays 0.
    void emcy_raise(uint16_t code, uint8_t reg, uint32_t info = 0)
    {
        co_emcy_t *e = emcy();
        if (!e)
            return;
        co_unsigned8_t msef[5] = {
            0, static_cast<co_unsigned8_t>(info), static_cast<co_unsigned8_t>(info >> 8),
            static_cast<co_unsigned8_t>(info >> 16), static_cast<co_unsigned8_t>(info >> 24)};
        co_emcy_push(e, code, reg, msef);
    }
    // Clears one active `code`; `quiet` sends no 0000h (another code takes its place).
    void emcy_clear(uint16_t code, bool quiet = false)
    {
        co_emcy_t *e = emcy();
        if (!e)
            return;
        const ssize_t n = co_emcy_find(e, code);
        if (n < 0)
            return;
        mute_emcy = quiet;
        co_emcy_remove(e, static_cast<size_t>(n));
        mute_emcy = false;
    }
    // Raises what the slave wants active and the stack lost (a reset or the stopped state).
    void emcy_sync()
    {
        if (!emcy() || node_id == 0)
            return;
        if (plc_stop_on && !emcy_active(kEmcyPlcStop))
            emcy_raise(kEmcyPlcStop, kRegManufacturer);
        for (uint32_t i = 0; i < CANOPEN_PLC_SLAVE_EMCY_SLOTS; ++i)
        {
            const ProgramEmcy &p = program[i];
            if (p.on && !emcy_active(p.code))
                emcy_raise(p.code, p.klass, p.info);
        }
        if (passive_on && !emcy_active(kEmcyPassive))
            emcy_raise(kEmcyPassive, 0);
        if (bus_off_on && !emcy_active(kEmcyBusOff))
            emcy_raise(kEmcyBusOff, kRegCommunication);
        if (master_missing && !emcy_active(kEmcyHeartbeat))
            emcy_raise(kEmcyHeartbeat, kRegCommunication);
    }
    // A new EMCY service starts with an empty stack while 0x1001 keeps the old bits: with nothing
    // active, 0x1001 is cleared and 0000h is sent.
    void emcy_settle()
    {
        co_emcy_t *e = emcy();
        if (!e || node_id == 0 || od_u8(0x1001, 0) == 0)
            return;
        co_unsigned16_t eec = 0;
        co_emcy_peek(e, &eec, nullptr);
        if (eec != 0)
            return;
        mute_emcy = true;
        co_emcy_push(e, 0xFF00, 0, nullptr);
        mute_emcy = false;
        co_emcy_clear(e);
        sync_history();
    }

    void set_plc_stop(bool on)
    {
        plc_stop_on = on;
        if (on)
            emcy_sync();
        else
            emcy_clear(kEmcyPlcStop);
    }

    // Raises, changes or clears the program's emergencies the blocks asked for since the last
    // tick. A change while active sends one EMCY with the new code, without a reset between.
    void apply_program_emcy(EmcySlot *slots)
    {
        for (uint32_t i = 0; i < CANOPEN_PLC_SLAVE_EMCY_SLOTS; ++i)
        {
            const uint32_t gen = slots[i].gen.load(std::memory_order_acquire);
            if (gen == program[i].gen)
                continue;
            program[i].gen      = gen;
            const bool active   = slots[i].active.load(std::memory_order_relaxed);
            const uint16_t code = slots[i].code.load(std::memory_order_relaxed);
            const uint32_t info = slots[i].info.load(std::memory_order_relaxed);
            ProgramEmcy &p      = program[i];
            if (active)
            {
                if (p.on && p.code == code && p.info == info)
                    continue;
                if (p.on)
                    emcy_clear(p.code, true);
                p.on    = true;
                p.code  = code;
                p.info  = info;
                p.klass = Slave::error_class_bit(code);
                emcy_raise(code, p.klass, info);
            }
            else if (p.on)
            {
                p.on = false;
                emcy_clear(p.code);
            }
        }
    }

    // At a PLC stop: every program emergency is cleared (the PLC stop's 6200h takes over).
    void clear_program_emcy(EmcySlot *slots)
    {
        for (uint32_t i = 0; i < CANOPEN_PLC_SLAVE_EMCY_SLOTS; ++i)
        {
            slots[i].active.store(false, std::memory_order_relaxed);
            program[i].gen = slots[i].gen.fetch_add(1, std::memory_order_acq_rel) + 1;
            if (program[i].on)
            {
                program[i].on = false;
                emcy_clear(program[i].code);
            }
        }
    }

    // The stack keeps its active errors in 0x1003 and answers a read past its sub 0 itself: the
    // object is given the history after every change the stack may have made.
    void sync_history()
    {
        if (!dev || !co_dev_find_obj(dev, 0x1003))
            return;
        co_dev_set_val_u8(dev, 0x1003, 0, static_cast<co_unsigned8_t>(history.size()));
        for (uint8_t s = 1; s <= kHistory; ++s)
            co_dev_set_val_u32(dev, 0x1003, s, s <= history.size() ? history[s - 1] : 0);
    }

    // ---- frames seen, for the status ----
    // A frame received from the bus.
    void seen_rx(const SlaveLink::Frame &f)
    {
        const uint32_t id = f.id & 0x7FF;
        if (f.id & SlaveLink::kRtr)
            return;
        if (id == 0 && f.dlc >= 2 && (f.data[1] == 0 || f.data[1] == node_id))
        {
            nmt_command = true;
            if (f.data[0] == CO_NMT_CS_RESET_NODE || f.data[0] == CO_NMT_CS_RESET_COMM)
            {
                SlaveEvent e;
                e.kind = SlaveEventKind::Reset;
                e.node = f.data[0] == CO_NMT_CS_RESET_NODE;
                push(e);
            }
            return;
        }
        if (master_node_id && id == 0x700u + master_node_id)
        {
            master_last = wall_ms();
            if (node_id != 0)
                master_beat = true;
            return;
        }
        if (id == kLssMasterId && f.dlc >= 1)
        {
            if (f.data[0] == kLssFastscan && f.dlc >= 8)
                fastscan = {f.data[5], f.data[6], f.data[7]};
            return;
        }
        if (node_id && id == 0x600u + node_id && f.dlc == 8)
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
        if (node_id && id == emcy_id && f.dlc >= 3)
        {
            const uint16_t code = static_cast<uint16_t>(f.data[0] | (f.data[1] << 8));
            last_emcy           = code;
            emcy_sent           = true;
            if (code != kEmcyNoError)
            {
                history.push_front(code | (static_cast<uint32_t>(f.data[2]) << 16) |
                                   (static_cast<uint32_t>(f.data[3]) << 24));
                if (history.size() > kHistory)
                    history.pop_back();
            }
            SlaveEvent e;
            e.kind    = code ? SlaveEventKind::Emcy : SlaveEventKind::EmcyReset;
            e.code    = code;
            e.reg     = f.data[2];
            e.program = code && program_code_active(code);
            push(e);
            return;
        }
        if (id == kLssSlaveId && f.dlc >= 1)
        {
            if (f.data[0] == kLssSwitchSelected)
                lss_selected = true;
            // The last fastscan step (CiA 305: LSSNext wraps below LSSSub): the slave is selected.
            else if (f.data[0] == kLssFastscanAnswer && fastscan.bitchk == 0 &&
                     fastscan.lssnext < fastscan.lsssub)
                lss_selected = true;
            return;
        }
        if (node_id && id == 0x580u + node_id && f.dlc == 8 && sdo.active)
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
        count_pdo(id, kSlavePdos);
    }

    // ---- the stack's callbacks ----
    static int send(const can_msg *msg, void *data)
    {
        Impl *im = static_cast<Impl *>(data);
        if (im->restart || (msg->flags & CAN_FLAG_IDE))
            return 0;
        SlaveLink::Frame f;
        f.id  = (msg->id & 0x7FF) | ((msg->flags & CAN_FLAG_RTR) ? SlaveLink::kRtr : 0);
        f.dlc = msg->len > 8 ? 8 : msg->len;
        std::memcpy(f.data, msg->data, f.dlc);
        if (im->mute_emcy && (f.id & 0x7FF) == im->emcy_id)
            return 0;
        if (im->link->write(f) != 0)
        {
            ++im->write_failures;
            return -1;
        }
        ++im->tx;
        im->seen_tx(f);
        return 0;
    }

    static void on_cs(co_nmt_t *, co_unsigned8_t cs, void *data)
    {
        Impl *im = static_cast<Impl *>(data);
        if (cs == CO_NMT_CS_RESET_NODE)
            im->self->reset_node();
        else if (cs == CO_NMT_CS_RESET_COMM)
        {
            // Before the boot-up: the node ID applies, the error services start later.
            im->reset_comm = true;
            if (im->at_reset_comm)
                im->at_reset_comm();
        }
    }

    static void on_hb(co_nmt_t *nmt, co_unsigned8_t id, int state, int reason, void *data)
    {
        co_nmt_on_hb(nmt, id, state, reason);
        Impl *im = static_cast<Impl *>(data);
        if (reason != CO_NMT_EC_TIMEOUT)
            return;
        if (state == CO_NMT_EC_OCCURRED)
        {
            im->master_beat    = false;
            im->master_lost    = true;
            im->master_missing = true;
        }
        else if (state == CO_NMT_EC_RESOLVED)
            im->master_missing = false;
    }

    static co_unsigned32_t app_dn(co_sub_t *sub, co_sdo_req *req, void *data)
    {
        AppSub *a          = static_cast<AppSub *>(data);
        const uint16_t typ = co_sub_get_type(sub);
        co_unsigned32_t ac = 0;
        co_val val;
        if (co_sdo_req_dn_val(req, typ, &val, &ac) == -1)
            return ac;
        const SlaveObject *o = a->obj;
        if (!o->string && (o->has_min || o->has_max))
        {
            uint64_t raw = 0;
            std::memcpy(&raw, &val, std::min<size_t>(o->bytes(), sizeof(raw)));
            if (o->has_min && compare_raw(o->type, raw, o->min) < 0)
                ac = CO_SDO_AC_PARAM_LO;
            else if (o->has_max && compare_raw(o->type, raw, o->max) > 0)
                ac = CO_SDO_AC_PARAM_HI;
        }
        if (!ac)
            co_sub_dn(sub, &val);
        co_val_fini(typ, &val);
        if (ac)
            return ac;
        if (o->dir == SlaveDirection::FromMaster)
        {
            a->impl->written = true;
            a->count->fetch_add(1, std::memory_order_relaxed);
        }
        return 0;
    }

    // 0x1029: sub 1 and 2 take 0-2 only.
    static co_unsigned32_t error_dn(co_sub_t *sub, co_sdo_req *req, void *)
    {
        const uint16_t typ = co_sub_get_type(sub);
        co_unsigned32_t ac = 0;
        co_val val;
        if (co_sdo_req_dn_val(req, typ, &val, &ac) == -1)
            return ac;
        if (co_sub_get_subidx(sub) > 0 && val.u8 > kErrorStopped)
            ac = CO_SDO_AC_PARAM_VAL;
        else
            co_sub_dn(sub, &val);
        co_val_fini(typ, &val);
        return ac;
    }

    // 0x1003 as an error history: the EMCY errors sent, newest at sub 1, never 0000h. Writing 0
    // to sub 0 clears it.
    static co_unsigned32_t history_up(const co_sub_t *sub, co_sdo_req *req, void *data)
    {
        Impl *im           = static_cast<Impl *>(data);
        const uint8_t s    = co_sub_get_subidx(sub);
        co_unsigned32_t ac = 0;
        if (s == 0)
        {
            const co_unsigned8_t n = static_cast<co_unsigned8_t>(im->history.size());
            co_sdo_req_up_val(req, CO_DEFTYPE_UNSIGNED8, &n, &ac);
            return ac;
        }
        if (s > im->history.size())
            return CO_SDO_AC_NO_DATA;
        const co_unsigned32_t v = im->history[s - 1];
        co_sdo_req_up_val(req, CO_DEFTYPE_UNSIGNED32, &v, &ac);
        return ac;
    }
    static co_unsigned32_t history_dn(co_sub_t *sub, co_sdo_req *req, void *data)
    {
        Impl *im           = static_cast<Impl *>(data);
        co_unsigned32_t ac = 0;
        co_val val;
        if (co_sdo_req_dn_val(req, co_sub_get_type(sub), &val, &ac) == -1)
            return ac;
        if (co_sub_get_subidx(sub) != 0 || val.u8 != 0)
            return CO_SDO_AC_PARAM_VAL;
        im->history.clear();
        im->sync_history();
        return 0;
    }

    // A frame that fits its mapping ends the RPDO timeout and the stack's length errors.
    static void on_rpdo(co_rpdo_t *pdo, co_unsigned32_t ac, const void *, size_t n, void *data)
    {
        Impl *im = static_cast<Impl *>(data);
        if (ac)
            return;
        const uint16_t map  = static_cast<uint16_t>(0x1600 + co_rpdo_get_num(pdo) - 1);
        const uint8_t count = im->od_u8(map, 0);
        unsigned bits       = 0;
        for (uint8_t k = 1; count <= 0x40 && k <= count; ++k)
            bits += im->od_u32(map, k) & 0xFFu;
        if ((bits + 7) / 8 < n)
            return;
        for (uint16_t code : {kEmcyRpdoTimeout, kEmcyRpdoShort, kEmcyRpdoLong})
            if (im->emcy_active(code))
                im->emcy_clear(code);
    }
    // The stack's RPDO errors, each sent once while it stays active.
    static void on_rpdo_err(co_rpdo_t *, co_unsigned16_t eec, co_unsigned8_t er, void *data)
    {
        Impl *im = static_cast<Impl *>(data);
        if (!im->emcy_active(eec))
            co_nmt_on_err(im->nmt, eec, er, nullptr);
    }

    static void on_lss_rate(co_lss_t *, co_unsigned16_t, int, void *)
    {
        // Activate bit timing is not taken: a stored bit rate applies at the next start.
    }

    static int on_lss_store(co_lss_t *, co_unsigned8_t id, co_unsigned16_t rate, void *data)
    {
        Impl *im = static_cast<Impl *>(data);
        return im->self->store_lss(id, rate) ? 0 : -1;
    }

    // PDO parameters while the stack's PDO services are off (pre-operational, stopped), checked
    // as the services check them: a mapping changes only while the PDO is invalid and its count
    // is 0, maps only mappable objects and fits in 8 bytes; a valid COB-ID is not changed.
    static co_unsigned32_t pdo_comm_dn(co_sub_t *sub, co_sdo_req *req, void *)
    {
        const uint16_t typ = co_sub_get_type(sub);
        co_unsigned32_t ac = 0;
        co_val val;
        if (co_sdo_req_dn_val(req, typ, &val, &ac) == -1)
            return ac;
        if (co_sub_get_subidx(sub) == 1)
        {
            const uint32_t old = co_sub_get_val_u32(sub);
            const bool valid   = !(old & 0x80000000u) && !(val.u32 & 0x80000000u);
            if (valid && (old & 0x1FFFFFFFu) != (val.u32 & 0x1FFFFFFFu))
                ac = CO_SDO_AC_PARAM_VAL;
        }
        if (!ac)
            co_sub_dn(sub, &val);
        co_val_fini(typ, &val);
        return ac;
    }
    static co_unsigned32_t pdo_map_dn(co_sub_t *sub, co_sdo_req *req, void *)
    {
        co_obj_t *obj      = co_sub_get_obj(sub);
        const uint16_t idx = co_obj_get_idx(obj);
        const bool tpdo    = idx >= 0x1A00;
        const co_dev_t *d  = co_obj_get_dev(obj);
        const uint16_t typ = co_sub_get_type(sub);
        co_unsigned32_t ac = 0;
        co_val val;
        if (co_sdo_req_dn_val(req, typ, &val, &ac) == -1)
            return ac;
        const bool valid =
            !(co_dev_get_val_u32(d, static_cast<uint16_t>(idx - 0x200), 1) & 0x80000000u);
        auto check = [&](uint32_t map) -> co_unsigned32_t
        {
            const uint16_t i = static_cast<uint16_t>(map >> 16);
            const uint8_t s  = static_cast<uint8_t>(map >> 8);
            return tpdo ? co_dev_chk_tpdo(d, i, s) : co_dev_chk_rpdo(d, i, s);
        };
        const uint8_t n_old = co_dev_get_val_u8(d, idx, 0);
        if (co_sub_get_subidx(sub) == 0)
        {
            const uint8_t n = val.u8;
            if (n != n_old)
            {
                if (valid || n > 8)
                    ac = CO_SDO_AC_PARAM_VAL;
                unsigned bits = 0;
                for (uint8_t k = 1; !ac && k <= n; ++k)
                {
                    const uint32_t map = co_dev_get_val_u32(d, idx, k);
                    if (!map)
                        continue;
                    if ((bits += map & 0xFFu) > 64)
                        ac = CO_SDO_AC_PDO_LEN;
                    else
                        ac = check(map);
                }
            }
        }
        else if (val.u32 != co_sub_get_val_u32(sub))
        {
            if (valid || n_old)
                ac = CO_SDO_AC_PARAM_VAL;
            else if (val.u32)
                ac = check(val.u32);
        }
        if (!ac)
            co_sub_dn(sub, &val);
        co_val_fini(typ, &val);
        return ac;
    }

    // Checks the stack leaves out, in every NMT state: a transmission type is 0-240, 254 or
    // 255, a mapping has at most 8 entries, and a mapped length fits the object it maps.
    static co_unsigned32_t map_len_ac(const co_dev_t *d, uint32_t map)
    {
        const uint16_t i    = static_cast<uint16_t>(map >> 16);
        const uint8_t s     = static_cast<uint8_t>(map >> 8);
        const unsigned bits = map & 0xFFu;
        if (co_type_is_basic(i) && !s)
            return bits > 8 * co_type_sizeof(i) ? CO_SDO_AC_NO_PDO : 0;
        const co_sub_t *sub = co_dev_find_sub(d, i, s);
        if (!sub)
            return 0; // the stack's own check answers
        const size_t bytes =
            co_val_write(co_sub_get_type(sub), co_sub_get_val(sub), nullptr, nullptr);
        return bits == 0 || bits > 8 * bytes ? CO_SDO_AC_NO_PDO : 0;
    }
    static co_unsigned32_t pdo_extra_ac(co_sub_t *sub, const co_val &val)
    {
        co_obj_t *obj      = co_sub_get_obj(sub);
        const uint16_t idx = co_obj_get_idx(obj);
        const uint8_t s    = co_sub_get_subidx(sub);
        if ((idx & 0x0200u) == 0)
            return s == 2 && val.u8 > 240 && val.u8 < 254 ? CO_SDO_AC_PARAM_VAL : 0;
        const co_dev_t *d = co_obj_get_dev(obj);
        if (s != 0)
            return val.u32 ? map_len_ac(d, val.u32) : 0;
        if (val.u8 > 8)
            return CO_SDO_AC_PARAM_VAL; // no MPDOs; at most 8 entries
        co_unsigned32_t ac = 0;
        for (uint8_t k = 1; !ac && k <= val.u8; ++k)
            if (const uint32_t map = co_dev_get_val_u32(d, idx, k))
                ac = map_len_ac(d, map);
        return ac;
    }
    struct PdoGuard
    {
        co_sub_dn_ind_t *prev = nullptr;
        void *prev_data       = nullptr;
    };
    PdoGuard guards[4 * kSlavePdos]; // 0x1400, 0x1600, 0x1800 and 0x1A00 ranges
    static co_unsigned32_t pdo_guard_dn(co_sub_t *sub, co_sdo_req *req, void *data)
    {
        const PdoGuard *g  = static_cast<const PdoGuard *>(data);
        const uint16_t typ = co_sub_get_type(sub);
        co_unsigned32_t ac = 0;
        co_val val;
        if (co_sdo_req_dn_val(req, typ, &val, &ac) == -1)
            return ac;
        ac = pdo_extra_ac(sub, val);
        if (!ac)
        {
            // The value goes on to the indication below as one expedited request.
            uint_least8_t bytes[8];
            const size_t n = co_val_write(typ, &val, bytes, bytes + sizeof(bytes));
            co_sdo_req r;
            co_sdo_req_init(&r);
            r.size  = n;
            r.buf   = bytes;
            r.nbyte = n;
            ac      = g->prev(sub, &r, g->prev_data);
            co_sdo_req_fini(&r);
        }
        co_val_fini(typ, &val);
        return ac;
    }
    void guard_pdos()
    {
        for (unsigned k = 0; k < 4 * kSlavePdos; ++k)
        {
            const uint16_t idx =
                static_cast<uint16_t>(0x1400 + (k / kSlavePdos) * 0x200 + k % kSlavePdos);
            co_obj_t *obj  = co_dev_find_obj(dev, idx);
            co_sub_t *sub0 = obj ? co_obj_find_sub(obj, 0) : nullptr;
            if (!sub0)
                continue;
            co_sub_dn_ind_t *ind = nullptr;
            void *data           = nullptr;
            co_sub_get_dn_ind(sub0, &ind, &data);
            if (ind == &pdo_guard_dn)
                continue;
            guards[k] = PdoGuard{ind, data};
            co_obj_set_dn_ind(obj, &pdo_guard_dn, &guards[k]);
        }
    }

    void pdo_inds(uint16_t comm)
    {
        if (co_obj_t *c = co_dev_find_obj(dev, comm))
            co_obj_set_dn_ind(c, &pdo_comm_dn, nullptr);
        if (co_obj_t *m = co_dev_find_obj(dev, static_cast<uint16_t>(comm + 0x200)))
            co_obj_set_dn_ind(m, &pdo_map_dn, nullptr);
    }

    // The indications the stack replaces when its services start again.
    void install()
    {
        sync_history();
        const uint8_t st = nmt_state(nmt);
        if (st == st_known)
            return;
        st_known = st;
        if (co_obj_t *h = co_dev_find_obj(dev, 0x1003))
        {
            co_obj_set_up_ind(h, &history_up, this);
            co_obj_set_dn_ind(h, &history_dn, this);
        }
        for (co_unsigned16_t n = 1; n <= kSlavePdos; ++n)
        {
            if (co_rpdo_t *pdo = co_nmt_get_rpdo(nmt, n))
            {
                co_rpdo_set_ind(pdo, &on_rpdo, this);
                co_rpdo_set_err(pdo, &on_rpdo_err, this);
            }
            else
                pdo_inds(static_cast<uint16_t>(0x1400 + n - 1));
            if (!co_nmt_get_tpdo(nmt, n))
                pdo_inds(static_cast<uint16_t>(0x1800 + n - 1));
        }
        guard_pdos();
        if (co_lss_t *lss = co_nmt_get_lss(nmt))
        {
            co_lss_set_store_ind(lss, &on_lss_store, this);
            co_lss_set_rate_ind(lss, self->dev_.lss.bitrate_change ? &on_lss_rate : nullptr, this);
        }
    }

    void fill_details(SlaveSnapshot &s);
};

// The details of the status, read in the slave thread; `s` keeps its vectors' memory.
void Slave::Impl::fill_details(SlaveSnapshot &s)
{
    s.detailed        = true;
    s.error_register  = od_u8(0x1001, 0);
    s.heartbeat_ms    = od_u16(0x1017, 0);
    s.master_watch_ms = static_cast<uint16_t>(od_u32(0x1016, 1) & 0xFFFFu);
    s.on_master_lost  = error_behaviour();
    s.master_last     = master_last;

    s.error_history.assign(history.begin(), history.end());
    // Active errors, newest first.
    s.emcy_active.clear();
    if (co_emcy_t *e = emcy())
    {
        std::vector<std::pair<ssize_t, uint16_t>> found;
        auto add = [&](uint16_t code)
        {
            const ssize_t n = co_emcy_find(e, code);
            if (n < 0)
                return;
            for (const auto &f : found)
                if (f.second == code)
                    return;
            found.emplace_back(n, code);
        };
        add(kEmcyPlcStop);
        for (const ProgramEmcy &p : program)
            if (p.on)
                add(p.code);
        add(kEmcyPassive);
        add(kEmcyBusOff);
        add(kEmcyHeartbeat);
        add(kEmcyRpdoTimeout);
        add(kEmcyRpdoShort);
        add(kEmcyRpdoLong);
        std::sort(found.begin(), found.end());
        for (const auto &f : found)
            s.emcy_active.push_back(f.second);
    }
    s.emcy_program.clear();
    for (const ProgramEmcy &p : program)
        if (p.on &&
            std::find(s.emcy_program.begin(), s.emcy_program.end(), p.code) == s.emcy_program.end())
            s.emcy_program.push_back(p.code);
    s.on_plc_stop_state = plc_stop_state();

    s.pdos.resize(2 * kSlavePdos);
    for (unsigned i = 0; i < 2 * kSlavePdos; ++i)
    {
        const bool tpdo        = i >= kSlavePdos;
        const uint16_t comm    = static_cast<uint16_t>((tpdo ? 0x1800 : 0x1400) + i % kSlavePdos);
        const uint16_t mapping = static_cast<uint16_t>(comm + 0x200);
        SlavePdoStatus &p      = s.pdos[i];
        p.number               = i % kSlavePdos + 1;
        p.tpdo                 = tpdo;
        p.cob_id               = od_u32(comm, 1);
        p.transmission         = od_u8(comm, 2);
        p.inhibit_100us        = tpdo ? od_u16(comm, 3) : 0;
        p.event_ms             = od_u16(comm, 5);
        const uint8_t n        = od_u8(mapping, 0);
        p.mapping.clear();
        for (uint8_t k = 1; k <= n && k <= 8; ++k)
            p.mapping.push_back(od_u32(mapping, k));
        p.frames   = pdo_frames[i];
        p.last     = pdo_last[i];
        pdo_cob[i] = (p.cob_id & 0x80000000u) ? 0 : (p.cob_id & 0x7FFu);
    }

    s.values.clear();
    for (const SlaveObject &o : self->dev_.objects)
    {
        if (o.string)
            continue;
        SlaveValue v;
        v.index    = o.index;
        v.subindex = o.subindex;
        v.bytes    = static_cast<uint8_t>(o.bytes() > 8 ? 8 : o.bytes());
        if (dev)
            if (const co_sub_t *sub = co_dev_find_sub(dev, o.index, o.subindex))
                std::memcpy(v.data, co_sub_get_val(sub), v.bytes);
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
    // The dictionary is loaded once here, so a file the stack refuses fails the start.
    const std::string dcf = slave_dcf(dev_, serial_, pending_node_);
    co_dev_t *check = co_dev_create_from_dcf_text(dcf.data(), dcf.data() + dcf.size(), nullptr);
    if (!check)
    {
        problem = "the CANopen stack refused the slave's object dictionary";
        return false;
    }
    co_dev_destroy(check);
    Impl &im = *impl_;
    im.self  = this;
    for (const SlaveObject &o : dev_.objects)
    {
        im.counts.emplace_back(0u);
        Impl::AppSub a;
        a.impl  = &im;
        a.obj   = &o;
        a.count = &im.counts.back();
        im.app_subs.push_back(a);
        if (o.exch == SlaveExchange::Constant)
            continue;
        Impl::Slot slot;
        slot.obj = &o;
        (o.dir == SlaveDirection::FromMaster ? im.from_master : im.to_master).push_back(slot);
    }
    link_->set_bitrate(link_bitrate_);
    in_.resize(im.from_master.size());
    out_.resize(im.to_master.size());
    // The defaults reach the PLC before the first frame does.
    uint64_t *back = in_.back();
    for (size_t i = 0; i < im.from_master.size(); ++i)
        back[i] = Impl::raw_value(im.from_master[i]);
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
    const auto &slots = impl_->from_master;
    for (size_t i = 0; i < slots.size(); ++i)
    {
        const SlaveObject &o = *slots[i].obj;
        if (!o.packed())
            image_write_input(rt, o.location, v[i]);
        for (size_t k = 0; k < o.bits.size(); ++k)
            image_write_input(rt, o.bits[k], (v[i] >> k) & 1u);
    }
}

void Slave::copy_from_plc(const plugin_runtime_args_t &rt)
{
    if (out_.size() == 0)
        return;
    uint64_t *back    = out_.back();
    const auto &slots = impl_->to_master;
    if (rt.image_lock)
        rt.image_lock();
    for (size_t i = 0; i < slots.size(); ++i)
    {
        const SlaveObject &o = *slots[i].obj;
        if (!o.packed())
        {
            back[i] = image_read_output(rt, o.location);
            continue;
        }
        uint64_t byte = 0;
        for (size_t k = 0; k < o.bits.size(); ++k)
            byte |= (image_read_output(rt, o.bits[k]) & 1u) << k;
        back[i] = byte;
    }
    if (rt.image_unlock)
        rt.image_unlock();
    out_.publish();
}

void Slave::set_plc_running(bool running)
{
    if (running)
    {
        // The scan does not run here: the blocks of the new run claim their slots again.
        emcy_epoch_.fetch_add(1, std::memory_order_acq_rel);
        for (EmcySlot &e : emcy_)
        {
            e.active.store(false, std::memory_order_relaxed);
            e.gen.fetch_add(1, std::memory_order_release);
            e.claimed.store(false, std::memory_order_release);
        }
        nmt_request_.store(0, std::memory_order_release);
    }
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

bool Slave::program_code_allowed(uint16_t code)
{
    return code != kEmcyNoError && code != kEmcyPlcStop && code != kEmcyHeartbeat;
}

uint8_t Slave::error_class_bit(uint16_t code)
{
    if ((code & 0xFF00u) == 0xFF00u)
        return kRegManufacturer;
    switch (code >> 12)
    {
    case 0x2:
        return kRegCurrent;
    case 0x3:
        return kRegVoltage;
    case 0x4:
        return kRegTemperature;
    case 0x8:
        return kRegCommunication;
    default:
        return 0;
    }
}

// A reset node (from a master, the program or a forget): the node ID and bit rate to come back
// with are the stored ones, else the project's. Runs inside the stack, before its reset
// communication.
void Slave::reset_node()
{
    Impl &im = *impl_;
    if (im.booting)
        return;
    log_info("%s: reset node", dev_.label().c_str());
    const unsigned opened_at = link_bitrate_;
    load_lss_values();
    co_nmt_set_id(im.nmt, pending_node_);
    co_dev_set_rate(im.dev, pending_kbit_);
    if (link_bitrate_ != opened_at)
        im.restart = true;
    im.after_node = true;
    im.written    = true; // the scan takes the restored values
}

void Slave::publish_plc(uint8_t nmt, uint8_t node_id, bool on_bus, bool watched, bool seen,
                        BusState bus, uint8_t error_register, uint8_t emcy_active)
{
    uint8_t flags = 0;
    if (on_bus)
        flags |= CANOPEN_PLC_SLAVE_ON_BUS;
    if (watched)
        flags |= CANOPEN_PLC_SLAVE_MASTER_WATCHED;
    if (watched && seen)
        flags |= CANOPEN_PLC_SLAVE_MASTER_SEEN;
    uint8_t bus_state = CANOPEN_PLC_BUS_UNKNOWN;
    switch (bus)
    {
    case BusState::ErrorActive:
    case BusState::ErrorWarning:
        bus_state = CANOPEN_PLC_BUS_ERROR_ACTIVE;
        break;
    case BusState::ErrorPassive:
        bus_state = CANOPEN_PLC_BUS_ERROR_PASSIVE;
        break;
    case BusState::BusOff:
        bus_state = CANOPEN_PLC_BUS_STATE_OFF;
        break;
    default:
        break;
    }
    const uint32_t seq = view_.seq.load(std::memory_order_relaxed);
    view_.seq.store(seq + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    view_.state.store(on_bus ? device_state_of(nmt) : static_cast<uint8_t>(CANOPEN_PLC_NOT_AVAIL),
                      std::memory_order_relaxed);
    view_.node_id.store(node_id, std::memory_order_relaxed);
    view_.flags.store(flags, std::memory_order_relaxed);
    view_.bus_state.store(bus_state, std::memory_order_relaxed);
    view_.error_register.store(error_register, std::memory_order_relaxed);
    view_.emcy_active.store(emcy_active, std::memory_order_relaxed);
    view_.seq.store(seq + 2, std::memory_order_release);
}

void Slave::plc_status(canopen_plc_slave_status &out) const
{
    // A torn read is retried a few times; the slave thread writes for a few nanoseconds only.
    for (int tries = 0; tries < 8; ++tries)
    {
        const uint32_t before = view_.seq.load(std::memory_order_acquire);
        if (before & 1u)
            continue;
        canopen_plc_slave_status s{};
        s.state          = view_.state.load(std::memory_order_relaxed);
        s.node_id        = view_.node_id.load(std::memory_order_relaxed);
        s.flags          = view_.flags.load(std::memory_order_relaxed);
        s.bus_state      = view_.bus_state.load(std::memory_order_relaxed);
        s.error_register = view_.error_register.load(std::memory_order_relaxed);
        s.emcy_active    = view_.emcy_active.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (view_.seq.load(std::memory_order_relaxed) == before)
        {
            out = s;
            return;
        }
    }
}

namespace
{

// The CiA 301 NMT state byte a local command leads to, 0 for a command the program may not use.
uint8_t state_of_command(uint8_t command)
{
    switch (command)
    {
    case CO_NMT_CS_START:
        return CO_NMT_ST_START;
    case CO_NMT_CS_STOP:
        return CO_NMT_ST_STOP;
    case CO_NMT_CS_ENTER_PREOP:
        return CO_NMT_ST_PREOP;
    default:
        return 0;
    }
}

} // namespace

int Slave::plc_nmt(uint8_t command)
{
    if (!state_of_command(command))
        return CANOPEN_PLC_WRONG_PARAMETER;
    nmt_request_.store(command, std::memory_order_release);
    return CANOPEN_PLC_NO_ERROR;
}

int Slave::plc_nmt_done(uint8_t command, bool &done) const
{
    const uint8_t want = state_of_command(command);
    if (!want)
        return CANOPEN_PLC_WRONG_PARAMETER;
    canopen_plc_slave_status st{};
    plc_status(st);
    done = nmt_request_.load(std::memory_order_acquire) == 0 && st.state == device_state_of(want);
    return CANOPEN_PLC_NO_ERROR;
}

int Slave::plc_emcy_claim(uint32_t &handle)
{
    const uint32_t epoch = emcy_epoch_.load(std::memory_order_acquire) & 0xFFFFFFu;
    for (uint32_t i = 0; i < CANOPEN_PLC_SLAVE_EMCY_SLOTS; ++i)
    {
        bool expected = false;
        if (emcy_[i].claimed.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
        {
            handle = (epoch << 8) | (i + 1);
            return CANOPEN_PLC_NO_ERROR;
        }
    }
    return CANOPEN_PLC_REQUEST_ERROR;
}

int Slave::plc_emcy(uint32_t handle, bool active, uint16_t code, uint32_t info)
{
    const uint32_t slot = handle & 0xFFu;
    if (slot < 1 || slot > CANOPEN_PLC_SLAVE_EMCY_SLOTS ||
        (handle >> 8) != (emcy_epoch_.load(std::memory_order_acquire) & 0xFFFFFFu) ||
        !emcy_[slot - 1].claimed.load(std::memory_order_acquire))
        return CANOPEN_PLC_REQUEST_ERROR;
    if (active && !program_code_allowed(code))
        return CANOPEN_PLC_WRONG_PARAMETER;
    EmcySlot &e = emcy_[slot - 1];
    if (e.active.load(std::memory_order_relaxed) == active &&
        (!active || (e.code.load(std::memory_order_relaxed) == code &&
                     e.info.load(std::memory_order_relaxed) == info)))
        return CANOPEN_PLC_NO_ERROR;
    e.code.store(code, std::memory_order_relaxed);
    e.info.store(info, std::memory_order_relaxed);
    e.active.store(active, std::memory_order_relaxed);
    e.gen.fetch_add(1, std::memory_order_release);
    return CANOPEN_PLC_NO_ERROR;
}

int Slave::plc_written(uint16_t index, uint8_t subindex, uint32_t &count) const
{
    const std::atomic<uint32_t> *c = impl_->write_count(index, subindex);
    if (!c)
        return CANOPEN_PLC_WRONG_PARAMETER;
    count = c->load(std::memory_order_relaxed);
    return CANOPEN_PLC_NO_ERROR;
}

namespace
{

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
    case CO_NMT_ST_START:
        return "operational";
    case CO_NMT_ST_PREOP:
        return "pre-operational";
    case CO_NMT_ST_STOP:
        return "stopped";
    default:
        return "initializing";
    }
}

void set_string(co_dev_t *dev, uint16_t index, uint8_t subindex, const std::string &text)
{
    if (co_sub_t *sub = co_dev_find_sub(dev, index, subindex))
        co_sub_set_val(sub, text.c_str(), text.size());
}

} // namespace

void Slave::thread_main()
{
    const std::string label = dev_.label();
    enter_realtime(dev_.priority, label);
    Impl &im          = *impl_;
    im.link           = link_.get();
    im.node_id        = 0;
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
    bool said_seen = false; // survives a reset communication

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

    // Reads the link into `bus`, raises or clears the bus's emergencies, and resets a bus-off
    // link the kernel does not restart (as a master network does). True when the link was taken
    // down.
    auto supervise = [&]() -> bool
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
        bus              = monitor.update(li, now);
        bus.has_resets   = li.kind == "can";
        bus.resets       = link_resets;
        const bool off   = bus.state == BusState::BusOff;
        const bool passv = li.has_berr && (li.tx_errors >= 128 || li.rx_errors >= 128);
        if (im.node_id != 0)
        {
            if (passv && !im.passive_on)
            {
                im.passive_on = true;
                im.emcy_raise(kEmcyPassive, 0);
            }
            else if (!passv && im.passive_on)
            {
                im.passive_on = false;
                im.emcy_clear(kEmcyPassive);
            }
            if (off && !im.bus_off_on)
            {
                im.bus_off_on = true;
                // A communication error: 0x1029 sub 1 applies, as for a lost master.
                co_nmt_on_err(im.nmt, kEmcyBusOff, kRegCommunication, nullptr);
            }
            else if (!off && im.bus_off_on)
            {
                im.bus_off_on = false;
                im.emcy_clear(kEmcyBusOff);
            }
        }
        if (li.kind != "can" || reset_forbidden)
            return false;
        const LinkFault fault = off ? LinkFault::BusOff : LinkFault::None;
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
    auto reopen = [&]() -> bool
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

    // Builds the stack from the configuration and boots it (reset node). False when the stack
    // refused the configuration.
    auto build = [&]() -> bool
    {
        im.destroy();
        const std::string dcf = slave_dcf(dev_, serial_, pending_node_);
        im.dev = co_dev_create_from_dcf_text(dcf.data(), dcf.data() + dcf.size(), nullptr);
        if (!im.dev)
            return false;
        if (!dev_.device_name.empty())
            set_string(im.dev, 0x1008, 0, dev_.device_name);
        if (!dev_.hardware_version.empty())
            set_string(im.dev, 0x1009, 0, dev_.hardware_version);
        if (!dev_.software_version.empty())
            set_string(im.dev, 0x100A, 0, dev_.software_version);
        for (Impl::AppSub &a : im.app_subs)
        {
            co_sub_t *sub = co_dev_find_sub(im.dev, a.obj->index, a.obj->subindex);
            if (!sub)
                return false;
            if (a.obj->string)
                co_sub_set_val(sub, a.obj->text.c_str(), a.obj->text.size());
            else if (co_sub_set_val(sub, &a.obj->value, a.obj->bytes()) != a.obj->bytes())
                return false;
            if (a.obj->exch != SlaveExchange::Constant)
                co_sub_set_dn_ind(sub, &Impl::app_dn, &a);
        }
        for (auto *slots : {&im.from_master, &im.to_master})
            for (Impl::Slot &s : *slots)
                s.sub = co_dev_find_sub(im.dev, s.obj->index, s.obj->subindex);
        if (co_obj_t *o = co_dev_find_obj(im.dev, 0x1029))
            co_obj_set_dn_ind(o, &Impl::error_dn, nullptr);
        co_dev_set_rate(im.dev, pending_kbit_);
        if (co_dev_write_dcf(im.dev, 0x2000, 0x9FFF, &im.app_dcf) == -1)
            return false;
        im.net = can_net_create();
        if (!im.net)
            return false;
        can_net_set_send_func(im.net, &Impl::send, &im);
        const timespec now = monotonic_now();
        can_net_set_time(im.net, &now);
        im.nmt = co_nmt_create(im.net, im.dev);
        if (!im.nmt)
            return false;
        co_nmt_set_cs_ind(im.nmt, &Impl::on_cs, &im);
        co_nmt_set_hb_ind(im.nmt, &Impl::on_hb, &im);
        im.restart    = false;
        im.reset_comm = false;
        im.after_node = true;
        im.st_known   = 0xFF;
        im.booting    = true;
        co_nmt_cs_ind(im.nmt, CO_NMT_CS_RESET_NODE);
        im.booting = false;
        im.install();
        return true;
    };

    // A reset communication: the node ID in use, before the boot-up is sent.
    im.at_reset_comm = [&]()
    {
        const uint8_t node      = co_dev_get_id(im.dev);
        const bool unconfigured = node == 0 || node > 127;
        if (!im.after_node && im.node_id != 0)
            log_info("%s: reset communication", label.c_str());
        im.after_node     = false;
        im.node_id        = unconfigured ? 0 : node;
        im.emcy_id        = im.od_u32(0x1014, 0) & 0x7FFu;
        im.master_beat    = false;
        im.master_lost    = false;
        im.master_missing = false;
        im.history.clear();
        for (uint32_t i = 0; i < CANOPEN_PLC_SLAVE_EMCY_SLOTS; ++i)
        {
            im.program[i]     = Impl::ProgramEmcy{};
            im.program[i].gen = emcy_[i].gen.load(std::memory_order_acquire) - 1;
        }
        im.program_nmt  = false;
        im.plc_stop_nmt = false;
        im.st_known     = 0xFF;
        if (unconfigured)
            log_info("%s: no node ID; waiting for an LSS master (serial number 0x%08X)",
                     label.c_str(), serial_);
        else
            log_info("%s: node %u on the bus", label.c_str(), node);
        {
            std::lock_guard<PiMutex> lock(status_mutex_);
            status_.node_id      = im.node_id;
            status_.unconfigured = unconfigured;
            // Stored wins; the start value keeps its source; any other ID came from a master.
            if (!unconfigured)
                status_.node_from = stored_node_ && node == stored_node_ ? "stored"
                                    : node == start_node_                ? start_from_
                                                                         : "lss";
        }
        SlaveEvent e;
        e.kind = SlaveEventKind::OnBus;
        im.push(e);
        im.nmt_command = false;
        im.sdo.active  = false;
    };
    // Then the slave is on the bus with a stack with no errors, which gets the ones the slave
    // wants active again.
    auto on_bus = [&]()
    {
        im.install();
        im.emcy_sync();
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

        bool link_ok     = true;
        bool new_bitrate = false; // a reset node brought another bit rate
        im.plc_stop_on   = !plc_running_.load();
        im.passive_on    = false;
        im.bus_off_on    = false;
        if (!build())
        {
            log_error("%s: the CANopen stack refused its configuration", label.c_str());
            im.destroy();
            stop_.store(true);
            link_->close();
            break;
        }

        Clock::time_point last_status;
        uint8_t last_state = 0xFF;
        uint8_t status_nmt = 0xFF;
        bool master_was    = false;
        while (!stop_.load())
        {
            SlaveLink::Frame f;
            int got     = link_->read(f, kTickMs);
            timespec ts = monotonic_now();
            can_net_set_time(im.net, &ts);
            im.sync_history();
            for (int n = 0; got == 1 && n < 64 && !im.restart; ++n)
            {
                ++im.rx;
                im.seen_rx(f);
                const uint32_t id = f.id & 0x7FF;
                const bool rtr    = (f.id & SlaveLink::kRtr) != 0;
                const bool lss    = id == kLssMasterId && !rtr && f.dlc >= 1;
                bool drop         = lss && !dev_.lss.enabled;
                // CiA 305: only an unconfigured slave in the waiting state takes part in fastscan.
                if (lss && f.data[0] == kLssFastscan &&
                    (co_dev_get_id(im.dev) != 0xFF || im.lss_selected))
                    drop = true;
                if (!drop)
                {
                    can_msg msg = CAN_MSG_INIT;
                    msg.id      = id;
                    if (rtr)
                        msg.flags |= CAN_FLAG_RTR;
                    msg.len = f.dlc;
                    std::memcpy(msg.data, f.data, 8);
                    can_net_recv(im.net, &msg);
                    if (lss && f.data[0] == kLssSwitchGlobal && f.dlc >= 2)
                    {
                        im.lss_selected = f.data[1] == 1;
                        // An unconfigured slave given a node ID starts with it on the switch to
                        // the waiting state (CiA 305).
                        if (f.data[1] == 0 && co_dev_get_id(im.dev) == 0xFF &&
                            co_nmt_get_id(im.nmt) != 0xFF)
                            co_nmt_cs_ind(im.nmt, CO_NMT_CS_RESET_COMM);
                    }
                }
                im.install();
                if (im.reset_comm)
                {
                    im.reset_comm = false;
                    on_bus();
                }
                got = link_->read(f, 0);
            }
            if (im.restart)
            {
                link_->set_bitrate(link_bitrate_);
                new_bitrate = true;
                break;
            }
            if (got < 0)
            {
                log_warn("%s: the CAN interface failed; reopening it", label.c_str());
                link_ok = false;
                break;
            }
            if (im.reset_comm)
            {
                im.reset_comm = false;
                on_bus();
            }
            const bool unconfigured = im.node_id == 0;

            // PLC start and stop.
            const bool plc = plc_running_.load();
            if (plc != plc_was)
            {
                if (!plc)
                {
                    im.set_plc_stop(true);
                    im.clear_program_emcy(emcy_);
                    if (dev_.on_plc_stop == SlaveOnStop::Zero)
                        for (auto &slot : im.to_master)
                            im.set_to_master(slot, 0);
                    // The NMT state set for a PLC stop (0x1029 sub 2, coded as sub 1).
                    const uint8_t now_state  = nmt_state(im.nmt);
                    const uint8_t stop_state = unconfigured ? kErrorNoChange : im.plc_stop_state();
                    if (stop_state == kErrorPreOperational && now_state == CO_NMT_ST_START)
                    {
                        im.plc_stop_nmt = true;
                        co_nmt_cs_ind(im.nmt, CO_NMT_CS_ENTER_PREOP);
                    }
                    else if (stop_state == kErrorStopped && now_state != CO_NMT_ST_STOP)
                    {
                        im.plc_stop_nmt = true;
                        co_nmt_cs_ind(im.nmt, CO_NMT_CS_STOP);
                    }
                }
                else
                {
                    if (!first_plc)
                    {
                        co_dev_read_dcf(im.dev, nullptr, nullptr, &im.app_dcf);
                        im.written = true;
                    }
                    im.set_plc_stop(false);
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
                for (size_t i = 0; i < im.to_master.size(); ++i)
                    im.set_to_master(im.to_master[i], out[i]);

            // The program NMT request and emergencies; an unconfigured slave stays silent.
            const uint8_t cmd = nmt_request_.exchange(0, std::memory_order_acq_rel);
            if (cmd && !unconfigured)
            {
                im.program_nmt = true;
                co_nmt_cs_ind(im.nmt, cmd);
            }
            if (!unconfigured)
                im.apply_program_emcy(emcy_);

            // LSS: a forget asked by the editor, and a node ID set by a master.
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
                co_nmt_cs_ind(im.nmt, CO_NMT_CS_RESET_NODE);
            }
            im.install();
            if (im.reset_comm)
            {
                im.reset_comm = false;
                on_bus();
            }
            if (im.restart)
            {
                link_->set_bitrate(link_bitrate_);
                new_bitrate = true;
                break;
            }
            const uint8_t pending = co_nmt_get_id(im.nmt);
            if (pending != pending_node_)
            {
                if (unconfigured)
                    log_info("%s: LSS master gave node ID %u", label.c_str(), pending);
                else if (pending == 0xFF)
                    log_info("%s: LSS master cleared the node ID (no node ID after reset "
                             "communication)",
                             label.c_str());
                else
                    log_info("%s: LSS master set node ID %u (active after reset communication)",
                             label.c_str(), pending);
                pending_node_ = pending;
            }

            // Values from the master to the scan.
            if (im.written)
            {
                im.written     = false;
                uint64_t *back = in_.back();
                for (size_t i = 0; i < im.from_master.size(); ++i)
                    back[i] = Impl::raw_value(im.from_master[i]);
                in_.publish();
            }

            const uint8_t state = nmt_state(im.nmt);
            if (state != last_state)
            {
                log_info("%s: %s", label.c_str(), state_text(state));
                SlaveNmtBy by = SlaveNmtBy::Boot;
                if (im.program_nmt)
                    by = SlaveNmtBy::Program;
                else if (im.plc_stop_nmt)
                    by = SlaveNmtBy::PlcStop;
                else if (im.nmt_command)
                    by = SlaveNmtBy::Master;
                else if (last_state == CO_NMT_ST_START)
                    by = SlaveNmtBy::Self;
                else if (state == CO_NMT_ST_START && dev_.start_alone)
                    by = SlaveNmtBy::StartAlone;
                if (state != CO_NMT_ST_BOOTUP)
                {
                    SlaveEvent e;
                    e.kind  = SlaveEventKind::Nmt;
                    e.state = state;
                    e.by    = by;
                    im.push(e);
                }
                // Event-driven TPDOs are sent once on entering operational, as before.
                if (state == CO_NMT_ST_START)
                    for (co_unsigned16_t n = 1; n <= kSlavePdos; ++n)
                    {
                        const uint8_t trans = im.od_u8(static_cast<uint16_t>(0x1800 + n - 1), 2);
                        co_tpdo_t *pdo      = co_nmt_get_tpdo(im.nmt, n);
                        if (pdo && trans >= 0xFE)
                            co_tpdo_event(pdo);
                    }
                // Leaving the stopped state brings a new EMCY service: it gets the active
                // errors back, without sending them again.
                if (last_state == CO_NMT_ST_STOP && !unconfigured)
                {
                    im.mute_emcy = true;
                    im.emcy_sync();
                    im.mute_emcy = false;
                    im.emcy_settle();
                }
                std::lock_guard<PiMutex> lock(status_mutex_);
                status_.since  = wall_ms();
                status_.nmt_by = by;
                last_state     = state;
            }
            im.nmt_command    = false;
            im.program_nmt    = false;
            im.plc_stop_nmt   = false;
            const bool master = status_.master_node_id && !unconfigured && im.master_beat;
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
                else if (!master && im.master_lost)
                {
                    log_warn("%s: heartbeat of master node %u lost", label.c_str(),
                             dev_.master_node_id);
                    said_seen = false;
                    SlaveEvent e;
                    e.kind = SlaveEventKind::Heartbeat;
                    im.push(e);
                }
                if (master)
                {
                    said_seen      = true;
                    im.master_lost = false;
                }
                master_was = master;
            }
            // Bus status and bus-off recovery, once a second.
            const Clock::time_point now = Clock::now();
            if (now - last_monitor >= std::chrono::seconds(1))
            {
                last_monitor = now;
                if (supervise())
                {
                    if (!reopen())
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
                status_.lss_selected = dev_.lss.enabled && im.lss_selected;
                im.fill_details(status_);
                const size_t active = status_.emcy_active.size();
                publish_plc(state, static_cast<uint8_t>(im.node_id), true,
                            status_.master_node_id != 0, master, bus.state, im.od_u8(0x1001, 0),
                            static_cast<uint8_t>(active > 255 ? 255 : active));
            }
        }
        im.destroy();
        link_->close();
        publish_plc(0, static_cast<uint8_t>(im.node_id), false, status_.master_node_id != 0, false,
                    BusState::Unknown, 0, 0);
        set_problem(stop_.load() || new_bitrate ? "" : "the CAN interface failed");
        {
            std::lock_guard<PiMutex> lock(status_mutex_);
            status_.bus_open = false;
        }
        if (!stop_.load() && !new_bitrate && !link_ok)
            sleep_or_stop(kRetryMs);
    }
    im.at_reset_comm = nullptr;
    log_info("%s: left the bus", label.c_str());
}

} // namespace canopen_plugin
