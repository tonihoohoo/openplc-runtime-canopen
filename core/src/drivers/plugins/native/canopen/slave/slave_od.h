// SPDX-License-Identifier: Apache-2.0
//
// slave_od.h - a CANopen slave's object dictionary, built at PLC start from its configuration
// in CANopenNode's form (301/CO_ODinterface.h): the communication objects from the settings and
// the application objects from the data rows. Everything is allocated in build(); the stack and
// the slave thread then only read and write the values in place.

#ifndef CANOPEN_SLAVE_OD_H
#define CANOPEN_SLAVE_OD_H

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "301/CO_ODinterface.h"
#include "CANopen.h"
#include "config.h"

namespace canopen_plugin
{

// 0x1029 sub 1 values (CiA 301): what the slave does on a communication error.
enum : uint8_t
{
    kErrorPreOperational = 0,
    kErrorNoChange       = 1,
    kErrorStopped        = 2
};

class SlaveOd
{
  public:
    // One application object's value in the dictionary.
    struct Slot
    {
        const SlaveObject *obj = nullptr;
        uint8_t *data          = nullptr; // obj->bytes() long
        OD_entry_t *entry      = nullptr;
    };

    SlaveOd()                           = default;
    SlaveOd(const SlaveOd &)            = delete;
    SlaveOd &operator=(const SlaveOd &) = delete;

    // Builds the dictionary of `dev` (kept by reference). False with a reason on failure.
    bool build(const DeviceConfig &dev, std::string &problem);
    // Restores the values to their configured defaults: all of them (a reset node), or only the
    // application objects (a PLC start), or only the communication objects (a reset
    // communication).
    enum class Restore
    {
        All,
        Application,
        Communication
    };
    void restore_defaults(Restore what);

    OD_t *od()
    {
        return &od_;
    }
    // CANopenNode's object counts and entries for CO_new().
    CO_config_t *co_config()
    {
        return &config_;
    }
    // The application objects from the master (%I) and to the master (%Q), in object order.
    std::vector<Slot> &from_master()
    {
        return from_master_;
    }
    std::vector<Slot> &to_master()
    {
        return to_master_;
    }
    // 0x1029 sub 1, which a master may change.
    uint8_t error_behaviour() const
    {
        return error_behaviour_;
    }
    // Set by an SDO or RPDO write to an object from the master; cleared by take_written().
    bool take_written();
    // Sets a value to the master and asks for its event-driven TPDOs when it changed.
    void set_to_master(Slot &slot, uint64_t raw);
    // A value from the master as raw little-endian bits.
    static uint64_t raw_value(const Slot &slot);

    // Lookup for the tests.
    OD_entry_t *find(uint16_t index);

  private:
    struct Record
    {
        std::vector<OD_obj_record_t> subs;
    };
    // The application objects of one index, for the range checks of SDO writes.
    struct AppIndex
    {
        SlaveOd *self = nullptr;
        std::array<const SlaveObject *, 256> objects{};
        OD_extension_t ext{};
    };

    uint8_t *store(size_t bytes);
    uint8_t *store_u8(uint8_t v);
    uint8_t *store_u16(uint16_t v);
    uint8_t *store_u32(uint32_t v);
    void add_var(uint16_t index, void *data, OD_attr_t attr, OD_size_t len);
    // Sub-index 0 is added with the highest sub-index given (`max_sub`) unless `count` is set.
    Record &add_record(uint16_t index);
    void add_sub(Record &rec, uint8_t sub, void *data, OD_attr_t attr, OD_size_t len);
    void finish();
    void add_pdos(bool tpdo);

    static ODR_t app_write(OD_stream_t *stream, const void *buf, OD_size_t count,
                           OD_size_t *countWritten);
    static ODR_t error_write(OD_stream_t *stream, const void *buf, OD_size_t count,
                             OD_size_t *countWritten);

    const DeviceConfig *dev_ = nullptr;
    std::deque<std::vector<uint8_t>> storage_;
    // Default bytes for restore_defaults(): target, copy.
    std::vector<std::pair<uint8_t *, std::vector<uint8_t>>> defaults_;
    struct Pending
    {
        uint16_t index;
        uint8_t type;
        void *object;
        uint8_t count;
        size_t record; // position in records_ for ODT_REC
    };
    std::vector<Pending> pending_;
    std::deque<OD_obj_var_t> vars_;
    std::deque<Record> records_;
    std::deque<AppIndex> app_;
    std::vector<OD_entry_t> list_;
    OD_t od_{};
    CO_config_t config_{};
    OD_extension_t error_ext_{};
    uint8_t *error_data_     = nullptr;
    uint8_t error_behaviour_ = kErrorPreOperational;
    std::vector<Slot> from_master_;
    std::vector<Slot> to_master_;
    bool written_         = false;
    size_t comm_defaults_ = 0; // defaults_ before this one are communication objects
};

} // namespace canopen_plugin

#endif // CANOPEN_SLAVE_OD_H
