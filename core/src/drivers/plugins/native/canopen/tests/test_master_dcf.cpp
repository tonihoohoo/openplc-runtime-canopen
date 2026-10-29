// SPDX-License-Identifier: Apache-2.0
//
// The master's layout and DCF (checked by loading it into Lely), the concise DCF of the
// configuration SDOs, and the PDO bit-field helpers.

#include <string>
#include <vector>

#include <lely/co/dcf.h>
#include <lely/co/dev.h>

#include "check.h"
#include "config.h"
#include "master_dcf.h"
#include "network.h"

using namespace canopen_plugin;

namespace
{

const char *kConfig = R"({
  "version": 3,
  "networks": [{
    "name": "CANopen 1", "interface": "can0", "bitrate": 250000, "master_node_id": 1,
    "heartbeat_producer_ms": 200,
    "sync": {"mode": "period", "period_us": 10000},
    "nodes": [
      {"node_id": 2, "identity": {"vendor_id": 1, "product_code": 2},
       "supervision": {"type": "heartbeat", "consumer_ms": 500},
       "sdo": [{"index": 8448, "subindex": 0, "data": "78563412"},
               {"index": 4119, "subindex": 0, "data": "6400"}],
       "tpdos": [{"number": 1, "cob_id": 386, "entries": [
         {"index": 24576, "subindex": 1, "type": "UNSIGNED8", "location": "%IB0"},
         {"index": 25601, "subindex": 1, "type": "INTEGER16", "location": "%IW0"}]}],
       "rpdos": [{"number": 1, "cob_id": 514, "entries": [
         {"index": 25088, "subindex": 1, "type": "UNSIGNED8", "location": "%QB0"}]}]},
      {"node_id": 5, "configure": false, "emcy": {"enabled": false},
       "supervision": {"type": "guarding", "guard_time_ms": 100, "life_time_factor": 3},
       "tpdos": [{"number": 2, "cob_id": 645, "receive_timeout_ms": 250, "entries": [
         {"index": 24832, "subindex": 1, "type": "BOOLEAN", "location": "%IX4.0"},
         {"index": 24832, "subindex": 2, "type": "REAL32", "location": "%ID2"}]}],
       "rpdos": [{"number": 1, "cob_id": 517, "entries": [
         {"index": 25088, "subindex": 1, "type": "UNSIGNED32", "location": "%QD1"}]}]}
    ]
  }]
})";

uint32_t u32(const co_dev_t *dev, uint16_t idx, uint8_t sub)
{
    return co_dev_get_val_u32(dev, idx, sub);
}

void test_layout_and_dcf()
{
    Config cfg;
    std::vector<std::string> errors;
    CHECK(parse_config(kConfig, cfg, errors));
    const NetworkConfig &net = cfg.networks[0];
    MasterLayout l           = master_layout(net);
    CHECK(l.rpdos.size() == 2 && l.tpdos.size() == 2);
    CHECK(l.input_entries == 4 && l.output_entries == 2);
    CHECK(l.rpdos[1].master_number == 2 && l.rpdos[1].node_id == 5 && l.rpdos[1].first_entry == 2);
    CHECK(l.tpdos[1].master_number == 2 && l.tpdos[1].first_entry == 1);

    std::string text = master_dcf(net, l);
    co_dev_t *dev    = co_dev_create_from_dcf_text(text.data(), text.data() + text.size(), nullptr);
    CHECK(dev != nullptr);
    if (!dev)
    {
        std::fprintf(stderr, "%s\n", text.c_str());
        return;
    }
    CHECK(co_dev_get_id(dev) == 1);
    CHECK(u32(dev, 0x1F80, 0) == 0x1);
    // Never mandatory (bit 3); node 5 is not booted (bit 2) and is guarded (100 ms, factor 3).
    CHECK(u32(dev, 0x1F81, 2) == 0x05);
    CHECK(u32(dev, 0x1F81, 5) == (100u << 16 | 3u << 8 | 0x01));
    CHECK(u32(dev, 0x1F81, 3) == 0);
    CHECK(u32(dev, 0x1F85, 2) == 1 && u32(dev, 0x1F86, 2) == 2);
    CHECK(u32(dev, 0x1005, 0) == 0x40000080);
    CHECK(u32(dev, 0x1006, 0) == 10000);
    CHECK(co_dev_get_val_u16(dev, 0x1017, 0) == 200);
    CHECK(co_dev_get_val_u8(dev, 0x1016, 0) == 1);
    CHECK(u32(dev, 0x1016, 1) == (2u << 16 | 500));
    // Emergency consumer; node 5's emergency is off.
    CHECK(u32(dev, 0x1028, 2) == 0x82 && u32(dev, 0x1028, 5) == 0x80000085);
    // Node TPDOs are the master's RPDOs; node RPDOs its TPDOs.
    CHECK(u32(dev, 0x1400, 1) == 0x182 && u32(dev, 0x1401, 1) == 0x285);
    // A receive timeout is the master RPDO's event timer; without one there is no sub 5.
    CHECK(co_dev_get_val_u8(dev, 0x1400, 0) == 2 && co_dev_get_val_u8(dev, 0x1401, 0) == 5);
    CHECK(co_dev_get_val_u16(dev, 0x1401, 5) == 250);
    CHECK(co_dev_get_val_u8(dev, 0x1600, 0) == 2);
    CHECK(u32(dev, 0x1600, 1) == 0x20000008 && u32(dev, 0x1600, 2) == 0x20010010);
    CHECK(u32(dev, 0x1601, 1) == 0x20020001 && u32(dev, 0x1601, 2) == 0x20030020);
    CHECK(u32(dev, 0x1800, 1) == 0x202 && u32(dev, 0x1801, 1) == 0x205);
    CHECK(co_dev_get_val_u8(dev, 0x1800, 2) == 0xFF);
    CHECK(u32(dev, 0x1A00, 1) == 0x30000008 && u32(dev, 0x1A01, 1) == 0x30010020);
    CHECK(co_dev_find_obj(dev, 0x2003) != nullptr && co_dev_find_obj(dev, 0x3001) != nullptr);
    CHECK(co_dev_find_obj(dev, 0x2004) == nullptr);
    co_dev_destroy(dev);
}

void test_concise_dcf()
{
    std::vector<SdoWrite> sdo(2);
    sdo[0].index              = 0x2100;
    sdo[0].subindex           = 0;
    sdo[0].data               = {0x78, 0x56, 0x34, 0x12};
    sdo[1].index              = 0x1017;
    sdo[1].subindex           = 0;
    sdo[1].data               = {0x64, 0x00};
    std::vector<uint8_t> want = {2,    0,    0,    0,    0x00, 0x21, 0x00, 4, 0, 0, 0,    0x78,
                                 0x56, 0x34, 0x12, 0x17, 0x10, 0x00, 2,    0, 0, 0, 0x64, 0x00};
    CHECK(concise_dcf(sdo) == want);
    CHECK((concise_dcf({}) == std::vector<uint8_t>{0, 0, 0, 0}));
}

void test_bits()
{
    uint8_t data[8] = {};
    pdo_set_bits(data, 0, 1, 1);
    pdo_set_bits(data, 1, 8, 0xA5);
    pdo_set_bits(data, 9, 16, 0xFB2E);
    CHECK(pdo_get_bits(data, 8, 0, 1) == 1);
    CHECK(pdo_get_bits(data, 8, 1, 8) == 0xA5);
    CHECK(pdo_get_bits(data, 8, 9, 16) == 0xFB2E);
    CHECK(data[0] == (0x01 | ((0xA5 << 1) & 0xFF)));
    pdo_set_bits(data, 1, 8, 0);
    CHECK(pdo_get_bits(data, 8, 1, 8) == 0 && pdo_get_bits(data, 8, 0, 1) == 1);
    uint8_t little[2] = {0x34, 0x12};
    CHECK(pdo_get_bits(little, 2, 0, 16) == 0x1234);
    // Bits past the received length read as 0.
    CHECK(pdo_get_bits(little, 1, 0, 16) == 0x34);
    uint8_t full[8];
    pdo_set_bits(full, 0, 64, 0x0123456789ABCDEFull);
    CHECK(pdo_get_bits(full, 8, 0, 64) == 0x0123456789ABCDEFull);
}

} // namespace

int main()
{
    test_layout_and_dcf();
    test_concise_dcf();
    test_bits();
    return TEST_RESULT();
}
