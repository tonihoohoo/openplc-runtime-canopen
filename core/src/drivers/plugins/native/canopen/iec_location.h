// SPDX-License-Identifier: Apache-2.0
//
// Derived from canworks ec7fd59d, plugin/src/can/iec_location.h.
//
// iec_location.h - IEC 61131-3 located addresses (%IX10.0, %QW100, ...) and
// the CANopen data types that can be bound to them.

#ifndef CANOPEN_IEC_LOCATION_H
#define CANOPEN_IEC_LOCATION_H

#include <cstdint>
#include <string>

namespace canopen_plugin
{

enum class IecArea : char
{
    Input  = 'I',
    Output = 'Q'
};

// Size letter of the location. Each size is its own table in the OpenPLC
// image (bool_input, byte_input, int_input, dint_input, lint_input, and the
// output equivalents), so locations of different sizes never overlap.
enum class IecSize : char
{
    X = 'X',
    B = 'B',
    W = 'W',
    D = 'D',
    L = 'L'
};

struct IecLocation
{
    IecArea area   = IecArea::Input;
    IecSize size   = IecSize::X;
    uint32_t index = 0; // byte/word/... index into the table
    uint8_t bit    = 0; // only for IecSize::X

    std::string str() const;
    bool overlaps(const IecLocation &other) const;
};

// Parses "%IX10.0", "%QW100", ... Returns false with a reason on error.
bool parse_iec_location(const std::string &text, IecLocation &out, std::string &error);

// Size in bits of a location of this size (X = 1, B = 8, ..., L = 64).
unsigned iec_size_bits(IecSize size);

// CANopen basic data types supported in PDO entries (CiA 301 codes).
enum class CoType : uint16_t
{
    BOOLEAN    = 0x0001,
    INTEGER8   = 0x0002,
    INTEGER16  = 0x0003,
    INTEGER32  = 0x0004,
    UNSIGNED8  = 0x0005,
    UNSIGNED16 = 0x0006,
    UNSIGNED32 = 0x0007,
    REAL32     = 0x0008,
    REAL64     = 0x0011,
    INTEGER64  = 0x0015,
    UNSIGNED64 = 0x001B,
};

// Parses a type name ("UNSIGNED32", ...). Returns false if unsupported.
bool parse_co_type(const std::string &name, CoType &out);
const char *co_type_name(CoType type);
// Bits the type occupies in a PDO (BOOLEAN = 1).
unsigned co_type_bits(CoType type);
// Whether an object of this type fits a location of this size.
bool co_type_fits(CoType type, IecSize size);

} // namespace canopen_plugin

#endif // CANOPEN_IEC_LOCATION_H
