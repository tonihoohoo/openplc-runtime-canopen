// SPDX-License-Identifier: Apache-2.0
//
// Derived from canworks ec7fd59d, plugin/src/can/iec_location.cpp.

#include "iec_location.h"

#include <cctype>
#include <cstdlib>

namespace canopen_plugin
{

std::string IecLocation::str() const
{
    std::string s = "%";
    s += static_cast<char>(area);
    s += static_cast<char>(size);
    s += std::to_string(index);
    if (size == IecSize::X)
    {
        s += ".";
        s += std::to_string(bit);
    }
    return s;
}

bool IecLocation::overlaps(const IecLocation &other) const
{
    if (area != other.area || size != other.size || index != other.index)
        return false;
    return size != IecSize::X || bit == other.bit;
}

namespace
{

bool parse_uint(const std::string &s, size_t &pos, uint32_t &out)
{
    size_t start = pos;
    uint64_t v   = 0;
    while (pos < s.size() && std::isdigit(static_cast<unsigned char>(s[pos])))
    {
        v = v * 10 + (s[pos] - '0');
        if (v > 0xFFFFFFFFu)
            return false;
        ++pos;
    }
    if (pos == start)
        return false;
    out = static_cast<uint32_t>(v);
    return true;
}

} // namespace

bool parse_iec_location(const std::string &text, IecLocation &out, std::string &error)
{
    const std::string &s = text;
    if (s.size() < 4 || s[0] != '%')
    {
        error = "expected a location like %IX0.0 or %QW10";
        return false;
    }
    char area = std::toupper(static_cast<unsigned char>(s[1]));
    if (area != 'I' && area != 'Q')
    {
        error = "only input (%I) and output (%Q) locations are supported";
        return false;
    }
    char size = std::toupper(static_cast<unsigned char>(s[2]));
    if (size != 'X' && size != 'B' && size != 'W' && size != 'D' && size != 'L')
    {
        error = "size must be one of X, B, W, D, L";
        return false;
    }
    size_t pos     = 3;
    uint32_t index = 0;
    if (!parse_uint(s, pos, index))
    {
        error = "missing or invalid index";
        return false;
    }
    uint32_t bit = 0;
    if (size == 'X')
    {
        if (pos >= s.size() || s[pos] != '.')
        {
            error = "bit locations need a bit number, e.g. %IX10.0";
            return false;
        }
        ++pos;
        if (!parse_uint(s, pos, bit) || bit > 7)
        {
            error = "bit number must be 0-7";
            return false;
        }
    }
    if (pos != s.size())
    {
        error = "unexpected characters after the location";
        return false;
    }
    out.area  = static_cast<IecArea>(area);
    out.size  = static_cast<IecSize>(size);
    out.index = index;
    out.bit   = static_cast<uint8_t>(bit);
    return true;
}

unsigned iec_size_bits(IecSize size)
{
    switch (size)
    {
    case IecSize::X:
        return 1;
    case IecSize::B:
        return 8;
    case IecSize::W:
        return 16;
    case IecSize::D:
        return 32;
    case IecSize::L:
        return 64;
    }
    return 0;
}

namespace
{

struct TypeInfo
{
    CoType type;
    const char *name;
    unsigned bits;
};

const TypeInfo kTypes[] = {
    {CoType::BOOLEAN, "BOOLEAN", 1},        {CoType::INTEGER8, "INTEGER8", 8},
    {CoType::INTEGER16, "INTEGER16", 16},   {CoType::INTEGER32, "INTEGER32", 32},
    {CoType::UNSIGNED8, "UNSIGNED8", 8},    {CoType::UNSIGNED16, "UNSIGNED16", 16},
    {CoType::UNSIGNED32, "UNSIGNED32", 32}, {CoType::REAL32, "REAL32", 32},
    {CoType::REAL64, "REAL64", 64},         {CoType::INTEGER64, "INTEGER64", 64},
    {CoType::UNSIGNED64, "UNSIGNED64", 64},
};

const TypeInfo *find_type(CoType type)
{
    for (const auto &t : kTypes)
        if (t.type == type)
            return &t;
    return nullptr;
}

} // namespace

bool parse_co_type(const std::string &name, CoType &out)
{
    std::string upper;
    for (char c : name)
        upper += std::toupper(static_cast<unsigned char>(c));
    for (const auto &t : kTypes)
    {
        if (upper == t.name)
        {
            out = t.type;
            return true;
        }
    }
    return false;
}

const char *co_type_name(CoType type)
{
    const TypeInfo *t = find_type(type);
    return t ? t->name : "UNKNOWN";
}

bool dummy_object_type(uint16_t index, uint8_t subindex, CoType &out)
{
    if (subindex != 0 || index < 0x0001 || index > 0x0007)
        return false;
    out = static_cast<CoType>(index);
    return true;
}

unsigned co_type_bits(CoType type)
{
    const TypeInfo *t = find_type(type);
    return t ? t->bits : 0;
}

bool co_type_fits(CoType type, IecSize size)
{
    return co_type_bits(type) == iec_size_bits(size);
}

} // namespace canopen_plugin
