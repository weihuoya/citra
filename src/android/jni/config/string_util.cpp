// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "config/string_util.h"

#include <algorithm>
#include <istream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "common/common_types.h"

bool TryParse(const std::string& str, u8* const output) {
    u64 value;
    if (!TryParse(str, &value))
        return false;

    if (value >= 0x100ull && value <= 0xFFFFFFFFFFFFFF00ull)
        return false;

    *output = static_cast<u8>(value);
    return true;
}

bool TryParse(const std::string& str, u16* const output) {
    u64 value;
    if (!TryParse(str, &value))
        return false;

    if (value >= 0x10000ull && value <= 0xFFFFFFFFFFFF0000ull)
        return false;

    *output = static_cast<u16>(value);
    return true;
}

bool TryParse(const std::string& str, u32* const output) {
    u64 value;
    if (!TryParse(str, &value))
        return false;

    if (value >= 0x100000000ull && value <= 0xFFFFFFFF00000000ull)
        return false;

    *output = static_cast<u32>(value);
    return true;
}

bool TryParse(const std::string& str, u64* const output) {
    char* end_ptr = nullptr;

    // Set errno to a clean slate
    errno = 0;

    u64 value = strtoull(str.c_str(), &end_ptr, 0);

    if (end_ptr == nullptr || *end_ptr != '\0')
        return false;

    if (errno == ERANGE)
        return false;

    *output = value;
    return true;
}

bool TryParse(const std::string& str, bool* const output) {
    float value;
    const bool is_valid_float = TryParse(str, &value);
    if ((is_valid_float && value == 1) || !strcasecmp("true", str.c_str()))
        *output = true;
    else if ((is_valid_float && value == 0) || !strcasecmp("false", str.c_str()))
        *output = false;
    else
        return false;

    return true;
}

std::string ValueToString(u16 value) {
    return std::to_string(value);
}

std::string ValueToString(u32 value) {
    return std::to_string(value);
}

std::string ValueToString(u64 value) {
    return std::to_string(value);
}

std::string ValueToString(float value) {
    return fmt::format("{:#.9g}", value);
}

std::string ValueToString(double value) {
    return fmt::format("{:#.17g}", value);
}

std::string ValueToString(int value) {
    return std::to_string(value);
}

std::string ValueToString(s64 value) {
    return std::to_string(value);
}

std::string ValueToString(bool value) {
    return value ? "True" : "False";
}

std::string ValueToString(const std::string& value) {
    return value;
}
