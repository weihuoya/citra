// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "common/common_types.h"

bool TryParse(const std::string& str, bool* output);
bool TryParse(const std::string& str, u8* output);
bool TryParse(const std::string& str, u16* output);
bool TryParse(const std::string& str, u32* output);
bool TryParse(const std::string& str, u64* output);

template <typename N>
inline bool TryParse(const std::string& str, N* const output) {
    std::istringstream iss(str);
    // is this right? not doing this breaks reading floats on locales that use different decimal
    // separators
    iss.imbue(std::locale("C"));

    N tmp;
    if (iss >> tmp) {
        *output = tmp;
        return true;
    }

    return false;
}

template <typename N>
inline bool TryParseVector(const std::string& str, std::vector<N>* output,
                           const char delimiter = ',') {
    output->clear();
    std::istringstream buffer(str);
    std::string variable;

    while (std::getline(buffer, variable, delimiter)) {
        N tmp = 0;
        if (!TryParse(variable, &tmp))
            return false;
        output->push_back(tmp);
    }
    return true;
}

std::string ValueToString(u16 value);
std::string ValueToString(u32 value);
std::string ValueToString(u64 value);
std::string ValueToString(float value);
std::string ValueToString(double value);
std::string ValueToString(int value);
std::string ValueToString(s64 value);
std::string ValueToString(bool value);
std::string ValueToString(const std::string& value);
template <typename T, std::enable_if_t<std::is_enum<T>::value>* = nullptr>
inline std::string ValueToString(T value) {
    return ValueToString(static_cast<std::underlying_type_t<T>>(value));
}
