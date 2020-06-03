// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/common_types.h"
#include "common/hash.h"

#ifdef ARCHITECTURE_ARM64
// Arm C Language Extension
#include <arm_acle.h>
#endif
#include "common/cityhash.h"

namespace Common {
#ifdef ANDROID
u64 ComputeHash64(const void* data, u32 len) {
    const u8* p = static_cast<const u8*>(data);
    union {
        u64 crc64;
        u32 crc32[2];
    } result{};
    u32 i = 0;
    while (len >= 8) {
        result.crc32[i & 1] = __crc32d(result.crc32[i & 1], *((u64*)p));
        p += 8;
        len -= 8;
        i += 1;
    }
    if (len >= 4) {
        result.crc32[i & 1] = __crc32w(result.crc32[i & 1], *((u32*)p));
        p += 4;
        len -= 4;
        i += 1;
    }
    if (len >= 2) {
        result.crc32[i & 1] = __crc32h(result.crc32[i & 1], *((u16*)p));
        p += 2;
        len -= 2;
        i += 1;
    }
    if (len == 1)
        result.crc32[i & 1] = __crc32b(result.crc32[i & 1], *p);
    return result.crc64;
}
#else
u64 ComputeHash64(const void* data, u32 len) {
    return CityHash64(static_cast<const char*>(data), len);
}
#endif
u64 TextureHash64(const void* data, u32 len) {
    return CityHash64(static_cast<const char*>(data), len);
}
} // namespace Common
