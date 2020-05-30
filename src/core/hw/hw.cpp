// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/common_types.h"
#include "common/logging/log.h"
#include "core/hw/aes/key.h"
#include "core/hw/gpu.h"
#include "core/hw/hw.h"
#include "core/hw/lcd.h"

namespace HW {

template <typename T>
inline void Read(T& var, const u32 addr) {
    if ((addr & 0xFFFF0000) == VADDR_GPU) {
        GPU::Read(var, addr);
    } else if ((addr & 0xFFFFF000) == VADDR_LCD) {
        LCD::Read(var, addr);
    } else {
        LOG_ERROR(HW_Memory, "unknown Read{} @ {:#010X}", sizeof(var) * 8, addr);
    }
}

template <typename T>
inline void Write(u32 addr, const T data) {
    if ((addr & 0xFFFF0000) == VADDR_GPU) {
        GPU::Write(addr, data);
    } else if ((addr & 0xFFFFF000) == VADDR_LCD) {
        LCD::Write(addr, data);
    } else {
        LOG_ERROR(HW_Memory, "unknown Write{} {:#010X} @ {:#010X}", sizeof(data) * 8, (u32)data,
                  addr);
    }
}

// Explicitly instantiate template functions because we aren't defining this in the header:

template void Read<u64>(u64& var, const u32 addr);
template void Read<u32>(u32& var, const u32 addr);
template void Read<u16>(u16& var, const u32 addr);
template void Read<u8>(u8& var, const u32 addr);

template void Write<u64>(u32 addr, const u64 data);
template void Write<u32>(u32 addr, const u32 data);
template void Write<u16>(u32 addr, const u16 data);
template void Write<u8>(u32 addr, const u8 data);

/// Update hardware
void Update() {}

/// Initialize hardware
void Init(Memory::MemorySystem& memory) {
    AES::InitKeys();
    GPU::Init(memory);
    LCD::Init();
    LOG_DEBUG(HW, "initialized OK");
}

/// Shutdown hardware
void Shutdown() {
    GPU::Shutdown();
    LCD::Shutdown();
    LOG_DEBUG(HW, "shutdown OK");
}
} // namespace HW
