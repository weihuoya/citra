// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/core.h"
#include "core/hle/ipc_helpers.h"
#include "core/hle/service/act/act.h"
#include "core/hle/service/act/act_a.h"
#include "core/hle/service/act/act_u.h"

namespace Service::ACT {

Module::Interface::Interface(std::shared_ptr<Module> act, const char* name)
    : ServiceFramework(name, 1 /* Placeholder */), act(std::move(act)) {}

Module::Interface::~Interface() = default;

void InstallInterfaces(Core::System& system) {
    auto& service_manager = system.ServiceManager();
    auto act = std::make_shared<Module>();
    std::make_shared<ACT_A>(act)->InstallAsService(service_manager);
    std::make_shared<ACT_U>(act)->InstallAsService(service_manager);
}

void Module::Interface::Initialize(Kernel::HLERequestContext& ctx) {
    IPC::RequestParser rp(ctx, 0x1, 2, 4);
    const u32 shared_memory_size = rp.Pop<u32>();
    const u32 version = rp.Pop<u32>();
    u32 PopPID();
    rp.Skip(2, false); // ProcessId descriptor
    IPC::RequestBuilder rb = rp.MakeBuilder(1, 0);
    rb.Push(RESULT_SUCCESS);
    LOG_WARNING(Service, "called, version={:#X}, shared_memory_size={:#X}", version,
                shared_memory_size);
}

void Module::Interface::UnknownMehod0x000E0080(Kernel::HLERequestContext& ctx) {
    IPC::RequestParser rp(ctx, 0x0E, 1, 2);
    // UNREACHABLE();
    LOG_WARNING(Service, "");
}

} // namespace Service::ACT
