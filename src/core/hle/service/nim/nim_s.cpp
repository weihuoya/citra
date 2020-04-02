// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/hle/service/nim/nim_s.h"

namespace Service::NIM {

NIM_S::NIM_S() : ServiceFramework("nim:s", 1) {
    const FunctionInfo functions[] = {
        {0x00010200, nullptr, "StartDownloadSimple"},
        {0x00020000, nullptr, "CancelDownload"},
        {0x00030000, nullptr, "GetProgress"},
        {0x00050082, nullptr, "UnregisterTask"},
        {0x00060080, nullptr, "IsTaskRegistered"},
        {0x000A0000, nullptr, "CheckSysupdateAvailableSOAP"},
        {0x000B0084, nullptr, "SetAttribute"},
        {0x0016020A, nullptr, "ListTitles"},
        {0x00290000, nullptr, "AccountCheckBalanceSOAP"},
        {0x002D0042, nullptr, "DownloadTickets"},
        {0x003C0002, nullptr, "RegisterSelf"},
        {0x00420240, nullptr, "StartDownload"},
        {0x00550246, nullptr, "RegisterTask"},
        {0x00570082, nullptr, "ConnectNoTicketDownload"},
    };
    RegisterHandlers(functions);
}

NIM_S::~NIM_S() = default;

} // namespace Service::NIM
