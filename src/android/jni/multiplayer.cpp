#include <android/log.h>
#include "multiplayer.h"
#include "network/network.h"
#include "core/core.h"
#include "core/hle/service/cfg/cfg.h"

bool NetworkInit() {
    return Network::Init();
}

void NetworkRoomMemberJoin() {
    if (auto room_member = Network::GetRoomMember().lock()) {
        std::string nickname = "Android";
        std::string console_id = Service::CFG::GetConsoleIdHash(Core::System::GetInstance());
        std::string server_addr = "172.10.1.52";
        room_member->Join(nickname, console_id, server_addr.c_str());
    }
}

void NetworkShutdown() {
    Network::Shutdown();
}
