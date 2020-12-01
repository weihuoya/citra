#include <android/log.h>

#include "multiplayer.h"
#include "jni_common.h"

#include "network/network.h"
#include "core/core.h"
#include "core/settings.h"
#include "core/hle/service/cfg/cfg.h"

bool NetworkInit() {
    bool result = Network::Init();

    if (!result) {
        return false;
    }

    if (auto member = Network::GetRoomMember().lock()) {
        // register the network structs to use in slots and signals
        member->BindOnStateChanged(
                [](const Network::RoomMember::State &state) {
                    NetPlayStatus status;
                    std::string msg;
                    switch (state) {
                    case Network::RoomMember::State::Uninitialized:
                        status = NetPlayStatus::ROOM_UNINITIALIZED;
                        break;
                    case Network::RoomMember::State::Idle:
                        status = NetPlayStatus::ROOM_IDLE;
                        break;
                    case Network::RoomMember::State::Joining:
                        status = NetPlayStatus::ROOM_JOINING;
                        break;
                    case Network::RoomMember::State::Joined:
                        status = NetPlayStatus::ROOM_JOINED;
                        break;
                    case Network::RoomMember::State::Moderator:
                        status = NetPlayStatus::ROOM_MODERATOR;
                        break;
                    }
                    AddNetPlayMessage(static_cast<int>(status), msg);
                });
        member->BindOnError(
                [](const Network::RoomMember::Error &error) {
                    NetPlayStatus status;
                    std::string msg;
                    switch (error) {
                    case Network::RoomMember::Error::LostConnection:
                        status = NetPlayStatus::LOST_CONNECTION;
                        break;
                    case Network::RoomMember::Error::HostKicked:
                        status = NetPlayStatus::HOST_KICKED;
                        break;
                    case Network::RoomMember::Error::UnknownError:
                        status = NetPlayStatus::UNKNOWN_ERROR;
                        break;
                    case Network::RoomMember::Error::NameCollision:
                        status = NetPlayStatus::NAME_COLLISION;
                        break;
                    case Network::RoomMember::Error::MacCollision:
                        status = NetPlayStatus::MAC_COLLISION;
                        break;
                    case Network::RoomMember::Error::ConsoleIdCollision:
                        status = NetPlayStatus::CONSOLE_ID_COLLISION;
                        break;
                    case Network::RoomMember::Error::WrongVersion:
                        status = NetPlayStatus::WRONG_VERSION;
                        break;
                    case Network::RoomMember::Error::WrongPassword:
                        status = NetPlayStatus::WRONG_PASSWORD;
                        break;
                    case Network::RoomMember::Error::CouldNotConnect:
                        status = NetPlayStatus::COULD_NOT_CONNECT;
                        break;
                    case Network::RoomMember::Error::RoomIsFull:
                        status = NetPlayStatus::ROOM_IS_FULL;
                        break;
                    case Network::RoomMember::Error::HostBanned:
                        status = NetPlayStatus::HOST_BANNED;
                        break;
                    case Network::RoomMember::Error::PermissionDenied:
                        status = NetPlayStatus::PERMISSION_DENIED;
                        break;
                    case Network::RoomMember::Error::NoSuchUser:
                        status = NetPlayStatus::NO_SUCH_USER;
                        break;
                    }
                    AddNetPlayMessage(static_cast<int>(status), msg);
                });
        member->BindOnStatusMessageReceived(
                [](const Network::StatusMessageEntry &status_message) {
                    NetPlayStatus status = NetPlayStatus::NO_ERROR;
                    std::string msg(status_message.nickname);
                    switch (status_message.type) {
                    case Network::IdMemberJoin:
                        status = NetPlayStatus::MEMBER_JOIN;
                        break;
                    case Network::IdMemberLeave:
                        status = NetPlayStatus::MEMBER_LEAVE;
                        break;
                    case Network::IdMemberKicked:
                        status = NetPlayStatus::MEMBER_KICKED;
                        break;
                    case Network::IdMemberBanned:
                        status = NetPlayStatus::MEMBER_BANNED;
                        break;
                    case Network::IdAddressUnbanned:
                        status = NetPlayStatus::ADDRESS_UNBANNED;
                        break;
                    }
                    AddNetPlayMessage(static_cast<int>(status), msg);
                });
        member->BindOnChatMessageRecieved(
                [](const Network::ChatEntry& chat) {
                    NetPlayStatus status = NetPlayStatus::CHAT_MESSAGE;
                    std::string msg(chat.nickname);
                    msg += ": ";
                    msg += chat.message;
                    AddNetPlayMessage(static_cast<int>(status), msg);
                });
    }

    return true;
}

NetPlayStatus NetPlayCreateRoom(const std::string& ipaddress, int port, const std::string& username) {
    auto member = Network::GetRoomMember().lock();
    if (!member) {
        return NetPlayStatus::NETWORK_ERROR;
    }

    if (member->GetState() == Network::RoomMember::State::Joining || member->IsConnected()) {
        return NetPlayStatus::ALREADY_IN_ROOM;
    }

    auto room = Network::GetRoom().lock();
    if (!room) {
        return NetPlayStatus::NETWORK_ERROR;
    }

    if (!room->Create(ipaddress, "", "", port, "", 8, username)) {
        return NetPlayStatus::CREATE_ROOM_ERROR;
    }

    std::string console = Service::CFG::GetConsoleIdHash(Core::System::GetInstance());
    member->Join(username, console, "127.0.0.1", port);
    return NetPlayStatus::NO_ERROR;
}

NetPlayStatus NetPlayJoinRoom(const std::string& ipaddress, int port, const std::string& username) {
    auto member = Network::GetRoomMember().lock();
    if (!member) {
        return NetPlayStatus::NETWORK_ERROR;
    }

    if (member->GetState() == Network::RoomMember::State::Joining || member->IsConnected()) {
        return NetPlayStatus::ALREADY_IN_ROOM;
    }

    std::string console = Service::CFG::GetConsoleIdHash(Core::System::GetInstance());
    member->Join(username, console, ipaddress.c_str(), port);
    return NetPlayStatus::NO_ERROR;
}

void NetPlaySendMessage(const std::string& msg) {
    if (auto room = Network::GetRoomMember().lock()) {
        if (room->GetState() != Network::RoomMember::State::Joined &&
            room->GetState() != Network::RoomMember::State::Moderator) {

            return;
        }
        room->SendChatMessage(msg);
    }
}

void NetPlayKickUser(const std::string& username) {
    if (auto room = Network::GetRoomMember().lock()) {
        auto members = room->GetMemberInformation();
        auto it = std::find_if(members.begin(), members.end(),
                               [&username](const Network::RoomMember::MemberInformation& member) {
                                   return member.nickname == username;
                               });
        if (it != members.end()) {
            room->SendModerationRequest(Network::RoomMessageTypes::IdModKick, username);
        }
    }
}

std::vector<std::string> NetPlayRoomInfo() {
    std::vector<std::string> infolist;
    if (auto room = Network::GetRoomMember().lock()) {
        auto members = room->GetMemberInformation();
        if (!members.empty()) {
            // name
            infolist.push_back(room->GetRoomInformation().name);
            // all
            for (const auto& member : members) {
                infolist.push_back(member.nickname);
            }
        }
    }
    return infolist;
}

bool NetPlayIsHostedRoom() {
    if (auto room = Network::GetRoom().lock()) {
        return room->GetState() == Network::Room::State::Open;
    }
    return false;
}

void NetPlayLeaveRoom() {
    if (auto room = Network::GetRoom().lock()) {
        // if you are in a room, leave it
        if (auto member = Network::GetRoomMember().lock()) {
            member->Leave();
        }

        // if you are hosting a room, also stop hosting
        if (room->GetState() == Network::Room::State::Open) {
            room->Destroy();
        }
    }
}

void NetworkShutdown() {
    Network::Shutdown();
}
