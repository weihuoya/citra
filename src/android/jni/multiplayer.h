#pragma once

#include <string>
#include <vector>

enum class NetPlayStatus {
    NO_ERROR,

    NETWORK_ERROR,
    LOST_CONNECTION,
    NAME_COLLISION,
    MAC_COLLISION,
    CONSOLE_ID_COLLISION,
    WRONG_VERSION,
    WRONG_PASSWORD,
    COULD_NOT_CONNECT,
    ROOM_IS_FULL,
    HOST_BANNED,
    PERMISSION_DENIED,
    NO_SUCH_USER,
    ALREADY_IN_ROOM,
    CREATE_ROOM_ERROR,
    HOST_KICKED,
    UNKNOWN_ERROR,

    ROOM_UNINITIALIZED,
    ROOM_IDLE,
    ROOM_JOINING,
    ROOM_JOINED,
    ROOM_MODERATOR,

    MEMBER_JOIN,
    MEMBER_LEAVE,
    MEMBER_KICKED,
    MEMBER_BANNED,
    ADDRESS_UNBANNED,

    CHAT_MESSAGE,
};

bool NetworkInit();
NetPlayStatus NetPlayCreateRoom(const std::string& ipaddress, const std::string& username);
NetPlayStatus NetPlayJoinRoom(const std::string& ipaddress, const std::string& username);
std::vector<std::string> NetPlayRoomInfo();
bool NetPlayIsHostedRoom();
void NetPlaySendMessage(const std::string& msg);
void NetPlayKickUser(const std::string& username);
void NetPlayLeaveRoom();
void NetworkShutdown();
