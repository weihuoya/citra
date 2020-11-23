package org.citra.emu.utils;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.SharedPreferences;
import android.net.DhcpInfo;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.text.format.Formatter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.preference.PreferenceManager;

import org.citra.emu.R;
import org.citra.emu.ui.EmulationActivity;
import org.citra.emu.ui.MainActivity;

public class NetPlayManager {

    public static void ShowCreateRoomDialog(final Activity activity) {
        final AlertDialog dialog = new AlertDialog.Builder(activity)
                .setCancelable(true)
                .setView(R.layout.dialog_multiplayer_room)
                .show();

        final TextView textTitle = dialog.findViewById(R.id.text_title);
        textTitle.setText(R.string.multiplayer_create_room);

        final EditText ipaddressView = dialog.findViewById(R.id.ip_address);
        ipaddressView.setText(GetIpAddressByWifi(activity));

        final EditText usernameView = dialog.findViewById(R.id.username);
        usernameView.setText(GetUsername(activity));

        Button btnConfirm = dialog.findViewById(R.id.btn_confirm);
        btnConfirm.setOnClickListener(v -> {
            String ipaddress = ipaddressView.getText().toString();
            String username = usernameView.getText().toString();
            if (ipaddress.length() < 7 || username.length() < 5) {
                Toast.makeText(activity, R.string.multiplayer_input_invalid, Toast.LENGTH_LONG).show();
            } else if (NetPlayCreateRoom(ipaddress, username) == 0) {
                SetUsername(activity, username);
                Toast.makeText(activity, R.string.multiplayer_create_room_success, Toast.LENGTH_LONG).show();
                dialog.dismiss();
            } else {
                Toast.makeText(activity, R.string.multiplayer_create_room_failed, Toast.LENGTH_LONG).show();
            }
        });
    }

    public static void ShowJoinRoomDialog(final Activity activity) {
        final AlertDialog dialog = new AlertDialog.Builder(activity)
                .setCancelable(true)
                .setView(R.layout.dialog_multiplayer_room)
                .show();

        final TextView textTitle = dialog.findViewById(R.id.text_title);
        textTitle.setText(R.string.multiplayer_join_room);

        final EditText ipaddressView = dialog.findViewById(R.id.ip_address);
        ipaddressView.setText(GetRoomAddress(activity));

        final EditText usernameView = dialog.findViewById(R.id.username);
        usernameView.setText(GetUsername(activity));

        Button btnConfirm = dialog.findViewById(R.id.btn_confirm);
        btnConfirm.setOnClickListener(v -> {
            String ipaddress = ipaddressView.getText().toString();
            String username = usernameView.getText().toString();
            if (ipaddress.length() < 7 || username.length() < 5) {
                Toast.makeText(activity, R.string.multiplayer_input_invalid, Toast.LENGTH_LONG).show();
            } else if (NetPlayJoinRoom(ipaddress, username) == 0) {
                SetRoomAddress(activity, ipaddress);
                SetUsername(activity, username);
                Toast.makeText(activity, R.string.multiplayer_join_room_success, Toast.LENGTH_LONG).show();
                dialog.dismiss();
            } else {
                Toast.makeText(activity, R.string.multiplayer_join_room_failed, Toast.LENGTH_LONG).show();
            }
        });
    }

    private static String GetUsername(final Activity activity) {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(activity);
        String name = "Citra" + (int) (Math.random() * 100);
        return prefs.getString("NetPlayUsername", name);
    }

    private static void SetUsername(final Activity activity, final String name) {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(activity);
        prefs.edit().putString("NetPlayUsername", name).apply();
    }

    private static String GetRoomAddress(final Activity activity) {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(activity);
        String address = GetIpAddressByWifi(activity);
        return prefs.getString("NetPlayRoomAddress", address);
    }

    private static void SetRoomAddress(final Activity activity, final String address) {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(activity);
        prefs.edit().putString("NetPlayRoomAddress", address).apply();
    }

    private static native int NetPlayCreateRoom(String ipaddress, String username);

    private static native int NetPlayJoinRoom(String ipaddress, String username);

    public static native String[] NetPlayRoomInfo();

    public static native boolean NetPlayIsHostedRoom();

    public static native void NetPlaySendMessage(String msg);

    public static native void NetPlayKickUser(String username);

    public static native void NetPlayLeaveRoom();

    public static void AddNetPlayMessage(int type, String msg) {
        EmulationActivity activity1 = EmulationActivity.get();
        MainActivity activity2 = MainActivity.get();

        if (activity1 != null) {
            activity1.runOnUiThread(() -> {
                activity1.addNetPlayMessage(FormatNetPlayStatus(activity1, type, msg));
            });
        } else if (activity2 != null) {
            activity2.runOnUiThread(() -> {
                activity2.addNetPlayMessage(FormatNetPlayStatus(activity2, type, msg));
            });
        }
    }

    private static String FormatNetPlayStatus(Context context, int type, String msg) {
        switch (type) {
        case NetPlayStatus.NETWORK_ERROR:
            return context.getString(R.string.multiplayer_network_error);
        case NetPlayStatus.LOST_CONNECTION:
            return context.getString(R.string.multiplayer_lost_connection);
        case NetPlayStatus.NAME_COLLISION:
            return context.getString(R.string.multiplayer_name_collision);
        case NetPlayStatus.MAC_COLLISION:
            return context.getString(R.string.multiplayer_mac_collision);
        case NetPlayStatus.CONSOLE_ID_COLLISION:
            return context.getString(R.string.multiplayer_console_id_collision);
        case NetPlayStatus.WRONG_VERSION:
            return context.getString(R.string.multiplayer_wrong_version);
        case NetPlayStatus.WRONG_PASSWORD:
            return context.getString(R.string.multiplayer_wrong_password);
        case NetPlayStatus.COULD_NOT_CONNECT:
            return context.getString(R.string.multiplayer_could_not_connect);
        case NetPlayStatus.ROOM_IS_FULL:
            return context.getString(R.string.multiplayer_room_is_full);
        case NetPlayStatus.HOST_BANNED:
            return context.getString(R.string.multiplayer_host_banned);
        case NetPlayStatus.PERMISSION_DENIED:
            return context.getString(R.string.multiplayer_permission_denied);
        case NetPlayStatus.NO_SUCH_USER:
            return context.getString(R.string.multiplayer_no_such_user);
        case NetPlayStatus.ALREADY_IN_ROOM:
            return context.getString(R.string.multiplayer_already_in_room);
        case NetPlayStatus.CREATE_ROOM_ERROR:
            return context.getString(R.string.multiplayer_create_room_error);
        case NetPlayStatus.HOST_KICKED:
            return context.getString(R.string.multiplayer_host_kicked);
        case NetPlayStatus.UNKNOWN_ERROR:
            return context.getString(R.string.multiplayer_unknown_error);

        case NetPlayStatus.ROOM_UNINITIALIZED:
            return context.getString(R.string.multiplayer_room_uninitialized);
        case NetPlayStatus.ROOM_IDLE:
            return context.getString(R.string.multiplayer_room_idle);
        case NetPlayStatus.ROOM_JOINING:
            return context.getString(R.string.multiplayer_room_joining);
        case NetPlayStatus.ROOM_JOINED:
            return context.getString(R.string.multiplayer_room_joined);
        case NetPlayStatus.ROOM_MODERATOR:
            return context.getString(R.string.multiplayer_room_moderator);

        case NetPlayStatus.MEMBER_JOIN:
            return context.getString(R.string.multiplayer_member_join, msg);
        case NetPlayStatus.MEMBER_LEAVE:
            return context.getString(R.string.multiplayer_member_leave, msg);
        case NetPlayStatus.MEMBER_KICKED:
            return context.getString(R.string.multiplayer_member_kicked, msg);
        case NetPlayStatus.MEMBER_BANNED:
            return context.getString(R.string.multiplayer_member_banned, msg);
        case NetPlayStatus.ADDRESS_UNBANNED:
            return context.getString(R.string.multiplayer_address_unbanned);

        case NetPlayStatus.CHAT_MESSAGE:
            return msg;
        }
        return "";
    }

    @SuppressWarnings("deprecation")
    private static String GetIpAddressByWifi(final Activity activity) {
        int ipaddress = 0;
        WifiManager wifiManager = (WifiManager) activity.getSystemService(Context.WIFI_SERVICE);
        WifiInfo wifiInfo = wifiManager.getConnectionInfo();
        if (wifiInfo != null) {
            ipaddress = wifiInfo.getIpAddress();
        }

        if (ipaddress == 0) {
            DhcpInfo dhcpInfo = wifiManager.getDhcpInfo();
            if (dhcpInfo != null) {
                ipaddress = dhcpInfo.ipAddress;
            }
        }

        if (ipaddress == 0) {
            return "192.168.0.1";
        } else {
            return Formatter.formatIpAddress(ipaddress);
        }
    }

    public static final class NetPlayStatus {
        public static final int NO_ERROR = 0;

        public static final int NETWORK_ERROR = 1;
        public static final int LOST_CONNECTION = 2;
        public static final int NAME_COLLISION = 3;
        public static final int MAC_COLLISION = 4;
        public static final int CONSOLE_ID_COLLISION = 5;
        public static final int WRONG_VERSION = 6;
        public static final int WRONG_PASSWORD = 7;
        public static final int COULD_NOT_CONNECT = 8;
        public static final int ROOM_IS_FULL = 9;
        public static final int HOST_BANNED = 10;
        public static final int PERMISSION_DENIED = 11;
        public static final int NO_SUCH_USER = 12;
        public static final int ALREADY_IN_ROOM = 13;
        public static final int CREATE_ROOM_ERROR = 14;
        public static final int HOST_KICKED = 15;
        public static final int UNKNOWN_ERROR = 16;

        public static final int ROOM_UNINITIALIZED = 17;
        public static final int ROOM_IDLE = 18;
        public static final int ROOM_JOINING = 19;
        public static final int ROOM_JOINED = 20;
        public static final int ROOM_MODERATOR = 21;

        public static final int MEMBER_JOIN = 22;
        public static final int MEMBER_LEAVE = 23;
        public static final int MEMBER_KICKED = 24;
        public static final int MEMBER_BANNED = 25;
        public static final int ADDRESS_UNBANNED = 26;

        public static final int CHAT_MESSAGE = 27;
    }
}
