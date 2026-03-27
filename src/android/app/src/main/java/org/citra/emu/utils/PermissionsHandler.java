package org.citra.emu.utils;

import static android.Manifest.permission.CAMERA;
import static android.Manifest.permission.RECORD_AUDIO;
import static android.Manifest.permission.WRITE_EXTERNAL_STORAGE;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Intent;
import android.content.Context;
import android.content.DialogInterface;
import android.net.Uri;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Environment;
import android.provider.Settings;
import android.widget.Toast;

import androidx.core.content.ContextCompat;

import org.citra.emu.R;

public final class PermissionsHandler {
    public static final int REQUEST_CODE_WRITE_PERMISSION = 500;
    public static final int REQUEST_CODE_CAMERA_PERMISSION = 501;
    public static final int REQUEST_CODE_RECORD_PERMISSION = 502;
    public static final int REQUEST_CODE_ALL_FILES_ACCESS = 503;

    public static boolean checkWritePermission(final Activity activity) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            if (Environment.isExternalStorageManager()) {
                return true;
            }

            showMessageOKCancel(
                activity, activity.getString(R.string.all_files_access_needed),
                (dialog, which) -> requestAllFilesAccess(activity));
            return false;
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q && !Environment.isExternalStorageLegacy()) {
            return true;
        }

        int hasWritePermission =
            ContextCompat.checkSelfPermission(activity, WRITE_EXTERNAL_STORAGE);

        if (hasWritePermission != PackageManager.PERMISSION_GRANTED) {
            if (activity.shouldShowRequestPermissionRationale(WRITE_EXTERNAL_STORAGE)) {
                showMessageOKCancel(
                    activity, activity.getString(R.string.write_permission_needed),
                    (dialog, which)
                        -> activity.requestPermissions(new String[] {WRITE_EXTERNAL_STORAGE},
                                                       REQUEST_CODE_WRITE_PERMISSION));
                return false;
            }

            activity.requestPermissions(new String[] {WRITE_EXTERNAL_STORAGE},
                                        REQUEST_CODE_WRITE_PERMISSION);
            return false;
        }

        return true;
    }

    public static boolean checkCameraPermission(final Activity activity) {
        int permission = ContextCompat.checkSelfPermission(activity, CAMERA);
        if (permission != PackageManager.PERMISSION_GRANTED) {
            if (activity.shouldShowRequestPermissionRationale(CAMERA)) {
                showMessageOKCancel(activity, activity.getString(R.string.camera_permission_needed),
                                    (dialog, which)
                                        -> activity.requestPermissions(
                                            new String[] {CAMERA}, REQUEST_CODE_CAMERA_PERMISSION));
                return false;
            }

            activity.requestPermissions(new String[] {CAMERA}, REQUEST_CODE_CAMERA_PERMISSION);
            return false;
        }

        return true;
    }

    public static boolean checkRecordPermission(final Activity activity) {
        int permission = ContextCompat.checkSelfPermission(activity, RECORD_AUDIO);
        if (permission != PackageManager.PERMISSION_GRANTED) {
            if (activity.shouldShowRequestPermissionRationale(RECORD_AUDIO)) {
                showMessageOKCancel(
                    activity, activity.getString(R.string.record_permission_needed),
                    (dialog, which)
                        -> activity.requestPermissions(new String[] {RECORD_AUDIO},
                                                       REQUEST_CODE_RECORD_PERMISSION));
                return false;
            }

            activity.requestPermissions(new String[] {RECORD_AUDIO},
                                        REQUEST_CODE_RECORD_PERMISSION);
            return false;
        }
        return true;
    }

    public static boolean hasWriteAccess(Context context) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            return Environment.isExternalStorageManager();
        }
        int hasWritePermission =
            ContextCompat.checkSelfPermission(context, WRITE_EXTERNAL_STORAGE);
        return hasWritePermission == PackageManager.PERMISSION_GRANTED;

    }

    private static void requestAllFilesAccess(Activity activity) {
        Intent intent = new Intent(Settings.ACTION_MANAGE_APP_ALL_FILES_ACCESS_PERMISSION);
        intent.setData(Uri.parse("package:" + activity.getPackageName()));
        try {
            activity.startActivityForResult(intent, REQUEST_CODE_ALL_FILES_ACCESS);
        } catch (Exception e) {
            activity.startActivityForResult(
                new Intent(Settings.ACTION_MANAGE_ALL_FILES_ACCESS_PERMISSION),
                REQUEST_CODE_ALL_FILES_ACCESS);
        }
    }

    private static void showMessageOKCancel(final Context context, String message,
                                            DialogInterface.OnClickListener okListener) {
        new AlertDialog.Builder(context)
            .setMessage(message)
            .setPositiveButton(android.R.string.ok, okListener)
            .setNegativeButton(
                android.R.string.cancel,
                (dialogInterface, i)
                    -> Toast.makeText(context, R.string.write_permission_needed, Toast.LENGTH_SHORT)
                           .show())
            .create()
            .show();
    }
}
