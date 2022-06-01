package org.citra.emu.utils;

import android.app.Activity;
import android.app.ActivityOptions;
import android.content.Intent;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;

import androidx.annotation.Nullable;

import com.nononsenseapps.filepicker.FilePickerActivity;
import com.nononsenseapps.filepicker.Utils;
import java.io.File;
import java.util.List;

import org.citra.emu.R;
import org.citra.emu.ui.GameFilePickerActivity;

public final class FileBrowserHelper {
    public static final int REQUEST_OPEN_DIRECTORY = 1;
    public static final int REQUEST_OPEN_FILE = 2;
    public static final int REQUEST_OPEN_DOCUMENT_TREE = 3;
    public static final int REQUEST_OPEN_DOCUMENT = 4;

    public static void openDirectoryPicker(Activity activity) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            if (!Environment.isExternalStorageLegacy()) {
                openDocumentTree(activity, REQUEST_OPEN_DOCUMENT_TREE);
                return;
            }
        }

        Intent i = new Intent(activity, GameFilePickerActivity.class);
        i.putExtra(FilePickerActivity.EXTRA_ALLOW_MULTIPLE, false);
        i.putExtra(FilePickerActivity.EXTRA_MODE, FilePickerActivity.MODE_DIR);
        i.putExtra(FilePickerActivity.EXTRA_START_PATH,
                   Environment.getExternalStorageDirectory().getPath());
        activity.startActivityForResult(i, REQUEST_OPEN_DIRECTORY);
    }

    public static void openFilePicker(Activity activity) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            if (!Environment.isExternalStorageLegacy()) {
                openDocument(activity, REQUEST_OPEN_DOCUMENT);
                return;
            }
        }

        Intent i = new Intent(activity, GameFilePickerActivity.class);
        i.putExtra(FilePickerActivity.EXTRA_ALLOW_MULTIPLE, true);
        i.putExtra(FilePickerActivity.EXTRA_MODE, FilePickerActivity.MODE_FILE);
        i.putExtra(FilePickerActivity.EXTRA_START_PATH,
                   Environment.getExternalStorageDirectory().getPath());
        activity.startActivityForResult(i, REQUEST_OPEN_FILE);
    }

    @Nullable
    public static String getSelectedDirectory(Intent result) {
        // Use the provided utility method to parse the result
        List<Uri> files = Utils.getSelectedFilesFromResult(result);
        if (!files.isEmpty()) {
            File file = Utils.getFileForUri(files.get(0));
            return file.getAbsolutePath();
        }

        return null;
    }

    @Nullable
    public static String[] getSelectedFiles(Intent result) {
        // Use the provided utility method to parse the result
        List<Uri> files = Utils.getSelectedFilesFromResult(result);
        if (!files.isEmpty()) {
            String[] paths = new String[files.size()];
            for (int i = 0; i < files.size(); i++)
                paths[i] = Utils.getFileForUri(files.get(i)).getAbsolutePath();
            return paths;
        }

        return null;
    }

    public static void openDocumentTree(Activity activity, int requestCode) {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT_TREE);
        intent.addCategory(Intent.CATEGORY_DEFAULT);
        intent.putExtra(Intent.EXTRA_LOCAL_ONLY, true);
        intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
        intent.addFlags(Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION);
        intent = Intent.createChooser(intent, activity.getString(R.string.main_choose_directory));
        Bundle bundle = ActivityOptions.makeSceneTransitionAnimation(activity).toBundle();
        activity.startActivityForResult(intent, requestCode, bundle);
    }

    public static void openDocument(Activity activity, int requestCode) {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_DEFAULT);
        intent.setType("*/*");
        intent.putExtra(Intent.EXTRA_ALLOW_MULTIPLE, true);
        intent.putExtra(Intent.EXTRA_LOCAL_ONLY, true);
        intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
        intent = Intent.createChooser(intent, activity.getString(R.string.grid_menu_install_cia));
        Bundle bundle = ActivityOptions.makeSceneTransitionAnimation(activity).toBundle();
        activity.startActivityForResult(intent, requestCode, bundle);

    }
}
