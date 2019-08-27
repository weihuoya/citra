package org.citra.emu.utils;

import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.os.Environment;
import android.support.annotation.Nullable;
import com.nononsenseapps.filepicker.FilePickerActivity;
import com.nononsenseapps.filepicker.Utils;
import java.io.File;
import java.util.List;
import org.citra.emu.ui.GameFilePickerActivity;
import org.citra.emu.ui.MainActivity;

public final class FileBrowserHelper {
    public static void openDirectoryPicker(Activity activity) {
        Intent i = new Intent(activity, GameFilePickerActivity.class);
        i.putExtra(FilePickerActivity.EXTRA_ALLOW_MULTIPLE, false);
        i.putExtra(FilePickerActivity.EXTRA_MODE, FilePickerActivity.MODE_DIR);
        i.putExtra(FilePickerActivity.EXTRA_START_PATH,
                   Environment.getExternalStorageDirectory().getPath());
        activity.startActivityForResult(i, MainActivity.REQUEST_ADD_DIRECTORY);
    }

    public static void openFilePicker(Activity activity, int requestCode) {
        Intent i = new Intent(activity, GameFilePickerActivity.class);
        i.putExtra(FilePickerActivity.EXTRA_ALLOW_MULTIPLE, true);
        i.putExtra(FilePickerActivity.EXTRA_MODE, FilePickerActivity.MODE_FILE);
        i.putExtra(FilePickerActivity.EXTRA_START_PATH,
                   Environment.getExternalStorageDirectory().getPath());
        activity.startActivityForResult(i, requestCode);
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
}
