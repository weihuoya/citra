package org.citra.emu.utils;

import android.content.Context;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.AsyncTask;
import android.os.Environment;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Collection;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.overlay.InputOverlay;

public final class DirectoryInitialization {
    private static DirectoryInitializationState sDirectoryState;
    private static String mUserPath;

    private static class InitTask extends AsyncTask<Context, Void, Void> {
        @Override
        protected Void doInBackground(Context... contexts) {
            initializeExternalStorage(contexts[0]);
            return null;
        }
    }

    public static void start(Context context) {
        if (sDirectoryState != DirectoryInitializationState.DIRECTORIES_INITIALIZED) {
            if (PermissionsHandler.hasWriteAccess(context)) {
                if (setUserDirectory()) {
                    new InitTask().execute(context);
                } else {
                    sDirectoryState = DirectoryInitializationState.CANT_FIND_EXTERNAL_STORAGE;
                }
            } else {
                sDirectoryState = DirectoryInitializationState.EXTERNAL_STORAGE_PERMISSION_NEEDED;
            }
        }
    }

    public static boolean isInitialized() {
        return sDirectoryState == DirectoryInitializationState.DIRECTORIES_INITIALIZED;
    }

    private static boolean setUserDirectory() {
        if (Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState())) {
            File externalPath = Environment.getExternalStorageDirectory();
            if (externalPath != null) {
                File userPath = new File(externalPath, "citra-emu");
                if (!userPath.isDirectory() && !userPath.mkdir()) {
                    return false;
                }
                mUserPath = userPath.getPath();
                NativeLibrary.SetUserPath(mUserPath);
                return true;
            }
        }
        return false;
    }

    private static void initializeExternalStorage(Context context) {
        File shaders = new File(getShadersDirectory());
        File sysdata = new File(getSysDataDirectory());
        File cheats = new File(getCheatsDirectory());
        File sdmc = new File(getSDMCDirectory());
        File theme = new File(getThemeDirectory());
        copyAssetFolder("shaders", shaders, false, context);
        copyAssetFolder("sysdata", sysdata, false, context);
        copyAssetFolder("sdmc", sdmc, false, context);
        if (theme.exists() || theme.mkdir()) {
            saveInputOverlay(context);
        }
        sDirectoryState = DirectoryInitializationState.DIRECTORIES_INITIALIZED;
    }

    private static void deleteDirectoryRecursively(File file) {
        if (file.isDirectory()) {
            for (File child : file.listFiles())
                deleteDirectoryRecursively(child);
        }
        file.delete();
    }

    public static String getUserDirectory() {
        return mUserPath;
    }

    public static File getCheatFile(String programId) {
        File cheatsPath = new File(mUserPath, "cheats");
        if (!cheatsPath.isDirectory() && !cheatsPath.mkdir()) {
            return null;
        }
        return new File(cheatsPath, programId + ".txt");
    }

    public static String getConfigFile() {
        return getUserDirectory() + File.separator + "config" + File.separator + "config-mmj.ini";
    }

    public static String getShadersDirectory() {
        return getUserDirectory() + File.separator + "shaders";
    }

    public static String getSysDataDirectory() {
        return getUserDirectory() + File.separator + "sysdata";
    }

    public static String getCheatsDirectory() {
        return getUserDirectory() + File.separator + "cheats";
    }

    public static String getAmiiboDirectory() {
        return getUserDirectory() + File.separator + "amiibo";
    }

    public static String getThemeDirectory() {
        return getUserDirectory() + File.separator + "theme";
    }

    public static String getSDMCDirectory() {
        return getUserDirectory() + File.separator + "sdmc";
    }

    public static String getSystemTitleDirectory() {
        return getUserDirectory() + "/nand/00000000000000000000000000000000/title";
    }

    public static String getSystemApplicationDirectory() {
        return getUserDirectory() + "/nand/00000000000000000000000000000000/title/00040010";
    }

    public static String getSystemAppletDirectory() {
        return getUserDirectory() + "/nand/00000000000000000000000000000000/title/00040030";
    }

    public static String getApplicationDirectory() {
        return getSDMCDirectory() + "/Nintendo 3DS/00000000000000000000000000000000/00000000000000000000000000000000/title";
    }

    public static File getShaderCacheFile(String programId) {
        return new File(getUserDirectory() + "/Cache/" + programId + ".cache");
    }

    public static void saveInputOverlay(Context context) {
        final int[] inputIds = InputOverlay.ResIds;
        final String[] inputNames = InputOverlay.ResNames;
        String path = getThemeDirectory() + "/default.zip";
        File file = new File(path);
        if (file.exists()) {
            return;
        }
        try {
            FileOutputStream outputStream = new FileOutputStream(file);
            ZipOutputStream zipOut = new ZipOutputStream(new BufferedOutputStream(outputStream));
            for (int i = 0; i < inputIds.length; ++i) {
                ZipEntry entry = new ZipEntry(inputNames[i]);
                zipOut.putNextEntry(entry);
                Bitmap bitmap = BitmapFactory.decodeResource(context.getResources(), inputIds[i]);
                Bitmap.CompressFormat format = Bitmap.CompressFormat.PNG;
                if (inputNames[i].endsWith(".jpg")) {
                    format = Bitmap.CompressFormat.JPEG;
                }
                bitmap.compress(format, 90, zipOut);
            }
            zipOut.close();
        } catch (IOException e) {
            // ignore
        }
    }

    public static Map<Integer, Bitmap> loadInputOverlay(Context context, String theme) {
        final int[] inputIds = InputOverlay.ResIds;
        final String[] inputNames = InputOverlay.ResNames;
        final String themePath = getThemeDirectory();
        Map<Integer, Bitmap> inputs = new HashMap<>();
        File file = new File(themePath + "/" + theme + ".zip");
        if (!file.exists()) {
            file = new File(themePath + "/default.zip");
        }

        // default bitmaps
        for (int id : inputIds) {
            Bitmap bitmap = BitmapFactory.decodeResource(context.getResources(), id);
            inputs.put(id, bitmap);
        }

        if (!file.exists()) {
            return inputs;
        }

        try {
            FileInputStream inputStream = new FileInputStream(file);
            ZipInputStream zipIn = new ZipInputStream(new BufferedInputStream(inputStream));

            ZipEntry entry;
            while ((entry = zipIn.getNextEntry()) != null) {
                for (int i = 0; i < inputNames.length; ++i) {
                    if (!entry.isDirectory() && inputNames[i].equals(entry.getName())) {
                        Bitmap bitmap = BitmapFactory.decodeStream(zipIn);
                        inputs.put(inputIds[i], bitmap);
                    }
                }
            }
            zipIn.close();
        } catch (IOException e) {
            // ignore
        }

        return inputs;
    }

    private static void copyAsset(String asset, File output, Boolean overwrite, Context context) {
        try {
            if (!output.exists() || overwrite) {
                InputStream in = context.getAssets().open(asset);
                OutputStream out = new FileOutputStream(output);
                copyFile(in, out);
                in.close();
                out.close();
            }
        } catch (IOException e) {
        }
    }

    private static void copyAssetFolder(String assetFolder, File outputFolder, Boolean overwrite,
                                        Context context) {
        try {
            boolean createdFolder = false;
            for (String file : context.getAssets().list(assetFolder)) {
                if (!createdFolder) {
                    outputFolder.mkdir();
                    createdFolder = true;
                }
                copyAssetFolder(assetFolder + File.separator + file, new File(outputFolder, file),
                                overwrite, context);
                copyAsset(assetFolder + File.separator + file, new File(outputFolder, file),
                          overwrite, context);
            }
        } catch (IOException e) {
        }
    }

    public static void copyFile(String from, String to) {
        try {
            InputStream in = new FileInputStream(from);
            OutputStream out = new FileOutputStream(to);
            copyFile(in, out);
        } catch (IOException e) {
        }
    }

    private static void copyFile(InputStream in, OutputStream out) throws IOException {
        byte[] buffer = new byte[4096];
        int read;
        while ((read = in.read(buffer)) != -1) {
            out.write(buffer, 0, read);
        }
    }

    public enum DirectoryInitializationState {
        DIRECTORIES_INITIALIZED,
        EXTERNAL_STORAGE_PERMISSION_NEEDED,
        CANT_FIND_EXTERNAL_STORAGE
    }
}
