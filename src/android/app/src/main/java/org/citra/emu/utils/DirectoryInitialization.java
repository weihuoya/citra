package org.citra.emu.utils;

import android.content.Context;
import android.content.Intent;
import android.os.Environment;
import android.support.v4.content.LocalBroadcastManager;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.concurrent.atomic.AtomicBoolean;
import org.citra.emu.NativeLibrary;

public final class DirectoryInitialization {
    public static final String BROADCAST_ACTION = "org.citra.emu.DIRECTORY_INITIALIZATION";

    public static final String EXTRA_STATE = "DirectoryState";
    private static volatile DirectoryInitializationState sDirectoryState;
    private static String mUserPath;
    private static AtomicBoolean mIsRunning = new AtomicBoolean(false);

    public static void start(Context context) {
        // Can take a few seconds to run, so don't block UI thread.
        // noinspection TrivialFunctionalExpressionUsage
        ((Runnable)() -> init(context)).run();
    }

    private static void init(Context context) {
        if (!mIsRunning.compareAndSet(false, true))
            return;

        if (sDirectoryState != DirectoryInitializationState.DIRECTORIES_INITIALIZED) {
            if (PermissionsHandler.hasWriteAccess(context)) {
                if (setUserDirectory()) {
                    initializeExternalStorage(context);
                    sDirectoryState = DirectoryInitializationState.DIRECTORIES_INITIALIZED;
                } else {
                    sDirectoryState = DirectoryInitializationState.CANT_FIND_EXTERNAL_STORAGE;
                }
            } else {
                sDirectoryState = DirectoryInitializationState.EXTERNAL_STORAGE_PERMISSION_NEEDED;
            }
        }

        mIsRunning.set(false);
        sendBroadcastState(sDirectoryState, context);
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
        copyAssetFolder("shaders", shaders, false, context);
        copyAssetFolder("sysdata", sysdata, false, context);
        copyAssetFolder("cheats", cheats, false, context);
        copyAssetFolder("sdmc", sdmc, false, context);
    }

    private static void deleteDirectoryRecursively(File file) {
        if (file.isDirectory()) {
            for (File child : file.listFiles())
                deleteDirectoryRecursively(child);
        }
        file.delete();
    }

    public static boolean isReady() {
        return sDirectoryState == DirectoryInitializationState.DIRECTORIES_INITIALIZED;
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

    public static String getSDMCDirectory() {
        return getUserDirectory() + File.separator + "sdmc";
    }

    private static void sendBroadcastState(DirectoryInitializationState state, Context context) {
        Intent localIntent = new Intent(BROADCAST_ACTION).putExtra(EXTRA_STATE, state);
        LocalBroadcastManager.getInstance(context).sendBroadcast(localIntent);
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
