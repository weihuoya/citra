package org.citra.emu.utils;

import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Environment;
import android.util.Log;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.iconcache.IconCache;
import org.citra.emu.model.GameInfo;
import org.citra.emu.overlay.InputOverlay;

public final class CitraDirectory {
    private static int sInitState;
    private static String mUserPath;
    private static IconCache mIconCache;
    private static Bitmap mDefaultIcon;
    private static final Dictionary<String, String> mTitleDB = new Hashtable<>();

    private static final int INIT_FAILED = -1;
    private static final int INIT_UNKNOWN = 0;
    private static final int INIT_LEGACY = 1;
    private static final int INIT_SAF = 2;


    private static class InitTask extends AsyncTask<Context, Void, Void> {
        @Override
        protected Void doInBackground(Context... contexts) {
            initializeExternalStorage(contexts[0]);
            return null;
        }
    }

    public static void start(Context context) {
        if (sInitState == INIT_LEGACY || sInitState == INIT_SAF) {
            return;
        }

        File externalPath = null;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q && !Environment.isExternalStorageLegacy()) {
            sInitState = INIT_SAF;
            externalPath = context.getExternalFilesDir(null);
        } else if (PermissionsHandler.hasWriteAccess(context)) {
            if (Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState())) {
                sInitState = INIT_LEGACY;
                externalPath = Environment.getExternalStorageDirectory();
            } else {
                sInitState = INIT_FAILED;
            }
        } else {
            sInitState = INIT_FAILED;
        }

        if (externalPath != null) {
            File userPath = new File(externalPath, "citra-emu");
            if (!userPath.isDirectory() && !userPath.mkdir()) {
                sInitState = INIT_FAILED;
            } else {
                mUserPath = userPath.getPath();
                mDefaultIcon = BitmapFactory.decodeResource(context.getResources(), R.drawable.no_banner);
                mIconCache = new IconCache(context);
                loadTitleDB(context.getAssets());
                NativeLibrary.SetUserPath(mUserPath);
                new InitTask().execute(context);
            }
        }
    }

    public static Bitmap getDefaultIcon() {
        return mDefaultIcon;
    }

    public static GameInfo loadGameInfo(String path) {
        GameInfo info = mIconCache.getEntry(path);
        if (info == null) {
            info = new GameInfo();
            info.path = path;
            info.id = NativeLibrary.GetAppId(path);
            info.name = NativeLibrary.GetAppTitle(path);
            info.region = NativeLibrary.GetAppRegion(path);
            info.icon = NativeLibrary.GetAppIcon(path);
            if (!isExternalStorageLegacy()) {
                int idx = info.name.indexOf("%2F");
                if (idx > 0) {
                    info.name = info.name.substring(idx + 3);
                }
            }
            mIconCache.addIconToDB(info);
        }
        // get name from title db
        String name = mTitleDB.get(info.id);
        if (name == null && info.id.length() > 8) {
            String id = "00040000" + info.id.substring(8);
            if (!info.id.equals(id)) {
                // DLC? try again
                name = mTitleDB.get(id);
            }
        }
        if (name != null) {
            info.name = name;
        }
        return info;
    }

    private static void loadTitleDB(AssetManager mgr) {
        String lan = Locale.getDefault().getLanguage();
        if (lan.equals("zh")) {
            lan = lan + "_" + Locale.getDefault().getCountry();
        }
        String asset = "3dstdb-" + lan + ".txt";
        try {
            BufferedReader input = new BufferedReader(new InputStreamReader(mgr.open(asset)));
            while (input.ready()) {
                String line = input.readLine();
                int sep = line.indexOf('=');
                if (sep > 0 && sep < line.length() - 1) {
                    String key = line.substring(0, sep).trim();
                    String value = line.substring(sep + 1).trim();
                    if (!key.isEmpty() && !value.isEmpty()) {
                        mTitleDB.put(key, value);
                    }
                }
            }
        } catch (IOException e) {
            Log.e("citra", "loadTitleDB error", e);
        }
    }

    public static boolean isInitialized() {
        return sInitState == INIT_LEGACY || sInitState == INIT_SAF;
    }

    public static boolean isExternalStorageLegacy() {
        return sInitState == INIT_LEGACY;
    }

    public static void clearIconCache() {
        mIconCache.clearCache();
    }

    private static void initializeExternalStorage(Context context) {
        File shaders = new File(getShadersDirectory());
        File sysdata = new File(getSysDataDirectory());
        File config = new File(getConfigDirectory());
        File nand = new File(getNandDirectory());
        File sdmc = new File(getSDMCDirectory());
        File theme = new File(getThemeDirectory());
        copyAssetFolder("shaders", shaders, false, context);
        copyAssetFolder("sysdata", sysdata, false, context);
        copyAssetFolder("config", config, false, context);
        copyAssetFolder("nand", nand, false, context);
        copyAssetFolder("sdmc", sdmc, false, context);
        if (theme.exists() || theme.mkdir()) {
            saveInputOverlay(context);
        }
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
        return getConfigDirectory() + File.separator + "config-mmj.ini";
    }

    public static File getGameListFile() {
        String path = getUserDirectory() + File.separator + "gamelist.bin";
        return new File(path);
    }

    public static String getConfigDirectory() {
        return getUserDirectory() + File.separator + "config";
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

    public static String getNandDirectory() {
        return getUserDirectory() + File.separator + "nand";
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
            Log.e("citra", "saveInputOverlay error", e);
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
            Log.e("citra", "loadInputOverlay error", e);
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
            Log.e("citra", "copyAsset error: " + asset, e);
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
            Log.e("citra", "copyAssetFolder error: " + assetFolder, e);
        }
    }

    public static void copyFile(String from, String to) {
        try {
            InputStream in = new FileInputStream(from);
            OutputStream out = new FileOutputStream(to);
            copyFile(in, out);
        } catch (IOException e) {
            Log.e("citra", "copyFile error from: " + from + ", to: " + to, e);
        }
    }

    private static void copyFile(InputStream in, OutputStream out) throws IOException {
        byte[] buffer = new byte[4096];
        int read;
        while ((read = in.read(buffer)) != -1) {
            out.write(buffer, 0, read);
        }
    }

    public static List<String> readAllLines(InputStream input) {
        byte[] buffer = new byte[1024*8];
        List<String> lines = new ArrayList<>();
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        try {
            while (true) {
                int size = input.read(buffer, 0, buffer.length);
                if (size > 0) {
                    int i = 0;
                    int offset = 0;
                    while (i < size) {
                        if (buffer[i++] == '\n') {
                            // new line start, save previous line
                            output.write(buffer, offset, i - offset - 1);
                            lines.add(output.toString());
                            output.reset();
                            offset = i;
                        }
                    }
                    if (offset < size) {
                        // save remain bytes
                        output.write(buffer, offset, size - offset);
                    }
                } else {
                    if (output.size() > 0) {
                        lines.add(output.toString());
                    }
                    break;
                }
            }
            input.close();
        } catch (IOException e) {
            Log.e("citra", "readAllLines error", e);
            lines.clear();
        }
        return lines;
    }
}
