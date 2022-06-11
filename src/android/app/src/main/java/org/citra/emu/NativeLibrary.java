package org.citra.emu;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Rect;
import android.net.Uri;
import android.os.ParcelFileDescriptor;
import android.util.Log;
import android.view.Surface;

import androidx.documentfile.provider.DocumentFile;

import java.io.File;
import java.io.FileOutputStream;
import java.util.HashMap;

import org.citra.emu.ui.EmulationActivity;
import org.citra.emu.ui.MainActivity;
import org.citra.emu.utils.NetPlayManager;
import org.citra.emu.utils.TranslateHelper;
import org.citra.emu.utils.WebRequestHandler;

public final class NativeLibrary {

    static {
        System.loadLibrary("main");
    }

    static HashMap<Integer, ParcelFileDescriptor> SafFileDescriptorMap = new HashMap<>();

    public static int SafOpen(String path, String mode) {
        Context context = getMainContext();
        if (context == null) {
            context = getEmulationContext();
            if (context == null) {
                return 0;
            }
        }

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < mode.length(); ++i) {
            switch (mode.charAt(i)) {
                case 'r':
                    sb.append('r');
                    break;
                case 'w':
                    sb.append('w');
                    break;
                case 'a':
                    sb.append('a');
                    break;
                default:
                    break;
            }
        }

        int fd = 0;
        try {
            final ParcelFileDescriptor file = context.getContentResolver()
                    .openFileDescriptor(Uri.parse(path), sb.toString());
            fd = file.getFd();
            SafFileDescriptorMap.put(fd, file);
        } catch (Exception e) {
            Log.e("citra", "SafOpen error: " + path, e);
        }
        return fd;
    }

    public static long SafLastModified(String path) {
        Context context = getMainContext();
        if (context == null) {
            context = getEmulationContext();
            if (context == null) {
                return 0;
            }
        }
        DocumentFile file = DocumentFile.fromTreeUri(context, Uri.parse(path));
        return file.lastModified();
    }

    public static int SafClose(int fd) {
        int ret = 0;
        final ParcelFileDescriptor file = SafFileDescriptorMap.get(fd);
        if (file != null) {
            try {
                file.close();
            } catch (Exception e) {
                Log.e("citra", "SafClose error: " + fd, e);
                ret = -1;
            }
            SafFileDescriptorMap.remove(fd);
        } else {
            ret = -1;
        }
        return ret;
    }

    static WebRequestHandler RemoteFileHandler = null;
    public static boolean RemoteFileOpen(String url) {
        if (RemoteFileHandler != null) {
            RemoteFileHandler.close();
        }
        RemoteFileHandler = WebRequestHandler.Create(url);
        return RemoteFileHandler != null;
    }

    public static byte[] RemoteFileData() {
        if (RemoteFileHandler != null) {
            return RemoteFileHandler.data();
        } else {
            return null;
        }
    }

    public static int RemoteFileRead() {
        if (RemoteFileHandler != null) {
            return RemoteFileHandler.read();
        } else {
            return 0;
        }
    }

    public static void RemoteFileClose() {
        if (RemoteFileHandler != null) {
            RemoteFileHandler.close();
            RemoteFileHandler = null;
        }
    }

    public interface OnScreenshotCompleteListener {
        void OnScreenshotComplete(int width, int height, int[] pixels);
    }

    public static Context getMainContext() {
        return MainActivity.get();
    }

    public static Context getEmulationContext() {
        return EmulationActivity.get();
    }

    public static AssetManager getAssetManager() {
        Context context = getMainContext();
        if (context == null) {
            context = getEmulationContext();
            if (context == null) {
                return null;
            }
        }
        return context.getAssets();
    }

    public static void notifyGameShudown() {
        Activity activity = EmulationActivity.get();
        if (activity != null) {
            activity.finish();
        }
    }

    public static void showMessageDialog(int type, String msg) {
        Context context = getMainContext();
        if (context == null) {
            context = getEmulationContext();
            if (context == null) {
                Log.e("citra", "showMessageDialog: " + msg);
                return;
            }
        }
        final Activity activity = (Activity)context;
        activity.runOnUiThread(() -> {
            AlertDialog.Builder builder = new AlertDialog.Builder(activity);
            builder.setTitle(R.string.error);
            builder.setMessage(msg);
            builder.show();
        });
    }

    public static void showInputBoxDialog(int maxLength, String error, String hint, String button0,
                                          String button1, String button2) {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            activity.runOnUiThread(
                () -> activity.showInputBoxDialog(maxLength, error, hint, button0, button1, button2));
        }
    }

    public static void showMiiSelectorDialog(boolean cancel, String title, String[] miis) {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            activity.runOnUiThread(() -> activity.showMiiSelectorDialog(cancel, title, miis));
        }
    }

    public static void handleNFCScanning(boolean isScanning) {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            activity.handleNFCScanning(isScanning);
        }
    }

    public static void saveImageToFile(String path, int[] pixels, int width, int height) {
        if (pixels.length > 0 && width > 0 && height > 0) {
            File file = new File(path);
            Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            bitmap.setPixels(pixels, 0, width, 0, 0, width, height);
            try {
                FileOutputStream out = new FileOutputStream(file);
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                out.close();
            } catch (Exception e) {
                Log.e("citra", "saveImageToFile error: " + path, e);
            }
        }
    }

    public static void loadImageFromFile(String path) {
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inPreferredConfig = Bitmap.Config.ARGB_8888;
        Bitmap bitmap = BitmapFactory.decodeFile(path, options);
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        int[] pixels = new int[width * height];
        bitmap.getPixels(pixels, 0, width, 0, 0, width, height);
        HandleImage(pixels, width, height);
    }

    public static void updateProgress(String name, long written, long total) {
        EmulationActivity activity1 = EmulationActivity.get();
        if (activity1 != null) {
            activity1.runOnUiThread(() -> activity1.updateProgress(name, written, total));
        } else {
            MainActivity activity2 = MainActivity.get();
            if (activity2 != null) {
                activity2.runOnUiThread(() -> activity2.updateProgress(name, written, total));
            }
        }
    }

    public static void pickImage(int width, int height) {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            activity.runOnUiThread(() -> activity.pickImage(width, height));
        }
    }

    public static boolean checkPermission(String permission) {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            return activity.checkPermission(permission);
        } else {
            return false;
        }
    }

    public static int getDisplayRotation() {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            return activity.getDisplayRotation();
        } else {
            return 0;
        }
    }

    public static int getSafeInsetLeft() {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            return activity.getSafeInsetLeft();
        } else {
            return 0;
        }
    }

    public static int getSafeInsetTop() {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            return activity.getSafeInsetTop();
        } else {
            return 0;
        }
    }

    public static int getSafeInsetRight() {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            return activity.getSafeInsetRight();
        } else {
            return 0;
        }
    }

    public static int getSafeInsetBottom() {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            return activity.getSafeInsetBottom();
        } else {
            return 0;
        }
    }

    public static float getScaleDensity() {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            return activity.getScaleDensity();
        } else {
            return 1;
        }
    }

    public static void setupTranslater(String key, String secret) {
        TranslateHelper.Initialize(key, secret);
    }

    public static boolean isValidFile(String filename) {
        String name = filename.toLowerCase();
        return (name.endsWith(".cci") || name.endsWith(".3ds") || name.endsWith(".elf") ||
                name.endsWith(".cxi") || name.endsWith(".app") || name.endsWith(".3dsx"));
    }

    public static void addNetPlayMessage(int type, String message) {
        NetPlayManager.AddNetPlayMessage(type, message);
    }

    public static void showRunningSetting() {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            activity.showRunningSetting();
        }
    }

    /**
     * meta info
     */
    public static native String GetAppId(String path);
    public static native String GetAppTitle(String path);
    public static native byte[] GetAppIcon(String path);
    public static native int GetAppRegion(String path);
    public static native boolean IsAppExecutable(String path);
    public static native boolean IsAppVisible(String path);
    public static native String GetBuildDate();
    public static native String GetDeviceIinfo(Surface surface);

    /**
     * emulation
     */
    public static native void SetUserPath(String path);
    public static native void loadAmiibo(String path);
    public static native void InstallCIA(String[] path);
    public static native void HandleImage(int[] pixels, int width, int height);
    public static native void Screenshot(OnScreenshotCompleteListener listener);

    /**
     * input
     */
    // input overlay
    public static native void InputEvent(int button, float value);
    // touch screen
    public static native void TouchEvent(int action, int x, int y);
    // gamepad
    public static native boolean KeyEvent(int button, int action);
    public static native void MoveEvent(int axis, float value);
    // edit box
    public static native void KeyboardEvent(int type, String text);

    /**
     * emulation
     */
    public static native boolean IsRunning();
    public static native void SurfaceChanged(Surface surface);
    public static native void SurfaceDestroyed();
    public static native void WindowChanged();
    public static native void DoFrame();
    public static native void Run(String path);
    public static native void ResumeEmulation();
    public static native void PauseEmulation();
    public static native void StopEmulation();

    /**
     * running settings
     */
    public static native int[] getRunningSettings();
    public static native void setRunningSettings(int[] settings);
    public static native void setCustomLayout(boolean isTopScreen, int left, int top, int right, int bottom);
    public static native Rect getCustomLayout(boolean isTopScreen);
    public static native void SetBackgroundImage(int[] pixels, int width, int height);
    public static native void ResetCamera();
    public static native void reloadCheatCode();

    /**
     * memory editor
     */
    public static native int[] searchMemory(int startAddr, int stopAddr, int valueType, int searchType, int scanType, int value);
    public static native int[] getSearchResults();
    public static native void resetSearchResults();
    public static native int[] loadPageTable();
    public static native byte[] loadPage(int index);
    public static native int readMemory(int addr, int valueType);
    public static native void writeMemory(int addr, int valueType, int value);

    /**
     * Button type for use in onTouchEvent
     */
    public static final class ButtonType {
        public static final int N3DS_BUTTON_A = 0;
        public static final int N3DS_BUTTON_B = 1;
        public static final int N3DS_BUTTON_X = 2;
        public static final int N3DS_BUTTON_Y = 3;

        public static final int N3DS_DPAD_UP = 4;
        public static final int N3DS_DPAD_DOWN = 5;
        public static final int N3DS_DPAD_LEFT = 6;
        public static final int N3DS_DPAD_RIGHT = 7;

        public static final int N3DS_BUTTON_L = 8;
        public static final int N3DS_BUTTON_R = 9;

        public static final int N3DS_BUTTON_START = 10;
        public static final int N3DS_BUTTON_SELECT = 11;
        public static final int N3DS_BUTTON_DEBUG = 12;
        public static final int N3DS_BUTTON_GPIO14 = 13;

        public static final int N3DS_BUTTON_ZL = 14;
        public static final int N3DS_BUTTON_ZR = 15;

        public static final int N3DS_BUTTON_HOME = 16;

        public static final int N3DS_CPAD_X = 17;
        public static final int N3DS_CPAD_Y = 18;
        public static final int N3DS_STICK_X = 19;
        public static final int N3DS_STICK_Y = 20;

        public static final int N3DS_TOUCH_X = 21;
        public static final int N3DS_TOUCH_Y = 22;
        public static final int N3DS_TOUCH_Z = 23;

        public static final int EMU_COMBO_KEY_1 = 101;
        public static final int EMU_COMBO_KEY_2 = 102;
        public static final int EMU_COMBO_KEY_3 = 103;
    }

    /**
     * Button states
     */
    public static final class ButtonState {
        public static final int RELEASED = 0;
        public static final int PRESSED = 1;
    }

    public static final class TouchEvent {
        public static final int TOUCH_PRESSED = 1;
        public static final int TOUCH_MOVED = 2;
        public static final int TOUCH_RELEASED = 4;
    }
}
