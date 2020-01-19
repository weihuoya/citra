package org.citra.emu;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;
import android.view.Surface;
import java.io.File;
import java.io.FileOutputStream;
import org.citra.emu.ui.EmulationActivity;
import org.citra.emu.ui.MainActivity;

public final class NativeLibrary {

    public static Context getMainContext() {
        return MainActivity.get();
    }

    public static Context getEmulationContext() {
        return EmulationActivity.get();
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

    public static void saveImageToFile(String path, int width, int height, int[] pixels) {
        if (pixels.length > 0 && width > 0 && height > 0) {
            File file = new File(path);
            Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            bitmap.setPixels(pixels, 0, width, 0, 0, width, height);
            try {
                FileOutputStream out = new FileOutputStream(file);
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                out.close();
            } catch (Exception e) {
                Log.i("citra", "saveImageToFile error: " + e.getMessage());
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

    public static void updateProgress(String name, int written, int total) {
        MainActivity activity = MainActivity.get();
        if (activity != null) {
            activity.runOnUiThread(() -> activity.updateProgress(name, written, total));
        }
    }

    public static void pickImage(int width, int height) {
        EmulationActivity activity = EmulationActivity.get();
        if (activity != null) {
            activity.runOnUiThread(() -> activity.pickImage(width, height));
        }
    }

    public static boolean isValidFile(String filename) {
        String name = filename.toLowerCase();
        return (name.endsWith(".cia") || name.endsWith(".cci") || name.endsWith(".3ds") ||
                name.endsWith(".cxi") || name.endsWith(".app"));
    }

    public static native String GetAppId(String path);

    public static native String GetAppTitle(String path);

    public static native int[] GetAppIcon(String path);

    public static native int GetAppRegion(String path);

    public static native boolean IsAppExecutable(String path);

    public static native void InstallCIA(String[] path);

    public static native void SetUserPath(String path);

    public static native void HandleImage(int[] pixels, int width, int height);

    // input overlay
    public static native void InputEvent(int button, float value);

    // touch screen
    public static native void TouchEvent(int action, int x, int y);

    // gamepad
    public static native boolean KeyEvent(int button, int action);
    public static native void MoveEvent(int axis, float value);

    // edit box
    public static native void KeyboardEvent(int type, String text);

    public static native boolean IsRunning();

    public static native void SurfaceChanged(Surface surf);

    public static native void SurfaceDestroyed();

    public static native void Run(String path);

    public static native void ResumeEmulation();

    public static native void PauseEmulation();

    public static native void StopEmulation();

    public static native int[] getRunningSettings();

    public static native void setRunningSettings(int[] settings);

    public static native void searchMemoryRegion(int region, int valueType, int searchType, int scanType, int value);

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
        public static final int BEGIN_TILT = 8;
        public static final int TILT = 16;
        public static final int END_TILT = 32;
    }

    /**
     * Game regions
     */
    public static final class GameRegion {
        public static final int Invalid = -1;
        public static final int Japan = 0;
        public static final int NorthAmerica = 1;
        public static final int Europe = 2;
        public static final int Australia = 3;
        public static final int China = 4;
        public static final int Korea = 5;
        public static final int Taiwan = 6;
    }
}
