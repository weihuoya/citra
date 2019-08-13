package org.citra.emu;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.Log;
import android.view.Surface;

import org.citra.emu.ui.EmulationActivity;

import java.io.File;
import java.io.FileOutputStream;

public final class NativeLibrary {
    public static final String TouchScreenDevice = "touchscreen";

    public static Context getEmulationContext() {
        return EmulationActivity.get();
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
                Log.i("zhangwei", "saveImageToFile error: " + e.getMessage());
            }
        }
    }


    public static native String GetAppTitle(String path);

    public static native void SaveScreenShot();

    public static native void SetUserPath(String path);

    public static native boolean InputEvent(String device, int button, float value);

    public static native void TouchEvent(int action, float x, float y);

    public static native boolean IsRunning();

    public static native void SurfaceChanged(Surface surf);

    public static native void SurfaceDestroyed();

    public static native void Run(String path);

    public static native void ResumeEmulation();

    public static native void PauseEmulation();

    public static native void StopEmulation();

    public static native int[] getRunningSettings();

    public static native void setRunningSettings(int[] settings);

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
}
