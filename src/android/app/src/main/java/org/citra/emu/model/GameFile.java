package org.citra.emu.model;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.BitmapShader;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Shader;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.IntBuffer;
import java.util.Dictionary;
import java.util.Hashtable;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;

public final class GameFile {
    private String mId;
    private String mPath;
    private String mName;
    private String mInfo;
    private Bitmap mIcon;
    private boolean mInstalled;
    private int mRegion = NativeLibrary.GameRegion.Invalid;
    private static Dictionary<String, String> sTitleDB = new Hashtable<>();

    public GameFile(String path) {
        mPath = path;
        mInstalled = false;
    }

    public GameFile(String path, boolean installed) {
        mPath = path;
        mInstalled = installed;
    }

    public String getId() {
        if (mId == null) {
            mId = NativeLibrary.GetAppId(mPath);
        }
        return mId;
    }

    public String getName() {
        if (mName == null) {
            mName = sTitleDB.get(getId());
            if (mName == null) {
                mName = NativeLibrary.GetAppTitle(mPath);
            }
        }
        return mName;
    }

    public String getInfo() {
        if (mInfo == null) {
            mInfo = mPath.substring(mPath.lastIndexOf('/') + 1);
        }
        return mInfo;
    }

    public boolean isInstalled() {
        return mInstalled;
    }

    public boolean isInstalledApp() {
        return isInstalled() && getPath().contains("/title/00040000/");
    }

    public boolean isInstalledDLC() {
        return isInstalled() && !getPath().contains("/title/00040000/");
    }

    public String getPath() {
        return mPath;
    }

    public int getRegion() {
        if (mRegion == NativeLibrary.GameRegion.Invalid) {
            mRegion = NativeLibrary.GetAppRegion(mPath);
        }
        return mRegion;
    }

    public Bitmap getIcon(Context context) {
        if (mIcon == null) {
            int[] pixels = NativeLibrary.GetAppIcon(mPath);
            if (pixels == null || pixels.length == 0) {
                mIcon = BitmapFactory.decodeResource(context.getResources(), R.drawable.no_banner);
            } else {
                Bitmap icon = Bitmap.createBitmap(48, 48, Bitmap.Config.RGB_565);
                icon.copyPixelsFromBuffer(IntBuffer.wrap(pixels));
                mIcon = roundBitmap(resizeBitmap(icon, 96, 96), 10);
            }
        }
        return mIcon;
    }

    public static Bitmap roundBitmap(Bitmap icon, float radius) {
        Rect rect = new Rect(0, 0, icon.getWidth(), icon.getHeight());
        Bitmap output =
                Bitmap.createBitmap(icon.getWidth(), icon.getHeight(), Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(output);
        BitmapShader shader =
                new BitmapShader(icon, Shader.TileMode.CLAMP, Shader.TileMode.CLAMP);
        Paint paint = new Paint();
        paint.setAntiAlias(true);
        paint.setShader(shader);
        // rect contains the bounds of the shape
        // radius is the radius in pixels of the rounded corners
        // paint contains the shader that will texture the shape
        canvas.drawRoundRect(new RectF(rect), radius, radius, paint);
        return output;
    }

    public static Bitmap resizeBitmap(Bitmap icon, int newWidth, int newHeight) {
        int width = icon.getWidth();
        int height = icon.getHeight();
        float scaleWidth = newWidth / (float) width;
        float scaleHeight = newHeight / (float) height;
        Matrix matrix = new Matrix();
        matrix.postScale(scaleWidth, scaleHeight);
        return Bitmap.createBitmap(icon, 0, 0, width, height, matrix, true);
    }

    public static void loadTitleDB(InputStream db) {
        BufferedReader input = new BufferedReader(new InputStreamReader(db));
        try {
            while (input.ready()) {
                String line = input.readLine();
                int sep = line.indexOf('=');
                if (sep > 0 && sep < line.length() - 1) {
                    String key = line.substring(0, sep).trim();
                    String value = line.substring(sep + 1).trim();
                    if (!key.isEmpty() && !value.isEmpty()) {
                        sTitleDB.put(key, value);
                    }
                }
            }
        } catch (IOException e) {

        }
    }
}
