package org.citra.emu.model;

import android.graphics.Bitmap;
import android.graphics.BitmapShader;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Shader;

import java.net.URLDecoder;
import java.nio.ByteBuffer;

import org.citra.emu.utils.CitraDirectory;

public final class GameFile {
    private final String mPath;
    private final boolean mInstalled;
    private GameInfo mInfo;
    private Bitmap mIcon;

    public GameFile(String path) {
        mPath = path;
        mInstalled = false;
    }

    public GameFile(String path, boolean installed) {
        mPath = path;
        mInstalled = installed;
    }

    public String getId() {
        init();
        return mInfo.id;
    }

    public String getName() {
        init();
        return mInfo.name;
    }

    public String getInfo() {
        String uri = mPath;
        if (uri.startsWith("content://")) {
            try {
                uri = URLDecoder.decode(uri, "utf-8");
            } catch (Exception e) {
                // ignore
            }
        }
        return uri.substring(uri.lastIndexOf('/') + 1);
    }

    public boolean isInstalled() {
        return mInstalled;
    }

    public boolean isInstalledApp() {
        return isInstalled() && isAppPath(getPath());
    }

    public boolean isInstalledDLC() {
        return isInstalled() && !isAppPath(getPath());
    }

    public String getPath() {
        return mPath;
    }

    public int getRegion() {
        return mInfo.region;
    }

    public Bitmap getIcon() {
        if (mIcon == null) {
            init();
            if (mInfo.icon == null || mInfo.icon.length == 0) {
                mIcon = CitraDirectory.getDefaultIcon();
            } else {
                Bitmap icon = Bitmap.createBitmap(48, 48, Bitmap.Config.RGB_565);
                icon.copyPixelsFromBuffer(ByteBuffer.wrap(mInfo.icon));
                mIcon = roundBitmap(resizeBitmap(icon, 96, 96), 10);
            }
        }
        return mIcon;
    }

    public void init() {
        if (mInfo == null) {
            mInfo = CitraDirectory.loadGameInfo(mPath);
        }
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

    public static boolean isAppPath(String path) {
        return path.contains("/title/00040000/") || path.contains("/title/00040010/") || path.contains("/title/00040030/");
    }
}
