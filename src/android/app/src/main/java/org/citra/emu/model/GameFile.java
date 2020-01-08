package org.citra.emu.model;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.BitmapShader;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.graphics.PorterDuffXfermode;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Shader;
import java.nio.IntBuffer;
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
            mName = NativeLibrary.GetAppTitle(mPath);
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

    public String getPath() {
        return mPath;
    }

    public Bitmap getIcon(Context context) {
        if (mIcon == null) {
            int[] pixels = NativeLibrary.GetAppIcon(mPath);
            if (pixels == null || pixels.length == 0) {
                mIcon = BitmapFactory.decodeResource(context.getResources(), R.drawable.no_banner);
            } else {
                mIcon = Bitmap.createBitmap(48, 48, Bitmap.Config.RGB_565);
                mIcon.copyPixelsFromBuffer(IntBuffer.wrap(pixels));
            }

            // rounded corners
            float radius = 5.0f;
            Rect rect = new Rect(0, 0, mIcon.getWidth(), mIcon.getHeight());
            Bitmap output =
                Bitmap.createBitmap(mIcon.getWidth(), mIcon.getHeight(), Bitmap.Config.ARGB_8888);
            Canvas canvas = new Canvas(output);
            BitmapShader shader =
                new BitmapShader(mIcon, Shader.TileMode.CLAMP, Shader.TileMode.CLAMP);
            Paint paint = new Paint();
            paint.setAntiAlias(true);
            paint.setShader(shader);
            // rect contains the bounds of the shape
            // radius is the radius in pixels of the rounded corners
            // paint contains the shader that will texture the shape
            canvas.drawRoundRect(new RectF(rect), radius, radius, paint);
            mIcon = output;
        }
        return mIcon;
    }

    public int getRegion() {
        if (mRegion == NativeLibrary.GameRegion.Invalid) {
            mRegion = NativeLibrary.GetAppRegion(mPath);
        }
        return mRegion;
    }
}
