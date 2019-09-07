package org.citra.emu.model;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import java.nio.IntBuffer;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;

public final class GameFile {
    private String mId;
    private String mPath;
    private String mName;
    private String mInfo;
    private Bitmap mIcon;
    private int mRegion = NativeLibrary.GameRegion.Invalid;

    public GameFile(String path) {
        mPath = path;
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
