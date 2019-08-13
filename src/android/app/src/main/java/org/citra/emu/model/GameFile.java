package org.citra.emu.model;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;

import java.nio.IntBuffer;

public final class GameFile {
    private String mPath;
    private String mName;
    private String mInfo;

    public GameFile(String path) {
        mPath = path;
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
}
