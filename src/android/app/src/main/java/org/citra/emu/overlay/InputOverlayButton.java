package org.citra.emu.overlay;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import org.citra.emu.NativeLibrary;

public final class InputOverlayButton {
    private Bitmap mDefaultBitmap;
    private Bitmap mPressedBitmap;
    private int mPreviousTouchX, mPreviousTouchY;
    private int mPointerId;
    private int mButtonId;
    private Rect mBounds;

    public InputOverlayButton(Bitmap defaultBitmap, Bitmap pressedBitmap,
                              int buttonId) {
        mPointerId = -1;
        mButtonId = buttonId;
        mDefaultBitmap = defaultBitmap;
        mPressedBitmap = pressedBitmap;
    }

    public void onConfigureBegin(int x, int y) {
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onConfigureMove(int x, int y) {
        mBounds.offset(x - mPreviousTouchX, y - mPreviousTouchY);
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onDraw(Canvas canvas, Paint paint) {
        canvas.drawBitmap(getCurrentBitmap(), null, mBounds, paint);
    }

    public void onPointerDown(int id, float x, float y) {
        mPointerId = id;
        NativeLibrary.InputEvent(mButtonId, NativeLibrary.ButtonState.PRESSED);
    }

    public void onPointerMove(int id, float x, float y) {}

    public void onPointerUp(int id, float x, float y) {
        mPointerId = -1;
        NativeLibrary.InputEvent(mButtonId, NativeLibrary.ButtonState.RELEASED);
    }

    private Bitmap getCurrentBitmap() {
        return mPointerId != -1 ? mPressedBitmap : mDefaultBitmap;
    }

    public int getButtonId() {
        return mButtonId;
    }

    public int getPointerId() {
        return mPointerId;
    }

    public Rect getBounds() {
        return mBounds;
    }

    public void setBounds(Rect bounds) {
        mBounds = bounds;
    }
}
