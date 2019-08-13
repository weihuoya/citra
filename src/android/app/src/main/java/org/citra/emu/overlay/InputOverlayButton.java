package org.citra.emu.overlay;

import android.graphics.Canvas;
import android.graphics.Rect;
import android.graphics.drawable.BitmapDrawable;

import org.citra.emu.NativeLibrary;

public final class InputOverlayButton {
    private BitmapDrawable mDefaultBitmap;
    private BitmapDrawable mPressedBitmap;
    private int mPreviousTouchX, mPreviousTouchY;
    private int mControlPositionX, mControlPositionY;
    private int mPointerId;
    private int mButtonId;

    public InputOverlayButton(BitmapDrawable defaultBitmap, BitmapDrawable pressedBitmap, int buttonId) {
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
        Rect bounds = getBounds();
        mControlPositionX += x - mPreviousTouchX;
        mControlPositionY += y - mPreviousTouchY;
        setBounds(new Rect(mControlPositionX, mControlPositionY,
                mControlPositionX + bounds.width(), mControlPositionY + bounds.height()));
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onDraw(Canvas canvas) {
        getCurrentBitmapDrawable().draw(canvas);
    }

    public void onPointerDown(int id, float x, float y) {
        mPointerId = id;
        NativeLibrary.InputEvent(NativeLibrary.TouchScreenDevice, mButtonId, NativeLibrary.ButtonState.PRESSED);
    }

    public void onPointerMove(int id, float x, float y) {

    }

    public void onPointerUp(int id, float x, float y) {
        mPointerId = -1;
        NativeLibrary.InputEvent(NativeLibrary.TouchScreenDevice, mButtonId, NativeLibrary.ButtonState.RELEASED);
    }

    private BitmapDrawable getCurrentBitmapDrawable() {
        return mPointerId != -1 ? mPressedBitmap : mDefaultBitmap;
    }

    public void setPosition(int x, int y) {
        mControlPositionX = x;
        mControlPositionY = y;
    }

    public void setAlpha(int value) {
        mDefaultBitmap.setAlpha(value);
    }

    public int getButtonId() {
        return mButtonId;
    }

    public int getPointerId() {
        return mPointerId;
    }

    public Rect getBounds() {
        return mDefaultBitmap.getBounds();
    }

    public void setBounds(Rect bounds) {
        mDefaultBitmap.setBounds(bounds);
        mPressedBitmap.setBounds(bounds);
    }
}
