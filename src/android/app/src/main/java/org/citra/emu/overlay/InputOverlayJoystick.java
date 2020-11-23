package org.citra.emu.overlay;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import org.citra.emu.NativeLibrary;

public final class InputOverlayJoystick {
    private final int[] mAxisIDs = {0, 0};
    private final float[] mAxises = {0f, 0f};

    private Bitmap mOuterBitmap;
    private Bitmap mDefaultInnerBitmap;
    private Bitmap mPressedInnerBitmap;
    private int mPreviousTouchX, mPreviousTouchY;
    private int mPointerId;
    private Rect mOuterBounds;
    private Rect mInnerBounds;
    private Rect mBounds;

    public InputOverlayJoystick(Bitmap bitmapOuter, Bitmap innerDefault, Bitmap innerPressed,
                                Rect rectOuter, Rect rectInner, int buttonId) {
        mPointerId = -1;
        mOuterBitmap = bitmapOuter;
        mDefaultInnerBitmap = innerDefault;
        mPressedInnerBitmap = innerPressed;

        mAxisIDs[0] = buttonId + 0;
        mAxisIDs[1] = buttonId + 1;

        mBounds = rectOuter;
        mOuterBounds = new Rect(rectOuter);
        mInnerBounds = rectInner;
        updateInnerBounds();
    }

    public void onConfigureBegin(int x, int y) {
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onConfigureMove(int x, int y) {
        mBounds.offset(x - mPreviousTouchX, y - mPreviousTouchY);
        mOuterBounds.offset(x - mPreviousTouchX, y - mPreviousTouchY);
        mInnerBounds.offset(x - mPreviousTouchX, y - mPreviousTouchY);
        updateInnerBounds();
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onDraw(Canvas canvas, Paint paint) {
        canvas.drawBitmap(mOuterBitmap, null, mOuterBounds, paint);
        canvas.drawBitmap(getCurrentBitmap(), null, mInnerBounds, paint);
    }

    public void onPointerDown(int id, float x, float y) {
        if (InputOverlay.sJoystickRelative) {
            // reCenter
            mOuterBounds.offset((int)x - mBounds.centerX(), (int)y - mBounds.centerY());
        }
        mPointerId = id;

        setJoystickState(x, y);
    }

    public void onPointerMove(int id, float x, float y) {
        setJoystickState(x, y);
    }

    public void onPointerUp(int id, float x, float y) {
        mOuterBounds.set(mBounds);
        mPointerId = -1;

        setJoystickState(x, y);
    }

    private void setJoystickState(float touchX, float touchY) {
        if (mPointerId != -1) {
            float maxY = mOuterBounds.bottom;
            float maxX = mOuterBounds.right;
            touchX -= mOuterBounds.centerX();
            maxX -= mOuterBounds.centerX();
            touchY -= mOuterBounds.centerY();
            maxY -= mOuterBounds.centerY();
            mAxises[0] = touchX / maxX;
            mAxises[1] = touchY / maxY;
        } else {
            mAxises[0] = mAxises[1] = 0.0f;
        }

        // Clamp the circle pad input to a circle
        float radius = (float) Math.sqrt(mAxises[0] *  mAxises[0] + mAxises[1] * mAxises[1]);
        if(radius > 1.0f)
        {
            float angle = (float) Math.atan2(mAxises[1], mAxises[0]);
            mAxises[0] = ((float)Math.cos(angle) * 1.0f);
            mAxises[1] = ((float)Math.sin(angle) * 1.0f);
        }

        updateInnerBounds();

        NativeLibrary.InputEvent(mAxisIDs[0], mAxises[0]);
        NativeLibrary.InputEvent(mAxisIDs[1], mAxises[1]);
    }

    public int getButtonId() {
        return mAxisIDs[0];
    }

    public int getPointerId() {
        return mPointerId;
    }

    public Rect getBounds() {
        return mBounds;
    }

    private Bitmap getCurrentBitmap() {
        return mPointerId != -1 ? mPressedInnerBitmap : mDefaultInnerBitmap;
    }

    private void updateInnerBounds() {
        float centerX = mOuterBounds.centerX();
        float centerY = mOuterBounds.centerY();
        float halfWidth = mOuterBounds.width() / 2.0f;
        float halfHeight = mOuterBounds.height() / 2.0f;

        float x = centerX + mAxises[0] * halfWidth;
        float y = centerY + mAxises[1] * halfHeight;

        if (x > centerX + halfWidth)
            x = centerX + halfWidth;
        if (x < centerX - halfWidth)
            x = centerX - halfWidth;

        if (y > centerY + halfHeight)
            y = centerY + halfHeight;
        if (y < centerY - halfHeight)
            y = centerY - halfHeight;

        mInnerBounds.offsetTo((int)(x - mInnerBounds.width() / 2.0f), (int)(y - mInnerBounds.height() / 2.0f));
    }
}
