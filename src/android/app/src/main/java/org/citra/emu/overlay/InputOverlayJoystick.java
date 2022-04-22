package org.citra.emu.overlay;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import org.citra.emu.NativeLibrary;

public final class InputOverlayJoystick implements InputOverlay.InputObject {
    private final int[] mAxisIDs = {0, 0};
    private final double[] mAxises = {0, 0};

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
            double maxY = mOuterBounds.bottom;
            double maxX = mOuterBounds.right;
            touchX -= mOuterBounds.centerX();
            maxX -= mOuterBounds.centerX();
            touchY -= mOuterBounds.centerY();
            maxY -= mOuterBounds.centerY();
            mAxises[0] = touchX / maxX;
            mAxises[1] = touchY / maxY;
        } else {
            mAxises[0] = mAxises[1] = 0;
        }

        // Clamp the circle pad input to a circle
        double radius = Math.sqrt(mAxises[0] *  mAxises[0] + mAxises[1] * mAxises[1]);
        if(radius > 1.0) {
            double angle = Math.atan2(mAxises[1], mAxises[0]);
            mAxises[0] = Math.cos(angle);
            mAxises[1] = Math.sin(angle);
        }

        updateInnerBounds();

        double x = 1.0 - (1 - mAxises[0]) * (1 - mAxises[0]);
        double y = 1.0 - (1 - mAxises[1]) * (1 - mAxises[1]);
        NativeLibrary.InputEvent(mAxisIDs[0], (float)x);
        NativeLibrary.InputEvent(mAxisIDs[1], (float)y);
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

    public boolean isPressed() {
        return false;
    }

    private Bitmap getCurrentBitmap() {
        return mPointerId != -1 ? mPressedInnerBitmap : mDefaultInnerBitmap;
    }

    private void updateInnerBounds() {
        double centerX = mOuterBounds.centerX();
        double centerY = mOuterBounds.centerY();
        double halfWidth = mOuterBounds.width() / 2.0;
        double halfHeight = mOuterBounds.height() / 2.0;

        double x = centerX + mAxises[0] * halfWidth;
        double y = centerY + mAxises[1] * halfHeight;

        if (x > centerX + halfWidth)
            x = centerX + halfWidth;
        if (x < centerX - halfWidth)
            x = centerX - halfWidth;

        if (y > centerY + halfHeight)
            y = centerY + halfHeight;
        if (y < centerY - halfHeight)
            y = centerY - halfHeight;

        mInnerBounds.offsetTo((int)(x - mInnerBounds.width() / 2.0), (int)(y - mInnerBounds.height() / 2.0));
    }
}
