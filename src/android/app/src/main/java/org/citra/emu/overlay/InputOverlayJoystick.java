package org.citra.emu.overlay;

import android.graphics.Canvas;
import android.graphics.Rect;
import android.graphics.drawable.BitmapDrawable;
import org.citra.emu.NativeLibrary;

public final class InputOverlayJoystick {
    private final int[] mAxisIDs = {0, 0};
    private final float[] mAxises = {0f, 0f};

    private BitmapDrawable mOuterBitmap;
    private BitmapDrawable mDefaultInnerBitmap;
    private BitmapDrawable mPressedInnerBitmap;
    private BitmapDrawable mBoundsBoxBitmap;
    private int mPreviousTouchX, mPreviousTouchY;
    private int mControlPositionX, mControlPositionY;
    private int mAlpha;
    private int mPointerId;
    private Rect mVirtBounds;
    private Rect mOrigBounds;

    public InputOverlayJoystick(BitmapDrawable bitmapBounds, BitmapDrawable bitmapOuter,
                                BitmapDrawable InnerDefault, BitmapDrawable InnerPressed,
                                Rect rectOuter, Rect rectInner, int buttonId) {
        mPointerId = -1;
        mOuterBitmap = bitmapOuter;
        mDefaultInnerBitmap = InnerDefault;
        mPressedInnerBitmap = InnerPressed;
        mBoundsBoxBitmap = bitmapBounds;

        mAxisIDs[0] = buttonId + 0;
        mAxisIDs[1] = buttonId + 1;

        setBounds(rectOuter);
        mDefaultInnerBitmap.setBounds(rectInner);
        mPressedInnerBitmap.setBounds(rectInner);
        mVirtBounds = mOuterBitmap.copyBounds();
        mOrigBounds = mOuterBitmap.copyBounds();
        mBoundsBoxBitmap.setAlpha(0);
        mBoundsBoxBitmap.setBounds(mVirtBounds);
        updateInnerBounds();
    }

    public void onConfigureBegin(int x, int y) {
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onConfigureMove(int x, int y) {
        int deltaX = x - mPreviousTouchX;
        int deltaY = y - mPreviousTouchY;
        mControlPositionX += deltaX;
        mControlPositionY += deltaY;
        setBounds(new Rect(mControlPositionX, mControlPositionY,
                           mOuterBitmap.getIntrinsicWidth() + mControlPositionX,
                           mOuterBitmap.getIntrinsicHeight() + mControlPositionY));
        mVirtBounds.set(mControlPositionX, mControlPositionY,
                        mOuterBitmap.getIntrinsicWidth() + mControlPositionX,
                        mOuterBitmap.getIntrinsicHeight() + mControlPositionY);
        updateInnerBounds();
        mOrigBounds.set(mControlPositionX, mControlPositionY,
                        mOuterBitmap.getIntrinsicWidth() + mControlPositionX,
                        mOuterBitmap.getIntrinsicHeight() + mControlPositionY);
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onDraw(Canvas canvas) {
        mOuterBitmap.draw(canvas);
        getCurrentBitmapDrawable().draw(canvas);
        mBoundsBoxBitmap.draw(canvas);
    }

    public void onPointerDown(int id, float x, float y) {
        mOuterBitmap.setAlpha(0);
        mBoundsBoxBitmap.setAlpha(mAlpha);
        if (InputOverlay.sJoystickRelative) {
            // reCenter
            mVirtBounds.offset((int)x - mVirtBounds.centerX(), (int)y - mVirtBounds.centerY());
        }
        mBoundsBoxBitmap.setBounds(mVirtBounds);
        mPointerId = id;

        setJoystickState(x, y);
    }

    public void onPointerMove(int id, float x, float y) {
        setJoystickState(x, y);
    }

    public void onPointerUp(int id, float x, float y) {
        mOuterBitmap.setAlpha(mAlpha);
        mBoundsBoxBitmap.setAlpha(0);
        mVirtBounds = new Rect(mOrigBounds);
        setBounds(mOrigBounds);
        mPointerId = -1;

        setJoystickState(x, y);
    }

    private void setJoystickState(float touchX, float touchY) {
        if (mPointerId != -1) {
            float maxY = mVirtBounds.bottom;
            float maxX = mVirtBounds.right;
            touchX -= mVirtBounds.centerX();
            maxX -= mVirtBounds.centerX();
            touchY -= mVirtBounds.centerY();
            maxY -= mVirtBounds.centerY();
            final float AxisX = touchX / maxX;
            final float AxisY = touchY / maxY;
            mAxises[0] = AxisX;
            mAxises[1] = -AxisY;
        } else {
            mAxises[0] = mAxises[1] = 0.0f;
        }

        updateInnerBounds();

        NativeLibrary.InputEvent(mAxisIDs[0], mAxises[0]);
        NativeLibrary.InputEvent(mAxisIDs[1], mAxises[1]);
    }

    public void setPosition(int x, int y) {
        mControlPositionX = x;
        mControlPositionY = y;
    }

    public void setAlpha(int value) {
        mAlpha = value;
        mDefaultInnerBitmap.setAlpha(value);
        mOuterBitmap.setAlpha(value);
    }

    public int getButtonId() {
        return mAxisIDs[0];
    }

    public int getPointerId() {
        return mPointerId;
    }

    public Rect getBounds() {
        return mOuterBitmap.getBounds();
    }

    public void setBounds(Rect bounds) {
        mOuterBitmap.setBounds(bounds);
    }

    private BitmapDrawable getCurrentBitmapDrawable() {
        return mPointerId != -1 ? mPressedInnerBitmap : mDefaultInnerBitmap;
    }

    private void updateInnerBounds() {
        int X = mVirtBounds.centerX() + (int)((mAxises[0]) * (mVirtBounds.width() / 2));
        int Y = mVirtBounds.centerY() + (int)((-mAxises[1]) * (mVirtBounds.height() / 2));

        if (X > mVirtBounds.centerX() + (mVirtBounds.width() / 2))
            X = mVirtBounds.centerX() + (mVirtBounds.width() / 2);
        if (X < mVirtBounds.centerX() - (mVirtBounds.width() / 2))
            X = mVirtBounds.centerX() - (mVirtBounds.width() / 2);
        if (Y > mVirtBounds.centerY() + (mVirtBounds.height() / 2))
            Y = mVirtBounds.centerY() + (mVirtBounds.height() / 2);
        if (Y < mVirtBounds.centerY() - (mVirtBounds.height() / 2))
            Y = mVirtBounds.centerY() - (mVirtBounds.height() / 2);

        int width = mPressedInnerBitmap.getBounds().width() / 2;
        int height = mPressedInnerBitmap.getBounds().height() / 2;
        mDefaultInnerBitmap.setBounds(X - width, Y - height, X + width, Y + height);
        mPressedInnerBitmap.setBounds(mDefaultInnerBitmap.getBounds());
    }
}
