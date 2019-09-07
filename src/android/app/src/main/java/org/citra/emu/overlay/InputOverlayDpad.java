package org.citra.emu.overlay;

import android.graphics.Canvas;
import android.graphics.Rect;
import android.graphics.drawable.BitmapDrawable;
import org.citra.emu.NativeLibrary;

public final class InputOverlayDpad {
    private boolean[] mPressStates = new boolean[4];
    private int[] mButtonIds = new int[4];
    private boolean mIsStick;
    private BitmapDrawable mDefaultBitmap;
    private BitmapDrawable mOnePressedBitmap;
    private BitmapDrawable mTwoPressedBitmap;
    private int mPreviousTouchX, mPreviousTouchY;
    private int mControlPositionX, mControlPositionY;
    private int mPointerId;

    public InputOverlayDpad(BitmapDrawable defaultBitmap, BitmapDrawable onePressedBitmap,
                            BitmapDrawable twoPressedBitmap, int buttonId) {
        mPointerId = -1;
        mDefaultBitmap = defaultBitmap;
        mOnePressedBitmap = onePressedBitmap;
        mTwoPressedBitmap = twoPressedBitmap;

        mIsStick = NativeLibrary.ButtonType.N3DS_DPAD_UP != buttonId;

        mButtonIds[0] = buttonId + 0;
        mButtonIds[1] = buttonId + 1;
        mButtonIds[2] = buttonId + 2;
        mButtonIds[3] = buttonId + 3;

        mPressStates[0] = false;
        mPressStates[1] = false;
        mPressStates[2] = false;
        mPressStates[3] = false;
    }

    public void onConfigureBegin(int x, int y) {
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onConfigureMove(int x, int y) {
        Rect bounds = getBounds();
        mControlPositionX += x - mPreviousTouchX;
        mControlPositionY += y - mPreviousTouchY;
        setBounds(new Rect(mControlPositionX, mControlPositionY, mControlPositionX + bounds.width(),
                           mControlPositionY + bounds.height()));
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onDraw(Canvas canvas) {
        Rect bounds = getBounds();
        int px = mControlPositionX + (bounds.width() / 2);
        int py = mControlPositionY + (bounds.height() / 2);

        boolean up = mPressStates[0];
        boolean down = mPressStates[1];
        boolean left = mPressStates[2];
        boolean right = mPressStates[3];

        if (up) {
            if (left)
                mTwoPressedBitmap.draw(canvas);
            else if (right) {
                canvas.save();
                canvas.rotate(90, px, py);
                mTwoPressedBitmap.draw(canvas);
                canvas.restore();
            } else
                mOnePressedBitmap.draw(canvas);
        } else if (down) {
            if (left) {
                canvas.save();
                canvas.rotate(270, px, py);
                mTwoPressedBitmap.draw(canvas);
                canvas.restore();
            } else if (right) {
                canvas.save();
                canvas.rotate(180, px, py);
                mTwoPressedBitmap.draw(canvas);
                canvas.restore();
            } else {
                canvas.save();
                canvas.rotate(180, px, py);
                mOnePressedBitmap.draw(canvas);
                canvas.restore();
            }
        } else if (left) {
            canvas.save();
            canvas.rotate(270, px, py);
            mOnePressedBitmap.draw(canvas);
            canvas.restore();
        } else if (right) {
            canvas.save();
            canvas.rotate(90, px, py);
            mOnePressedBitmap.draw(canvas);
            canvas.restore();
        } else {
            mDefaultBitmap.draw(canvas);
        }
    }

    public void onPointerDown(int id, float x, float y) {
        mPointerId = id;
        if (mIsStick)
            setDpadState2((int)x, (int)y);
        else
            setDpadState4((int)x, (int)y);
    }

    public void onPointerMove(int id, float x, float y) {
        if (mIsStick)
            setDpadState2((int)x, (int)y);
        else
            setDpadState4((int)x, (int)y);
    }

    public void onPointerUp(int id, float x, float y) {
        mPointerId = -1;
        if (mIsStick)
            setDpadState2((int)x, (int)y);
        else
            setDpadState4((int)x, (int)y);
    }

    private void setDpadState2(int pointerX, int pointerY) {
        // x, y
        float[] axises = {0f, 0f};
        // Up, Down, Left, Right
        mPressStates[0] = false;
        mPressStates[1] = false;
        mPressStates[2] = false;
        mPressStates[3] = false;

        if (mPointerId != -1) {
            Rect bounds = getBounds();
            if (bounds.top + (bounds.height() / 3) > pointerY) {
                axises[1] = 1.0f;
                mPressStates[0] = true;
            } else if (bounds.bottom - (bounds.height() / 3) < pointerY) {
                axises[1] = -1.0f;
                mPressStates[1] = true;
            }

            if (bounds.left + (bounds.width() / 3) > pointerX) {
                axises[0] = -1.0f;
                mPressStates[2] = true;
            } else if (bounds.right - (bounds.width() / 3) < pointerX) {
                axises[0] = 1.0f;
                mPressStates[3] = true;
            }
        }

        NativeLibrary.InputEvent(mButtonIds[0], axises[0]);
        NativeLibrary.InputEvent(mButtonIds[1], axises[1]);
    }

    private void setDpadState4(int pointerX, int pointerY) {
        // Up, Down, Left, Right
        boolean[] pressed = {false, false, false, false};

        if (mPointerId != -1) {
            Rect bounds = getBounds();

            if (bounds.top + (bounds.height() / 3) > pointerY)
                pressed[0] = true;
            if (bounds.bottom - (bounds.height() / 3) < pointerY)
                pressed[1] = true;
            if (bounds.left + (bounds.width() / 3) > pointerX)
                pressed[2] = true;
            if (bounds.right - (bounds.width() / 3) < pointerX)
                pressed[3] = true;
        }

        for (int i = 0; i < pressed.length; ++i) {
            if (pressed[i] != mPressStates[i]) {
                NativeLibrary.InputEvent(mButtonIds[i], pressed[i]
                                                            ? NativeLibrary.ButtonState.PRESSED
                                                            : NativeLibrary.ButtonState.RELEASED);
                mPressStates[i] = pressed[i];
            }
        }
    }

    public void setPosition(int x, int y) {
        mControlPositionX = x;
        mControlPositionY = y;
    }

    public void setAlpha(int value) {
        mDefaultBitmap.setAlpha(value);
    }

    public int getButtonId() {
        return mButtonIds[0];
    }

    public int getPointerId() {
        return mPointerId;
    }

    public Rect getBounds() {
        return mDefaultBitmap.getBounds();
    }

    public void setBounds(Rect bounds) {
        mDefaultBitmap.setBounds(bounds);
        mOnePressedBitmap.setBounds(bounds);
        mTwoPressedBitmap.setBounds(bounds);
    }
}
