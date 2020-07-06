package org.citra.emu.overlay;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import org.citra.emu.NativeLibrary;

public final class InputOverlayDpad {
    private boolean[] mPressStates = new boolean[4];
    private int[] mButtonIds = new int[4];
    private boolean mIsStick;
    private Bitmap mDefaultBitmap;
    private Bitmap mOnePressedBitmap;
    private Bitmap mTwoPressedBitmap;
    private int mPreviousTouchX, mPreviousTouchY;
    private int mPointerId;
    private Rect mBounds;

    public InputOverlayDpad(Bitmap defaultBitmap, Bitmap onePressedBitmap,
                            Bitmap twoPressedBitmap, int buttonId) {
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
        mBounds.offset(x - mPreviousTouchX, y - mPreviousTouchY);
        mPreviousTouchX = x;
        mPreviousTouchY = y;
    }

    public void onDraw(Canvas canvas, Paint paint) {
        int px = mBounds.left + (mBounds.width() / 2);
        int py = mBounds.top + (mBounds.height() / 2);

        boolean up = mPressStates[0];
        boolean down = mPressStates[1];
        boolean left = mPressStates[2];
        boolean right = mPressStates[3];

        if (up) {
            if (left) {
                canvas.drawBitmap(mTwoPressedBitmap, null, mBounds, paint);
            } else if (right) {
                canvas.save();
                canvas.rotate(90, px, py);
                canvas.drawBitmap(mTwoPressedBitmap, null, mBounds, paint);
                canvas.restore();
            } else {
                canvas.drawBitmap(mOnePressedBitmap, null, mBounds, paint);
            }
        } else if (down) {
            if (left) {
                canvas.save();
                canvas.rotate(270, px, py);
                canvas.drawBitmap(mTwoPressedBitmap, null, mBounds, paint);
                canvas.restore();
            } else if (right) {
                canvas.save();
                canvas.rotate(180, px, py);
                canvas.drawBitmap(mTwoPressedBitmap, null, mBounds, paint);
                canvas.restore();
            } else {
                canvas.save();
                canvas.rotate(180, px, py);
                canvas.drawBitmap(mOnePressedBitmap, null, mBounds, paint);
                canvas.restore();
            }
        } else if (left) {
            canvas.save();
            canvas.rotate(270, px, py);
            canvas.drawBitmap(mOnePressedBitmap, null, mBounds, paint);
            canvas.restore();
        } else if (right) {
            canvas.save();
            canvas.rotate(90, px, py);
            canvas.drawBitmap(mOnePressedBitmap, null, mBounds, paint);
            canvas.restore();
        } else {
            canvas.drawBitmap(mDefaultBitmap, null, mBounds, paint);
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

    public int getButtonId() {
        return mButtonIds[0];
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
