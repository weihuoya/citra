package org.citra.emu.overlay;

import org.citra.emu.NativeLibrary;

public final class InputOverlayPointer {
    private int mPointerId;

    public InputOverlayPointer() {
        mPointerId = -1;
    }

    public void onPointerDown(int id, float x, float y) {
        mPointerId = id;
        NativeLibrary.TouchEvent(0, x, y);
    }

    public void onPointerMove(int id, float x, float y) {
        NativeLibrary.TouchEvent(1, x, y);
    }

    public void onPointerUp(int id, float x, float y) {
        mPointerId = -1;
        NativeLibrary.TouchEvent(2, x, y);
    }

    public int getPointerId() {
        return mPointerId;
    }
}
