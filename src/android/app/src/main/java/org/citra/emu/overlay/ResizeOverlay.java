package org.citra.emu.overlay;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import org.citra.emu.R;

public class ResizeOverlay extends View {
    /**
     * point 0 and point 2 are of same group and same as point 1 and point 3
     */
    private Point[] points = new Point[4];
    // move rect
    private int initialX = -1;
    private int initialY = -1;
    private Point[] initials = new Point[4];
    // resize rect
    private int groupId = -1;
    private int pointId = -1;
    // touch
    private int touchId = -1;
    // draw
    private Paint paint;
    private Bitmap bitmap;

    public interface ResizeListener {
        void onResize(View v);
    }
    ResizeListener mListener;

    public ResizeOverlay(Context context) {
        super(context);
        paint = new Paint();
        // necessary for getting the touch events
        setFocusable(true);
        setRect(new Rect(0, 0, 500, 500));

        bitmap = BitmapFactory.decodeResource(context.getResources(), R.drawable.gray_circle);
    }

    public ResizeOverlay(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }

    public ResizeOverlay(Context context, AttributeSet attrs) {
        super(context, attrs);
        paint = new Paint();
        // necessary for getting the touch events
        setFocusable(true);
        setRect(new Rect(0, 0, 500, 500));

        bitmap = BitmapFactory.decodeResource(context.getResources(), R.drawable.gray_circle);
    }

    public void setResizeListener(ResizeListener listener) {
        mListener = listener;
    }

    public void setRect(Rect rect) {
        points[0] = new Point();
        points[0].x = rect.left;
        points[0].y = rect.top;

        points[1] = new Point();
        points[1].x = rect.right;
        points[1].y = rect.top;

        points[2] = new Point();
        points[2].x = rect.right;
        points[2].y = rect.bottom;

        points[3] = new Point();
        points[3].x = rect.left;
        points[3].y = rect.bottom;

        if (mListener != null) {
            mListener.onResize(this);
        }
    }

    public Rect getRect() {
        int left, right;
        if (points[0].x > points[2].x) {
            left = points[2].x;
            right = points[0].x;
        } else {
            left = points[0].x;
            right = points[2].x;
        }

        int top, bottom;
        if (points[0].y > points[2].y) {
            top = points[2].y;
            bottom = points[0].y;
        } else {
            top = points[0].y;
            bottom = points[2].y;
        }

        return new Rect(left, top, right, bottom);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        paint.setAntiAlias(true);
        paint.setDither(true);
        paint.setColor(0x55000000);
        paint.setStyle(Paint.Style.FILL);
        paint.setStrokeJoin(Paint.Join.ROUND);
        paint.setStrokeWidth(5);

        canvas.drawPaint(paint);
        paint.setColor(0x55FFFFFF);

        float radius = bitmap.getWidth() / 2.0f;

        if (groupId == 1) {
            canvas.drawRect(points[0].x, points[2].y, points[2].x, points[0].y, paint);
        } else {
            canvas.drawRect(points[1].x, points[3].y, points[3].x, points[1].y, paint);
        }

        // draw the balls on the canvas
        canvas.drawBitmap(bitmap, points[0].x - radius, points[0].y - radius, paint);
        canvas.drawBitmap(bitmap, points[1].x - radius, points[1].y - radius, paint);
        canvas.drawBitmap(bitmap, points[2].x - radius, points[2].y - radius, paint);
        canvas.drawBitmap(bitmap, points[3].x - radius, points[3].y - radius, paint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        boolean handled = false;
        int id = event.getPointerId(0);
        int x = (int) event.getX();
        int y = (int) event.getY();

        switch (event.getAction()) {
        case MotionEvent.ACTION_DOWN:
            if (touchId == -1) {
                double minDistance = bitmap.getWidth();
                for (int i = 0; i < points.length; ++i) {
                    // check if inside the bounds of the ball (circle)
                    int centerX = points[i].x;
                    int centerY = points[i].y;
                    // calculate the radius from the touch to the center of the ball
                    double distance = Math.sqrt((centerX - x) * (centerX - x) + (centerY - y) * (centerY - y));
                    if (distance < minDistance) {
                        pointId = i;
                        minDistance = distance;
                    }
                }
                if (pointId == 1 || pointId == 3) {
                    groupId = 2;
                    touchId = event.getPointerId(0);
                    handled = true;
                } else if (pointId == 0 || pointId == 2) {
                    groupId = 1;
                    touchId = id;
                    handled = true;
                } else {
                    Rect rect = getRect();
                    if (rect.contains(x, y)) {
                        touchId = id;
                        initialX = x;
                        initialY = y;
                        initials[0] = new Point(points[0]);
                        initials[1] = new Point(points[1]);
                        initials[2] = new Point(points[2]);
                        initials[3] = new Point(points[3]);
                        handled = true;
                    }
                }
            }
            break;
        case MotionEvent.ACTION_MOVE:
            if (touchId == id) {
                if (pointId != -1) {
                    points[pointId].x = Math.max(x, 0);
                    points[pointId].y = Math.max(y, 0);
                    if (groupId == 1) {
                        points[1].x = points[0].x;
                        points[1].y = points[2].y;
                        points[3].x = points[2].x;
                        points[3].y = points[0].y;
                    } else {
                        points[0].x = points[1].x;
                        points[0].y = points[3].y;
                        points[2].x = points[3].x;
                        points[2].y = points[1].y;
                    }
                } else {
                    int moveX = x - initialX;
                    int moveY = y - initialY;
                    for (int i = 0; i < points.length; ++i) {
                        points[i].x = Math.max(initials[i].x + moveX, 0);
                        points[i].y = Math.max(initials[i].y + moveY, 0);
                    }
                }
                handled = true;
            }
            break;
        case MotionEvent.ACTION_UP:
            if (touchId == id) {
                touchId = -1;
                pointId = -1;
                groupId = -1;
                handled = true;
            }
            break;
        }
        if (handled) {
            // redraw the canvas
            invalidate();
            if (mListener != null) {
                mListener.onResize(this);
            }
        }
        return handled || super.onTouchEvent(event);
    }
}
