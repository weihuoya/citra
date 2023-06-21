package org.citra.emu.ui;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.Rect;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.view.Choreographer;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewTreeObserver;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethod;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.overlay.InputOverlay;
import org.citra.emu.overlay.LassoOverlay;
import org.citra.emu.overlay.ResizeOverlay;
import org.citra.emu.utils.NetPlayManager;
import org.citra.emu.utils.TranslateHelper;

import java.util.ArrayList;
import java.util.List;

import static android.os.Looper.getMainLooper;

public final class EmulationFragment extends Fragment implements SurfaceHolder.Callback, Choreographer.FrameCallback {
    private static final String KEY_GAMEPATH = "gamepath";

    private static final int TASK_PROGRESS_BAIDUOCR0 = 0;
    private static final int TASK_PROGRESS_BAIDUOCR1 = 1;
    private static final int TASK_PROGRESS_TRANSLATE = 2;

    private String mPath;
    private Surface mSurface;
    private EmulationState mState;
    private boolean mRunWhenSurfaceIsValid;
    private InputOverlay mInputOverlay;
    private ResizeOverlay mResizeOverlayTop;
    private ResizeOverlay mResizeOverlayBottom;
    private LassoOverlay mLassoOverlay;
    private TextView mTranslateText;
    private TextView mNetPlayMessage;
    private ProgressBar mProgressBar;
    private Button mBtnDone;

    private LinearLayout mChatLayout;
    private EditText mChatEditText;
    private ImageButton mBtnChatSend;

    private List<String> mMessageList;
    private Handler mTaskHandler;

    private static class BaiduOCRTask extends AsyncTask<Bitmap, Integer, Integer> {
        @SuppressLint("StaticFieldLeak")
        EmulationFragment mParent;

        public BaiduOCRTask(EmulationFragment parent) {
            mParent = parent;
        }

        @Override
        protected void onPreExecute() {
            TranslateHelper.IsRunning = true;
        }

        @Override
        protected Integer doInBackground(Bitmap... args) {
            Bitmap image = args[0];
            publishProgress(TASK_PROGRESS_BAIDUOCR0);
            int error = TranslateHelper.RequestBaiduOCR(image);
            if (error == TranslateHelper.ERROR_BAIDUOCR_TRY_AGAIN) {
                publishProgress(TASK_PROGRESS_BAIDUOCR1);
                error = TranslateHelper.RequestBaiduOCR(image);
            }
            if (error == TranslateHelper.ERROR_SUCCESS && TranslateHelper.Service != TranslateHelper.NONE_TRANSLATE) {
                publishProgress(TASK_PROGRESS_TRANSLATE);
                if (TranslateHelper.Service == TranslateHelper.GOOGLE_TRANSLATE) {
                    error = TranslateHelper.RequestGoogle();
                } else if (TranslateHelper.Service == TranslateHelper.YOUDAO_TRANSLATE) {
                    error = TranslateHelper.RequestYoudao();
                } else if (TranslateHelper.Service == TranslateHelper.YEEKIT_TRANSLATE) {
                    error = TranslateHelper.RequestYeekit();
                }
            }
            image.recycle();
            return error;
        }

        @Override
        protected void onProgressUpdate(Integer... values) {
            mParent.setTranslateProgress(values[0]);
        }

        @Override
        protected void onPostExecute(Integer error) {
            TranslateHelper.IsRunning = false;
            mParent.setTranslateText(error);
        }
    }

    public static EmulationFragment newInstance(String gamePath) {
        Bundle args = new Bundle();
        args.putString(KEY_GAMEPATH, gamePath);
        EmulationFragment fragment = new EmulationFragment();
        fragment.setArguments(args);
        return fragment;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // So this fragment doesn't restart on configuration changes; i.e. rotation.
        setRetainInstance(true);

        mPath = getArguments().getString(KEY_GAMEPATH);
        mState = EmulationState.STOPPED;
    }

    @SuppressLint("ClickableViewAccessibility")
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View contents = inflater.inflate(R.layout.fragment_emulation, container, false);

        SurfaceView surfaceView = contents.findViewById(R.id.surface_emulation);
        surfaceView.getHolder().addCallback(this);

        mProgressBar = contents.findViewById(R.id.running_progress);
        mTranslateText = contents.findViewById(R.id.translate_text);
        mNetPlayMessage = contents.findViewById(R.id.netplay_message);
        mInputOverlay = contents.findViewById(R.id.surface_input_overlay);
        mResizeOverlayTop = contents.findViewById(R.id.resize_overlay_top);
        mResizeOverlayTop.setDesiredRatio(400 / 240.0f);
        mResizeOverlayTop.setOnResizeListener(v -> {
            Rect rect = ((ResizeOverlay)v).getRect();
            NativeLibrary.setCustomLayout(true, rect.left, rect.top, rect.right, rect.bottom);
        });
        mResizeOverlayBottom = contents.findViewById(R.id.resize_overlay_bottom);
        mResizeOverlayBottom.setDesiredRatio(320 / 240.0f);
        mResizeOverlayBottom.setOnResizeListener(v -> {
            Rect rect = ((ResizeOverlay)v).getRect();
            NativeLibrary.setCustomLayout(false, rect.left, rect.top, rect.right, rect.bottom);
        });
        mLassoOverlay = contents.findViewById(R.id.lasso_overlay);
        mLassoOverlay.setOnMoveListener(v -> layoutTranslateText());
        mLassoOverlay.setOnClickListener(v -> requestScreenshot());
        mBtnDone = contents.findViewById(R.id.done_control_config);
        mBtnDone.setOnClickListener(v -> {
            stopConfiguringControls();
            stopConfiguringLayout();
            mInputOverlay.onPressedFeedback();
        });

        mChatLayout = contents.findViewById(R.id.chat_input);
        if (!NetPlayManager.NetPlayIsJoined()) {
            mChatLayout.setVisibility(View.GONE);
        }
        mChatLayout.setOnTouchListener(new View.OnTouchListener() {
            private int prevX;
            private int prevY;
            private int leftMargin;
            private int topMargin;
            private FrameLayout.LayoutParams params;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_MOVE: {
                        final int currX = (int)event.getRawX();
                        final int currY = (int)event.getRawY();
                        //params.leftMargin = leftMargin + currX - prevX;
                        params.topMargin = topMargin + currY - prevY;
                        v.setLayoutParams(params);
                        return true;
                    }
                    case MotionEvent.ACTION_UP: {
                        return true;
                    }
                    case MotionEvent.ACTION_DOWN: {
                        prevX = (int) event.getRawX();
                        prevY = (int) event.getRawY();
                        params = (FrameLayout.LayoutParams)v.getLayoutParams();
                        leftMargin = params.leftMargin;
                        topMargin = params.topMargin;
                        return true;
                    }
                }
                return false;
            }
        });

        mBtnChatSend = contents.findViewById(R.id.chat_send_button);
        mBtnChatSend.setOnTouchListener(new View.OnTouchListener() {
            private int prevX;
            private int prevY;
            private int leftMargin;
            private int topMargin;
            private FrameLayout.LayoutParams params;
            private long touchtime;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_MOVE: {
                        final int currX = (int)event.getRawX();
                        final int currY = (int)event.getRawY();
                        //params.leftMargin = leftMargin + currX - prevX;
                        params.topMargin = topMargin + currY - prevY;
                        mChatLayout.setLayoutParams(params);
                        return true;
                    }
                    case MotionEvent.ACTION_UP: {
                        if (System.currentTimeMillis() - touchtime < 100) {
                            toggleInput();
                        }
                        return true;
                    }
                    case MotionEvent.ACTION_DOWN: {
                        prevX = (int) event.getRawX();
                        prevY = (int) event.getRawY();
                        params = (FrameLayout.LayoutParams)mChatLayout.getLayoutParams();
                        leftMargin = params.leftMargin;
                        topMargin = params.topMargin;
                        touchtime = System.currentTimeMillis();
                        return true;
                    }
                }
                return false;
            }
        });

        mChatEditText = contents.findViewById(R.id.chat_text_input);
        mChatEditText.setOnEditorActionListener((view, action, event) -> {
            if (action == EditorInfo.IME_ACTION_SEND) {
                String name = NetPlayManager.GetUsername(getActivity());
                String msg = view.getText().toString().trim();
                if (!msg.isEmpty()) {
                    addNetPlayMessage(name + ": " + msg);
                    NetPlayManager.NetPlaySendMessage(msg);
                    view.setText("");
                    toggleInput();
                }
                return true;
            }
            return false;
        });

        return contents;
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        // All work is done in surfaceChanged
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        mSurface = holder.getSurface();
        if (mRunWhenSurfaceIsValid) {
            runWithValidSurface();
        }
    }

    @Override
    public void doFrame(long frameTimeNanos) {
        Choreographer.getInstance().postFrameCallback(this);
        NativeLibrary.DoFrame();
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        if (mSurface == null) {
            // [EmulationFragment] clearSurface called, but surface already null.
        } else {
            mSurface = null;
            if (mState == EmulationState.RUNNING) {
                NativeLibrary.SurfaceDestroyed();
                mState = EmulationState.PAUSED;
            } else if (mState == EmulationState.PAUSED) {
                // [EmulationFragment] Surface cleared while emulation paused.
            } else {
                // [EmulationFragment] Surface cleared while emulation stopped.
            }
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        Choreographer.getInstance().postFrameCallback(this);

        if (NativeLibrary.IsRunning()) {
            mState = EmulationState.PAUSED;
        }

        // If the surface is set, run now. Otherwise, wait for it to get set.
        if (mSurface != null) {
            runWithValidSurface();
        } else {
            mRunWhenSurfaceIsValid = true;
        }
    }

    @Override
    public void onPause() {
        if (mState == EmulationState.RUNNING) {
            mState = EmulationState.PAUSED;
            // Release the surface before pausing, since emulation has to be running for that.
            NativeLibrary.SurfaceDestroyed();
            NativeLibrary.PauseEmulation();
        }
        Choreographer.getInstance().removeFrameCallback(this);
        super.onPause();
    }

    @Override
    public void onConfigurationChanged(@NonNull Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        mInputOverlay.refreshControls();

        final ViewTreeObserver observer = getView().getViewTreeObserver();
        observer.addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                EmulationActivity activity = (EmulationActivity)getActivity();
                activity.updateDisplayConfig();
                activity.updateBackgroundImage();
                NativeLibrary.WindowChanged();
                mResizeOverlayTop.setRect(NativeLibrary.getCustomLayout(true));
                mResizeOverlayBottom.setRect(NativeLibrary.getCustomLayout(false));
                observer.removeOnGlobalLayoutListener(this);
            }
        });
    }

    private void toggleInput() {
        float alpha = mChatLayout.getAlpha();
        final Activity activity = getActivity();
        final View decor = activity.getWindow().getDecorView();
        InputMethodManager imm = activity.getSystemService(InputMethodManager.class);

        if (alpha > 0.5f) {
            mChatEditText.clearFocus();
            mChatEditText.setVisibility(View.GONE);
            mChatLayout.animate().alpha(0.5f).setDuration(500).start();
            imm.hideSoftInputFromWindow(decor.getWindowToken(), 0);
        } else {
            mChatLayout.animate().alpha(1.0f).setDuration(500).start();
            mChatEditText.setVisibility(View.VISIBLE);
            mChatEditText.requestFocus();
            imm.showSoftInput(mChatEditText, InputMethod.SHOW_FORCED);
        }
    }

    public void setTranslateProgress(int progress) {
        layoutTranslateText();
        if (progress == TASK_PROGRESS_BAIDUOCR0) {
            mTranslateText.setText("百度识图");
        } else if (progress == TASK_PROGRESS_BAIDUOCR1) {
            mTranslateText.setText("超时重试");
        } else if (progress == TASK_PROGRESS_TRANSLATE) {
            mTranslateText.setText("翻译文本");
        }
    }

    public void setTranslateText(int error) {
        layoutTranslateText();
        if (error == TranslateHelper.ERROR_SUCCESS) {
            StringBuilder sb = new StringBuilder();
            if (TranslateHelper.ShowOCRResults) {
                for (int i = 0; i < TranslateHelper.BaiduOCRResults.size(); ++i) {
                    sb.append(TranslateHelper.BaiduOCRResults.get(i));
                    sb.append(System.lineSeparator());
                }
            }
            for (int i = 0; i < TranslateHelper.TranslateResults.size(); ++i) {
                sb.append(TranslateHelper.TranslateResults.get(i));
                sb.append(System.lineSeparator());
            }
            mTranslateText.setText(sb.toString());
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_NET_ERROR) {
            mTranslateText.setText("网络错误！");
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_MISMATCH_API) {
            mTranslateText.setText("Mismatch API!");
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_OUT_OF_USAGE) {
            mTranslateText.setText("Out of Usage!");
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_TRY_AGAIN) {
            mTranslateText.setText("Try Again!");
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_TOKEN_OVERDUE) {
            mTranslateText.setText("Token Overdue!");
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_TOO_SMALL) {
            mTranslateText.setText("Too Small!");
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_INVALID_CONTENT) {
            mTranslateText.setText("Invalid Content!");
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_EMPTY_RESULT) {
            mTranslateText.setText("");
        } else if (error == TranslateHelper.ERROR_BAIDUOCR_UNKNOWN_ERROR) {
            mTranslateText.setText("未知错误！");
        } else if (error == TranslateHelper.ERROR_YOUDAO_NET_ERROR) {
            mTranslateText.setText("网络错误！");
        } else if (error == TranslateHelper.ERROR_YOUDAO_INVALID_CONTENT) {
            mTranslateText.setText("Invalid Content!");
        } else if (error == TranslateHelper.ERROR_YOUDAO_UNKNOWN_ERROR) {
            mTranslateText.setText("未知错误！");
        } else if (error == TranslateHelper.ERROR_YEEKIT_NET_ERROR) {
            mTranslateText.setText("网络错误！");
        } else if (error == TranslateHelper.ERROR_YEEKIT_INVALID_CONTENT) {
            mTranslateText.setText("Invalid Content!");
        } else if (error == TranslateHelper.ERROR_YEEKIT_UNKNOWN_ERROR) {
            mTranslateText.setText("未知错误！");
        } else if (error == TranslateHelper.ERROR_GOOGLE_NET_ERROR) {
            mTranslateText.setText("网络错误！");
        } else if (error == TranslateHelper.ERROR_GOOGLE_INVALID_CONTENT) {
            mTranslateText.setText("Invalid Content!");
        } else if (error == TranslateHelper.ERROR_GOOGLE_UNKNOWN_ERROR) {
            mTranslateText.setText("未知错误！");
        }
    }

    public void layoutTranslateText() {
        Rect rect = mLassoOverlay.getRect();
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(rect.width(), rect.height() * 2);
        params.setMargins(rect.left, rect.top + rect.height(), rect.right, 0);
        mTranslateText.setLayoutParams(params);
    }

    public void requestScreenshot() {
        if (TranslateHelper.IsRunning) {
            mTranslateText.setText("等待翻译");
            return;
        } else {
            mTranslateText.setText("屏幕截图");
        }
        NativeLibrary.Screenshot((width, height, pixels) -> {
            final Bitmap screenshot = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            screenshot.setPixels(pixels, 0, width, 0, 0, width, height);
            ((Activity) getContext()).runOnUiThread(() -> {
                Rect rect = mLassoOverlay.getRect();
                int fit_width = Math.min(rect.width(), screenshot.getWidth() - rect.left);
                int fit_height = Math.min(rect.height(), screenshot.getHeight() - rect.top);
                Bitmap image = Bitmap.createBitmap(screenshot, rect.left, rect.top, fit_width, fit_height);
                BaiduOCRTask task = new BaiduOCRTask(this);
                TranslateHelper.BaiduOCRLanguage = TranslateHelper.BaiduOCRLanguageJPN;
                task.execute(image);
                screenshot.recycle();
            });
        });
    }

    public void startConfiguringControls() {
        mBtnDone.setVisibility(View.VISIBLE);
        mInputOverlay.setInEditMode(true);
    }

    public void stopConfiguringControls() {
        if (!mInputOverlay.isInEditMode()) {
            return;
        }
        mBtnDone.setVisibility(View.INVISIBLE);
        mInputOverlay.setInEditMode(false);
    }

    public void stopEmulation() {
        if (mState != EmulationState.STOPPED) {
            mState = EmulationState.STOPPED;
            NativeLibrary.StopEmulation();
        }
    }

    public void refreshControls() {
        mInputOverlay.refreshControls();
    }

    private boolean mPreviousHide;
    public void startConfiguringLayout() {
        mPreviousHide = InputOverlay.sHideInputOverlay;
        InputOverlay.sHideInputOverlay = true;
        mInputOverlay.setInEditMode(false);
        refreshControls();
        mResizeOverlayTop.setVisibility(View.VISIBLE);
        mResizeOverlayTop.setRect(NativeLibrary.getCustomLayout(true));
        mResizeOverlayBottom.setVisibility(View.VISIBLE);
        mResizeOverlayBottom.setRect(NativeLibrary.getCustomLayout(false));
        mBtnDone.setVisibility(View.VISIBLE);
    }

    public void stopConfiguringLayout() {
        if (mResizeOverlayTop.getVisibility() == View.INVISIBLE) {
            return;
        }
        InputOverlay.sHideInputOverlay = mPreviousHide;
        refreshControls();
        mResizeOverlayTop.setVisibility(View.INVISIBLE);
        mResizeOverlayBottom.setVisibility(View.INVISIBLE);
        mBtnDone.setVisibility(View.INVISIBLE);
    }

    public void setLassoOverlay(boolean enabled) {
        if (enabled) {
            mTranslateText.setVisibility(View.VISIBLE);
            mLassoOverlay.setVisibility(View.VISIBLE);
        } else {
            mLassoOverlay.setVisibility(View.INVISIBLE);
            mTranslateText.setVisibility(View.INVISIBLE);
            mTranslateText.setText("");
        }
    }

    public void addNetPlayMessage(String msg) {
        if (msg.isEmpty()) {
            return;
        }

        mChatLayout.setVisibility(NetPlayManager.NetPlayIsJoined() ? View.VISIBLE : View.GONE);

        if (mMessageList == null) {
            mMessageList = new ArrayList<>();
            mTaskHandler = new Handler(getMainLooper());
        }
        mMessageList.add(msg);
        if (mMessageList.size() > 10) {
            mMessageList.remove(0);
        }
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < mMessageList.size(); ++i) {
            sb.append(mMessageList.get(i));
            sb.append(System.lineSeparator());
        }
        mNetPlayMessage.setText(sb.toString());
        mNetPlayMessage.setVisibility(View.VISIBLE);

        mTaskHandler.removeCallbacksAndMessages(null);
        mTaskHandler.postDelayed(() -> {
            mNetPlayMessage.setVisibility(View.INVISIBLE);
            if (mMessageList != null) {
                mMessageList.clear();
            }
        }, 6 * 1000);
    }

    public void updateProgress(String name, long written, long total) {
        mProgressBar.setVisibility(written < total ? View.VISIBLE : View.INVISIBLE);
    }

    public boolean isLassoOverlayEnabled() {
        return mLassoOverlay.getVisibility() == View.VISIBLE;
    }

    private void runWithValidSurface() {
        mRunWhenSurfaceIsValid = false;
        if (mState == EmulationState.STOPPED) {
            NativeLibrary.SurfaceChanged(mSurface);
            new Thread(() -> NativeLibrary.Run(mPath), "NativeEmulation").start();
        } else if (mState == EmulationState.PAUSED) {
            NativeLibrary.SurfaceChanged(mSurface);
            NativeLibrary.ResumeEmulation();
        }
        mState = EmulationState.RUNNING;
    }

    private enum EmulationState { STOPPED, RUNNING, PAUSED }
}
