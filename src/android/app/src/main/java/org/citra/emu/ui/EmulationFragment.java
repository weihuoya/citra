package org.citra.emu.ui;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.overlay.InputOverlay;

public final class EmulationFragment extends Fragment implements SurfaceHolder.Callback {
    private static final String KEY_GAMEPATH = "gamepath";
    private String mPath;
    private Surface mSurface;
    private EmulationState mState;
    private boolean mRunWhenSurfaceIsValid;
    private InputOverlay mInputOverlay;
    private Button mBtnDone;

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

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View contents = inflater.inflate(R.layout.fragment_emulation, container, false);

        SurfaceView surfaceView = contents.findViewById(R.id.surface_emulation);
        surfaceView.getHolder().addCallback(this);

        mInputOverlay = contents.findViewById(R.id.surface_input_overlay);
        mBtnDone = contents.findViewById(R.id.done_control_config);
        mBtnDone.setOnClickListener(v -> stopConfiguringControls());

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
        super.onPause();
    }

    public void startConfiguringControls() {
        mBtnDone.setVisibility(View.VISIBLE);
        mInputOverlay.setInEditMode(true);
    }

    public void stopConfiguringControls() {
        mBtnDone.setVisibility(View.GONE);
        mInputOverlay.setInEditMode(false);
    }

    public void stopEmulation() {
        if (mState != EmulationState.STOPPED) {
            mState = EmulationState.STOPPED;
            NativeLibrary.StopEmulation();
        } else {
            // [EmulationFragment] Stop called while already stopped.
        }
    }

    private void runWithValidSurface() {
        mRunWhenSurfaceIsValid = false;
        if (mState == EmulationState.STOPPED) {
            new Thread(() -> {
                NativeLibrary.SurfaceChanged(mSurface);
                NativeLibrary.Run(mPath);
            }, "NativeEmulation").start();
        } else if (mState == EmulationState.PAUSED) {
            NativeLibrary.SurfaceChanged(mSurface);
            NativeLibrary.ResumeEmulation();
        } else {
            // [EmulationFragment] Bug, run called while already running.
        }
        mState = EmulationState.RUNNING;
    }

    private enum EmulationState { STOPPED, RUNNING, PAUSED }
}
