package org.citra.emu.ui;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import java.lang.ref.WeakReference;
import java.util.List;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.utils.ControllerMappingHelper;

public final class EmulationActivity extends AppCompatActivity {
    private static final String EXTRA_GAMEPATH = "SelectedGames";
    private static WeakReference<EmulationActivity> sInstance = new WeakReference<>(null);
    private String mPath;
    private View mDecorView;
    private boolean mMenuVisible;
    private boolean mStopEmulation;
    private EmulationFragment mEmulationFragment;

    public static void launch(Context context, String path) {
        Intent intent = new Intent(context, EmulationActivity.class);
        intent.putExtra(EXTRA_GAMEPATH, path);
        context.startActivity(intent);
    }

    public static EmulationActivity get() {
        return sInstance.get();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_emulation);
        sInstance = new WeakReference<>(this);

        if (savedInstanceState == null) {
            Intent intent = getIntent();
            mPath = intent.getStringExtra(EXTRA_GAMEPATH);
        } else {
            mPath = savedInstanceState.getString(EXTRA_GAMEPATH);
        }

        // Get a handle to the Window containing the UI.
        mDecorView = getWindow().getDecorView();
        mDecorView.setOnSystemUiVisibilityChangeListener(visibility -> {
            if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
                // Go back to immersive fullscreen mode in 3s
                Handler handler = new Handler(getMainLooper());
                handler.postDelayed(this ::enableFullscreenImmersive, 3000 /* 3s */);
            }
        });
        mStopEmulation = false;

        // Find or create the EmulationFragment
        mEmulationFragment = (EmulationFragment)getSupportFragmentManager().findFragmentById(
            R.id.fragment_emulation);
        if (mEmulationFragment == null) {
            mEmulationFragment = EmulationFragment.newInstance(mPath);
            getSupportFragmentManager()
                .beginTransaction()
                .add(R.id.fragment_emulation, mEmulationFragment)
                .commit();
        }

        if (mPath != null) {
            setTitle(NativeLibrary.GetAppTitle(mPath));
        }
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        outState.putString(EXTRA_GAMEPATH, mPath);
        super.onSaveInstanceState(outState);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_emulation, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case R.id.menu_emulation_screenshot:
            NativeLibrary.SaveScreenShot();
            return true;

        case R.id.menu_running_setting:
            RunningSettingDialog.newInstance().show(getSupportFragmentManager(),
                                                    "RunningSettingDialog");
            return true;

        case R.id.menu_emulation_edit_layout:
            mEmulationFragment.startConfiguringControls();
            return true;
        }

        return false;
    }

    private void enableFullscreenImmersive() {
        if (mStopEmulation) {
            return;
        }
        mMenuVisible = false;
        // It would be nice to use IMMERSIVE_STICKY, but that doesn't show the toolbar.
        mDecorView.setSystemUiVisibility(
            View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
            View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION |
            View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_IMMERSIVE);
    }

    private void disableFullscreenImmersive() {
        mMenuVisible = true;
        mDecorView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_STABLE |
                                         View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
                                         View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN);
    }

    @Override
    public void onBackPressed() {
        if (mMenuVisible) {
            mStopEmulation = true;
            mEmulationFragment.stopEmulation();
            finish();
        } else {
            disableFullscreenImmersive();
        }
    }

    // Gets button presses
    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {
        if (mMenuVisible) {
            return super.dispatchKeyEvent(event);
        }

        InputDevice input = event.getDevice();
        int button = event.getKeyCode();
        int action;
        switch (event.getAction()) {
        case KeyEvent.ACTION_DOWN:
            // Handling the case where the back button is pressed.
            if (button == KeyEvent.KEYCODE_BACK) {
                onBackPressed();
                return true;
            }
            // Normal key events.
            action = NativeLibrary.ButtonState.PRESSED;
            break;
        case KeyEvent.ACTION_UP:
            action = NativeLibrary.ButtonState.RELEASED;
            break;
        default:
            return false;
        }

        if (input != null)
            return NativeLibrary.KeyEvent(button, action);
        else
            return false;
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent event) {
        if (mMenuVisible) {
            return false;
        }

        if (((event.getSource() & InputDevice.SOURCE_CLASS_JOYSTICK) == 0)) {
            return super.dispatchGenericMotionEvent(event);
        }

        // Don't attempt to do anything if we are disconnecting a device.
        if (event.getActionMasked() == MotionEvent.ACTION_CANCEL)
            return true;

        InputDevice input = event.getDevice();
        List<InputDevice.MotionRange> motions = input.getMotionRanges();

        for (InputDevice.MotionRange range : motions) {
            int axis = range.getAxis();
            float origValue = event.getAxisValue(axis);
            float value = ControllerMappingHelper.scaleAxis(input, axis, origValue);
            // If the input is still in the "flat" area, that means it's really zero.
            // This is used to compensate for imprecision in joysticks.
            if (Math.abs(value) > range.getFlat()) {
                NativeLibrary.MoveEvent(axis, value);
            } else {
                NativeLibrary.MoveEvent(axis, 0.0f);
            }
        }

        return true;
    }

    public void showInputBoxDialog(int maxLength, String hint, String button0, String button1,
                                   String button2) {
        KeyboardDialog.newInstance(maxLength, hint, button0, button1, button2)
            .show(getSupportFragmentManager(), "KeyboardDialog");
    }

    public void showMiiSelectorDialog(boolean cancel, String title, String[] miis) {
        // todo
    }

    public void refreshControls() {
        mEmulationFragment.refreshControls();
    }
}
