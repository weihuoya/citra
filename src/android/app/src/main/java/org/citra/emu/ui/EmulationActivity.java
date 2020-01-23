package org.citra.emu.ui;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
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
import java.io.IOException;
import java.io.InputStream;
import java.lang.ref.WeakReference;
import java.util.List;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.utils.ControllerMappingHelper;

public final class EmulationActivity extends AppCompatActivity {
    private static final String EXTRA_GAMEPATH = "SelectedGames";
    public static final int REQUEST_PICK_IMAGE = 1;
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
        // fullscreen on create
        enableFullscreenImmersive();
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
        menu.getItem(3).setVisible(false);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case R.id.menu_mem_region:
            MemoryActivity.launch(this);
            break;

        case R.id.menu_screen_rotation:
            if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT) {
                setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            } else {
                setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
            }
            break;

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

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == REQUEST_PICK_IMAGE && resultCode == Activity.RESULT_OK) {
            if (data == null || data.getData() == null) {
                return;
            }

            InputStream inputStream;
            try {
                inputStream = getContentResolver().openInputStream(data.getData());
            } catch (IOException e) {
                return;
            }

            Bitmap image = BitmapFactory.decodeStream(inputStream);
            int srcWidth = image.getWidth();
            int srcHeight = image.getHeight();
            int dstWidth = data.getIntExtra("width", image.getWidth());
            int dstHeight = data.getIntExtra("height", image.getHeight());
            int[] pixels = new int[dstWidth * dstHeight];
            if (dstWidth != srcWidth || dstHeight != srcHeight) {
                Matrix matrix = new Matrix();
                matrix.postScale(dstWidth / (float)srcWidth, dstHeight / (float)srcHeight);
                image = Bitmap.createBitmap(image, 0, 0, srcWidth, srcHeight, matrix, true);
            }
            image.getPixels(pixels, 0, dstWidth, 0, 0, dstWidth, dstHeight);
            NativeLibrary.HandleImage(pixels, dstWidth, dstHeight);
        }
    }

    public void pickImage(int width, int height) {
        Intent intent = new Intent();
        intent.setType("image/*");
        intent.putExtra("width", width);
        intent.putExtra("height", height);
        intent.setAction(Intent.ACTION_GET_CONTENT);
        startActivityForResult(Intent.createChooser(intent, "Select Picture"), REQUEST_PICK_IMAGE);
    }

    public void showInputBoxDialog(int maxLength, String error, String hint, String button0, String button1,
                                   String button2) {
        KeyboardDialog.newInstance(maxLength, error, hint, button0, button1, button2)
            .show(getSupportFragmentManager(), "KeyboardDialog");
    }

    public void showMiiSelectorDialog(boolean cancel, String title, String[] miis) {
        MiiSelectorDialog.newInstance(cancel, title, miis)
            .show(getSupportFragmentManager(), "MiiSelectorDialog");
    }

    public void refreshControls() {
        mEmulationFragment.refreshControls();
    }
}
